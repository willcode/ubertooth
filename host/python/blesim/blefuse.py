#!/usr/bin/env python3
"""
blefuse - fuse an Ubertooth connection capture with a HackRF IQ recording.

The Ubertooth follows a connection on LE 1M (CONNECT_IND, hopping,
LL control) but cannot receive LE 2M. The HackRF records ~10 BLE channels
of IQ at any PHY. Run both at the same time, then:

    blefuse.py ubertooth.pcapng hackrf.iq --fc 2431 -o fused.pcapng

1. take the connection parameters from the CONNECT_IND in the Ubertooth
   capture (access address, CRCInit, interval, channel map, CSA)
2. decode the HackRF capture for that access address at 1M and 2M
3. align the two clocks on packets both radios received, or, if there are
   none, on the connection's event lattice and hop sequence
4. label every packet with its connection event counter, direction
   (master/slave), PHY, and whether it is encrypted
5. write one pcapng (Wireshark decodes the LE 2M packets) and print a
   traffic analysis summary; encrypted traffic still shows timing,
   direction, sizes and LL control structure

Only channels inside the HackRF window are seen after a switch to 2M, so
the summary reports coverage: events observed vs. events whose channel
was inside the window.

This file is part of Project Ubertooth (GPL-2.0-or-later).
"""

import argparse
import collections
import json
import math
import struct
import sys

import numpy as np

import blesim
import blerx
import lepcapng

LL_OPCODES = {
    0x00: 'CONNECTION_UPDATE_IND', 0x01: 'CHANNEL_MAP_IND', 0x02: 'TERMINATE_IND',
    0x03: 'ENC_REQ', 0x04: 'ENC_RSP', 0x05: 'START_ENC_REQ', 0x06: 'START_ENC_RSP',
    0x07: 'UNKNOWN_RSP', 0x08: 'FEATURE_REQ', 0x09: 'FEATURE_RSP',
    0x0A: 'PAUSE_ENC_REQ', 0x0B: 'PAUSE_ENC_RSP', 0x0C: 'VERSION_IND',
    0x0D: 'REJECT_IND', 0x0E: 'PERIPHERAL_FEATURE_REQ', 0x0F: 'CONNECTION_PARAM_REQ',
    0x10: 'CONNECTION_PARAM_RSP', 0x11: 'REJECT_EXT_IND', 0x12: 'PING_REQ',
    0x13: 'PING_RSP', 0x14: 'LENGTH_REQ', 0x15: 'LENGTH_RSP', 0x16: 'PHY_REQ',
    0x17: 'PHY_RSP', 0x18: 'PHY_UPDATE_IND', 0x19: 'MIN_USED_CHANNELS_IND',
}

# plaintext LL control PDU lengths (opcode + CtrData), Core 6.0 Vol 6 Part B 2.4.2
LL_CTRL_LEN = {
    0x00: 12, 0x01: 8, 0x02: 2, 0x03: 23, 0x04: 13, 0x05: 1, 0x06: 1,
    0x07: 2, 0x08: 9, 0x09: 9, 0x0A: 1, 0x0B: 1, 0x0C: 6, 0x0D: 2, 0x0E: 9,
    0x0F: 24, 0x10: 24, 0x11: 3, 0x12: 1, 0x13: 1, 0x14: 9, 0x15: 9,
    0x16: 3, 0x17: 3, 0x18: 5, 0x19: 3,
}


def looks_encrypted(pdu):
    """True if a non-empty data PDU cannot be plaintext. An L2CAP start
    fragment carries its own length (PDU length - 4); an LL control PDU
    has a fixed length per opcode. Encryption adds a 4-byte MIC and
    scrambles both. Continuation fragments are not judged."""
    llid, n = pdu[0] & 3, pdu[1]
    if n == 0 or len(pdu) < 2 + n:
        return False
    body = pdu[2:2 + n]
    if llid == 2:
        return n < 4 or struct.unpack_from('<H', body)[0] != n - 4
    if llid == 3:
        return LL_CTRL_LEN.get(body[0]) != n
    return False



def parse_connect_ind(pdu):
    if len(pdu) < 2 + 34 or (pdu[0] & 0xf) != 5 or pdu[1] != 34:
        return None
    p = pdu[2:]
    return dict(
        chsel=(pdu[0] >> 5) & 1,
        inita=p[0:6], adva=p[6:12],
        aa=struct.unpack_from('<I', p, 12)[0],
        crc_init=p[16] | (p[17] << 8) | (p[18] << 16),
        win_size=p[19], win_offset=struct.unpack_from('<H', p, 20)[0],
        interval=struct.unpack_from('<H', p, 22)[0],
        latency=struct.unpack_from('<H', p, 24)[0],
        timeout=struct.unpack_from('<H', p, 26)[0],
        chm=int.from_bytes(p[28:33], 'little') & ((1 << 37) - 1),
        hop=p[33] & 0x1f, sca=p[33] >> 5)


def channel_for(conn, csa, counter):
    if csa == 2:
        return blesim.csa2(counter & 0xffff, conn['aa'], conn['chm'])
    used = blesim.used_channels(conn['chm'])
    u = ((counter + 1) * conn['hop']) % 37
    return u if (conn['chm'] >> u) & 1 else used[u % len(used)]


def fit_line(x, y):
    """Least squares y = a + b x."""
    x = np.asarray(x, dtype=np.float64)
    y = np.asarray(y, dtype=np.float64)
    if len(x) < 2:
        return (float(y[0] - x[0]) if len(x) else 0.0), 1.0
    b, a = np.polyfit(x - x[0], y, 1)
    return a - b * x[0], b


def align_common(ut, hk):
    """Map HackRF microseconds to Ubertooth nanoseconds using shared packets.

    Returns (a, b) with ut_ns = a + b * hk_us, or None.
    """
    by_key = collections.defaultdict(list)
    for p in ut:
        by_key[(p['chan'], p['pdu'])].append(p['ts_ns'])
    offs = []
    for h in hk:
        for t in by_key.get((h['chan'], bytes.fromhex(h['pdu'] + h['crc'])), []):
            offs.append((t - h['t_us'] * 1000.0, h['t_us'], t))
    if len(offs) < 2:
        return None
    # mode of the offsets (50 us bins), then a linear fit on the inliers
    o = np.array([x[0] for x in offs])
    bins = np.round(o / 50e3)
    vals, counts = np.unique(bins, return_counts=True)
    mode = vals[np.argmax(counts)] * 50e3
    inl = [x for x in offs if abs(x[0] - mode) < 100e3]
    if len(inl) < 2:
        return None
    a, b = fit_line([x[1] for x in inl], [x[2] for x in inl])
    res = np.array([x[2] - (a + b * x[1]) for x in inl])
    align_common.residual_ns = (float(np.sqrt(np.mean(res ** 2))), float(np.max(np.abs(res))))
    return a, b, len(inl)


def align_lattice(conn, csa, anchor0_ns, interval_ns, hk):
    """Align on the event lattice: vote for offsets that put HackRF packets
    on an event whose channel matches. Returns (a, 1.0, votes) or None."""
    votes = collections.Counter()
    data = [h for h in hk if h['aa'] == '%08x' % conn['aa']]
    if not data:
        return None
    t0 = data[0]['t_us']
    span = (data[-1]['t_us'] - t0) * 1000.0
    for h in data[:200]:
        # any event k whose channel is h's channel is a candidate
        for k in range(0, 20000):
            if channel_for(conn, csa, k) != h['chan']:
                continue
            off = anchor0_ns + k * interval_ns - h['t_us'] * 1000.0
            votes[round(off / 20e3)] += 1
    if not votes:
        return None
    best, n = votes.most_common(1)[0]
    return best * 20e3, 1.0, n


def track_events(conn, csa, pkts, win_start, interval_ns):
    """Label packets with event counter, direction and expected channel."""
    chm = conn['chm']
    interval = interval_ns
    # event 0 is in the transmit window; anchor on the first packet
    k_ref, t_ref = 0, None
    pending = []          # (instant, kind, value)
    last_k, n_in_event = None, 0
    chm_at = {}           # counter -> map in force, for the hop check
    for p in pkts:
        t = p['ts_ns']
        if t_ref is None:
            k_ref = max(0, round((t - win_start) / interval))
            t_ref = t
        k = k_ref + round((t - t_ref) / interval)
        # apply updates whose instant has been reached
        for inst, kind, val in sorted(pending):
            if k < inst:
                continue
            pending.remove((inst, kind, val))
            if kind == 'chm':
                chm = val
            elif kind == 'conn':
                new_interval, win_offset = val
                t_inst = t_ref + (inst - k_ref) * interval   # old timing
                interval = new_interval
                k_ref, t_ref = inst, t_inst + win_offset
                k = k_ref + round((t - t_ref) / interval)
        if k != last_k:
            n_in_event = 0
            last_k = k
            # the first packet of an event is the master's: re-anchor
            k_ref, t_ref = k, t
        p['counter'] = k
        p['slave'] = n_in_event % 2 == 1
        n_in_event += 1
        p['expected_chan'] = channel_for(dict(conn, chm=chm), csa, k)
        # plaintext LL control PDUs that change the connection
        pdu = p['pdu']
        if (pdu[0] & 3) == 3 and pdu[1] >= 1 and not p.get('encrypted'):
            if pdu[2] == 0x00 and pdu[1] == 12:
                win_off = struct.unpack_from('<H', pdu, 4)[0] * 1.25e6
                iv = struct.unpack_from('<H', pdu, 6)[0] * 1.25e6
                inst = struct.unpack_from('<H', pdu, 12)[0]
                if not any(x[0] == inst and x[1] == 'conn' for x in pending):
                    pending.append((inst, 'conn', (iv, win_off)))
            elif pdu[2] == 0x01 and pdu[1] == 8:
                new = int.from_bytes(pdu[3:8], 'little') & ((1 << 37) - 1)
                inst = struct.unpack_from('<H', pdu, 8)[0]
                if not any(x[0] == inst and x[1] == 'chm' for x in pending):
                    pending.append((inst, 'chm', new))


def analyze(args):
    ut = []
    for p in lepcapng.read(args.ubertooth):
        if p['rf_channel'] is None:
            continue
        ch = lepcapng.rf_index_to_chan(p['rf_channel'])
        ut.append(dict(ts_ns=p['ts_ns'], chan=ch, aa=p['aa'], pdu=p['pdu'],
                       signal=p['signal'], phy='1M', src='ubertooth'))
    ut.sort(key=lambda p: p['ts_ns'])

    # connection parameters
    conn, conn_ts = None, None
    adv_chsel = {}
    for p in ut:
        if p['aa'] != blesim.ADV_AA:
            continue
        t = p['pdu'][0] & 0xf
        if t in (0, 1) and len(p['pdu']) >= 8:
            adv_chsel[bytes(p['pdu'][2:8])] = (p['pdu'][0] >> 5) & 1
        c = parse_connect_ind(p['pdu'])
        if c and (args.aa is None or c['aa'] == args.aa):
            conn, conn_ts = c, p['ts_ns']
            if not args.last:
                break
    if conn is None:
        sys.exit('no CONNECT_IND in %s' % args.ubertooth)

    if args.csa:
        csa = args.csa
    elif not conn['chsel']:
        csa = 1
    else:
        csa = 2 if adv_chsel.get(bytes(conn['adva']), 1) else 1

    interval_ns = conn['interval'] * 1.25e6
    pdu_len = 2 + 34 + 3
    win_start = conn_ts + (40 + pdu_len * 8) * 1000 + 1.25e6 + conn['win_offset'] * 1.25e6

    # Ubertooth data packets of this connection
    ut_data = [p for p in ut if p['aa'] == conn['aa']]

    # HackRF decode
    hk = blerx.decode_capture(args.hackrf, args.fc, args.fs, [(conn['aa'], conn['crc_init'])],
                              thresh_db=args.thresh_db)

    # clock alignment
    al = align_common(ut, hk)
    how = 'shared packets'
    if al is None:
        anchor0 = ut_data[0]['ts_ns'] if ut_data else win_start
        al = align_lattice(conn, csa, anchor0, interval_ns, hk)
        how = 'event lattice'
    if al is None:
        sys.exit('could not align the HackRF capture with the Ubertooth capture')
    a, b, support = al
    for h in hk:
        h['ts_ns'] = int(a + b * h['t_us'])

    # event lattice: anchors from master packets (first packet of each event)
    pkts = []
    for p in ut_data:
        pkts.append(dict(ts_ns=p['ts_ns'], chan=p['chan'], pdu=p['pdu'], phy='1M',
                         crc_ok=None, src='ubertooth', signal=p['signal']))
    keyset = {(p['chan'], bytes(p['pdu'])) for p in pkts}
    for h in hk:
        if h['aa'] != '%08x' % conn['aa']:
            continue
        pdu = bytes.fromhex(h['pdu'] + h['crc'])
        dup = [p for p in pkts if p['chan'] == h['chan'] and p['pdu'] == pdu and
               abs(p['ts_ns'] - h['ts_ns']) < 100e3]
        if dup:
            dup[0]['hackrf_too'] = True
            continue
        pkts.append(dict(ts_ns=h['ts_ns'], chan=h['chan'], pdu=pdu, phy=h['phy'],
                         crc_ok=h['crc_ok'], src='hackrf', signal=h['power_db']))
    pkts.sort(key=lambda p: p['ts_ns'])

    # assign events with a software follower: walk the packets in time
    # order, re-anchor on each event's first (master) packet, apply
    # LL_CONNECTION_UPDATE_IND / LL_CHANNEL_MAP_IND at their instants
    track_events(conn, csa, pkts, win_start, interval_ns)

    # encryption and PHY changes from LL control (and from the data:
    # LE 2M packets mean the PHY changed even if the update was encrypted)
    enc_from = None
    enc_index = None   # the LL_START_ENC_REQ packet: everything after is encrypted
    phy_change = None
    for i, p in enumerate(pkts):
        pdu = p['pdu']
        if (pdu[0] & 3) == 3 and pdu[1] >= 1:
            op = pdu[2]
            if op == 0x05 and pdu[1] == 1 and enc_from is None:
                enc_from = p['counter']
                enc_index = i
            if op == 0x18 and pdu[1] == 5 and phy_change is None:
                phy_change = (struct.unpack_from('<H', pdu, 5)[0], pdu[3], pdu[4])
    if phy_change is None:
        first_2m = next((p['counter'] for p in pkts if p['phy'] == '2M'), None)
        if first_2m is not None:
            phy_change = (first_2m, 2, 2)
    enc_inferred = False
    if enc_index is None:
        # LL_START_ENC_REQ not captured (e.g. its channel was outside the
        # HackRF window): after an LL_ENC_REQ/RSP, encryption starts at
        # the first packet that cannot be plaintext
        seen_enc = None
        for i, p in enumerate(pkts):
            pdu = p['pdu']
            if (pdu[0] & 3) == 3 and pdu[1] >= 1 and pdu[2] in (0x03, 0x04) \
                    and pdu[1] == LL_CTRL_LEN[pdu[2]]:
                seen_enc = i
            elif seen_enc is not None and looks_encrypted(pdu):
                enc_index = i - 1
                enc_from = p['counter']
                enc_inferred = True
                break
    for i, p in enumerate(pkts):
        p['encrypted'] = enc_index is not None and i > enc_index and p['pdu'][1] > 0
        p['llid'] = p['pdu'][0] & 3
    events = collections.OrderedDict()
    for p in pkts:
        events.setdefault(p['counter'], []).append(p)

    # outputs
    if args.output:
        w = lepcapng.Writer(args.output)
        for p in ut:
            if p['aa'] == blesim.ADV_AA:
                w.packet(int(p['ts_ns']), p['chan'], p['aa'], p['pdu'], '1M',
                         crc_ok=blesim.crc24(blesim.ADV_CRC_INIT, p['pdu'][:-3]) == p['pdu'][-3:],
                         signal=p['signal'], ref_aa=blesim.ADV_AA)
        for p in pkts:
            pdu = p['pdu']
            crc_ok = blesim.crc24(conn['crc_init'], pdu[:-3]) == pdu[-3:]
            w.packet(int(p['ts_ns']), p['chan'], conn['aa'], pdu, p['phy'],
                     crc_ok=crc_ok, signal=p['signal'], ref_aa=conn['aa'],
                     slave=p['slave'])
        w.close()

    return conn, csa, how, support, b, pkts, events, (enc_from, enc_inferred), phy_change


def summarize(conn, csa, how, support, drift, pkts, events, enc_from, phy_change, args):
    out = []
    out.append('connection  AA %08x  CRCInit %06x  interval %.2f ms  CSA#%d  map %d channels'
               % (conn['aa'], conn['crc_init'], conn['interval'] * 1.25, csa,
                  len(blesim.used_channels(conn['chm']))))
    out.append('alignment   %s (%d votes/packets), HackRF clock %+.1f ppm vs Ubertooth'
               % (how, support, (drift - 1000.0) / 1000.0 * 1e6 if drift > 100 else (drift - 1) * 1e6))
    if how == 'shared packets' and getattr(align_common, 'residual_ns', None):
        out.append('            residual rms %.2f us, max %.2f us' % (
            align_common.residual_ns[0] / 1e3, align_common.residual_ns[1] / 1e3))
    if phy_change:
        out.append('PHY update  at event %d  M->S %s  S->M %s' % (
            phy_change[0], {1: '1M', 2: '2M', 4: 'Coded'}.get(phy_change[1], phy_change[1]),
            {1: '1M', 2: '2M', 4: 'Coded'}.get(phy_change[2], phy_change[2])))
    enc_from, enc_inferred = enc_from
    if enc_from is not None:
        out.append('encryption  starts at event %d%s' % (
            enc_from, ' (inferred: LL_START_ENC_REQ not captured)' if enc_inferred else ''))

    by_src = collections.Counter((p['src'], p['phy']) for p in pkts)
    out.append('packets     ' + ', '.join('%s %s: %d' % (s, ph, n) for (s, ph), n in sorted(by_src.items())))

    # coverage after the last Ubertooth packet (the 2M phase)
    last_ut = max((p['counter'] for p in pkts if p['src'] == 'ubertooth'), default=None)
    hk_events = sorted({p['counter'] for p in pkts if p['src'] == 'hackrf' and
                        (last_ut is None or p['counter'] > last_ut)})
    if hk_events:
        lo, hi = hk_events[0], hk_events[-1]
        fc = args.fc * 1e6
        in_window = [k for k in range(lo, hi + 1)
                     if abs(blesim.chan_idx_to_mhz(channel_for(conn, csa, k)) * 1e6 - fc) + 1.2e6
                     < args.fs / 2]
        out.append('HackRF-only events %d..%d: observed %d of %d in window (%d total, %.0f%% of events)'
                   % (lo, hi, len(hk_events), len(in_window), hi - lo + 1,
                      100.0 * len(hk_events) / (hi - lo + 1)))
    wrong = sum(1 for p in pkts if p['chan'] != p['expected_chan'])
    out.append('hop check   %d of %d packets on the predicted channel'
               % (len(pkts) - wrong, len(pkts)))

    # traffic analysis
    for side in (False, True):
        ps = [p for p in pkts if p['slave'] == side]
        nonempty = [p for p in ps if p['pdu'][1] > 0]
        sizes = collections.Counter(p['pdu'][1] for p in nonempty)
        llid = collections.Counter({1: 'cont/empty', 2: 'start', 3: 'control'}.get(p['llid'], '?')
                                   for p in nonempty)
        out.append('%-6s      %d packets, %d non-empty, %d encrypted; LLID %s; lengths %s'
                   % ('slave' if side else 'master', len(ps), len(nonempty),
                      sum(1 for p in ps if p['encrypted']), dict(llid),
                      dict(sorted(sizes.items()))))
    ctrl = collections.Counter()
    for p in pkts:
        if p['llid'] == 3 and p['pdu'][1] > 0:
            name = ('encrypted, %d bytes' % p['pdu'][1]) if p['encrypted'] else \
                LL_OPCODES.get(p['pdu'][2], '0x%02x' % p['pdu'][2])
            ctrl[('S ' if p['slave'] else 'M ') + name] += 1
    if ctrl:
        out.append('LL control  ' + ', '.join('%s x%d' % kv for kv in sorted(ctrl.items())))
    return '\n'.join(out)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('ubertooth', help='pcapng from ubertooth-btle -f -r')
    ap.add_argument('hackrf', help='int8 IQ from hackrf_transfer -r')
    ap.add_argument('--fc', type=float, required=True, help='HackRF center MHz')
    ap.add_argument('--fs', type=float, default=20e6)
    ap.add_argument('--aa', type=lambda s: int(s, 16), default=None,
                    help='connection access address (default: first CONNECT_IND)')
    ap.add_argument('--last', action='store_true', help='use the last CONNECT_IND')
    ap.add_argument('--csa', type=int, choices=(1, 2), default=None)
    ap.add_argument('--thresh-db', type=float, default=5.0)
    ap.add_argument('-o', '--output', help='fused pcapng')
    ap.add_argument('--json', help='write per-packet labels as JSON lines')
    args = ap.parse_args()

    res = analyze(args)
    conn, csa, how, support, drift, pkts, events, enc_from, phy_change = res
    print(summarize(*res, args))
    if args.json:
        with open(args.json, 'w') as f:
            for p in pkts:
                f.write(json.dumps(dict(ts_ns=int(p['ts_ns']), counter=p['counter'],
                                        chan=p['chan'], phy=p['phy'], src=p['src'],
                                        slave=p['slave'], encrypted=p['encrypted'],
                                        llid=p['llid'], length=p['pdu'][1],
                                        pdu=p['pdu'].hex())) + '\n')


if __name__ == '__main__':
    main()
