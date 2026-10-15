#!/usr/bin/env python3
"""
blesim - synthesize Bluetooth LE baseband for a HackRF (or any IQ sink).

Generates bit-exact BLE link-layer packets (CRC, whitening, CSA#1/CSA#2
hopping, LE 1M / 2M / Coded PHY GFSK) and renders scenario timelines to
interleaved int8 IQ suitable for `hackrf_transfer -t`.

The HackRF covers only a slice of the 2.4 GHz band, so scenarios place
the advertising channel and the data channel map inside that slice.  With
the defaults (fc = 2431 MHz, fs = 20 Msps) advertising channel 38
(2426 MHz) and data channels 10..16 (2424..2438 MHz) are covered, which is
enough for a complete connection: CONNECT_IND, hopping data events,
LL control procedures, encryption start, and a PHY update.

Every scenario also writes a ground-truth JSON file describing each
packet transmitted, so a capture can be scored against it.

This file is part of Project Ubertooth (GPL-2.0-or-later).
"""

import argparse
import json
import math
import os
import random
import struct
import sys

import numpy as np

ADV_AA = 0x8E89BED6
ADV_CRC_INIT = 0x555555

# ---------------------------------------------------------------------------
# channels
# ---------------------------------------------------------------------------


def chan_idx_to_mhz(idx):
    if idx < 11:
        return 2404 + 2 * idx
    if idx < 37:
        return 2428 + 2 * (idx - 11)
    return {37: 2402, 38: 2426, 39: 2480}[idx]


# ---------------------------------------------------------------------------
# CRC / whitening (bit order: LSB of each byte first on air)
# ---------------------------------------------------------------------------


def crc24(crc_init, data):
    """BLE CRC.  Returns the 3 CRC bytes in on-air order."""
    # LFSR implementation per Core Spec Vol 6 Part B 3.1.1, operating on the
    # bit-reversed state so bytes can be processed LSB first.
    state = 0
    for i in range(24):  # reverse crc_init bits
        state |= ((crc_init >> i) & 1) << (23 - i)
    for byte in data:
        for _ in range(8):
            nb = (state ^ byte) & 1
            byte >>= 1
            state >>= 1
            if nb:
                state |= 1 << 23
                state ^= 0x5A6000
    return bytes([state & 0xFF, (state >> 8) & 0xFF, (state >> 16) & 0xFF])


def whiten_bits(bits, chan_idx):
    """XOR a bit list with the whitening sequence for chan_idx."""
    # 7-bit LFSR x^7 + x^4 + 1; position 0 = 1, positions 1..6 = channel
    # index MSB first.
    reg = [1] + [(chan_idx >> (5 - i)) & 1 for i in range(6)]
    out = []
    for b in bits:
        w = reg[6]
        out.append(b ^ w)
        # shift: new position0 = old position6, position4 ^= old position6
        reg = [w] + reg[:6]
        reg[4] ^= w
    return out


def bytes_to_bits(data):
    return [(byte >> i) & 1 for byte in data for i in range(8)]


def u32_bits(v):
    return [(v >> i) & 1 for i in range(32)]


# ---------------------------------------------------------------------------
# channel selection algorithms
# ---------------------------------------------------------------------------


def used_channels(chm):
    """chm: 37-bit int channel map -> sorted list of used channel indices."""
    return [i for i in range(37) if (chm >> i) & 1]


def csa1(last_unmapped, hop, chm):
    """Returns (unmapped, channel) for the next event under CSA#1."""
    unmapped = (last_unmapped + hop) % 37
    used = used_channels(chm)
    if (chm >> unmapped) & 1:
        return unmapped, unmapped
    return unmapped, used[unmapped % len(used)]


def _perm(v):
    # bit-reverse each byte of a 16-bit value
    lo = int('{:08b}'.format(v & 0xFF)[::-1], 2)
    hi = int('{:08b}'.format((v >> 8) & 0xFF)[::-1], 2)
    return (hi << 8) | lo


def csa2_prn_e(counter, chan_id):
    prn = (counter ^ chan_id) & 0xFFFF
    for _ in range(3):
        prn = _perm(prn)
        prn = (17 * prn + chan_id) & 0xFFFF
    return prn ^ chan_id


def csa2(counter, aa, chm):
    chan_id = ((aa >> 16) ^ aa) & 0xFFFF
    prn_e = csa2_prn_e(counter, chan_id)
    unmapped = prn_e % 37
    if (chm >> unmapped) & 1:
        return unmapped
    used = used_channels(chm)
    return used[(len(used) * prn_e) >> 16]


# ---------------------------------------------------------------------------
# PDUs
# ---------------------------------------------------------------------------


def adv_header(pdu_type, length, txadd=0, rxadd=0, chsel=0):
    return bytes([(pdu_type & 0xF) | (chsel << 5) | (txadd << 6) | (rxadd << 7),
                  length])


def addr_bytes(addr_str):
    """'aa:bb:cc:dd:ee:ff' -> 6 bytes little endian (on-air order)."""
    return bytes(int(x, 16) for x in reversed(addr_str.split(':')))


def ad_name(name):
    n = name.encode()
    return bytes([1 + len(n), 0x09]) + n


def ad_flags(flags=0x06):
    return bytes([2, 0x01, flags])


def adv_ind(adva, adv_data, txadd=1, chsel=0):
    payload = addr_bytes(adva) + adv_data
    return adv_header(0x0, len(payload), txadd=txadd, chsel=chsel) + payload


def connect_ind(inita, adva, aa, crc_init, win_size, win_offset, interval,
                latency, timeout, chm, hop, sca=0, chsel=1, txadd=1, rxadd=1):
    payload = (addr_bytes(inita) + addr_bytes(adva) + struct.pack('<I', aa)
               + struct.pack('<I', crc_init)[:3]
               + struct.pack('<BHHHH', win_size, win_offset, interval,
                             latency, timeout)
               + chm.to_bytes(5, 'little')
               + bytes([(hop & 0x1F) | ((sca & 7) << 5)]))
    assert len(payload) == 34
    return adv_header(0x5, 34, txadd=txadd, rxadd=rxadd, chsel=chsel) + payload


def data_pdu(llid, payload, nesn, sn, md=0):
    return bytes([(llid & 3) | (nesn << 2) | (sn << 3) | (md << 4),
                  len(payload)]) + payload


def aux_ptr(chan_idx, offset_us, phy=0):
    """AuxPtr field.  Uses 30 us units when possible."""
    if offset_us < 245700:
        units, off = 0, offset_us // 30
    else:
        units, off = 1, offset_us // 300
    assert off < (1 << 13)
    return bytes([(chan_idx & 0x3F) | (units << 7),
                  off & 0xFF, ((off >> 8) & 0x1F) | ((phy & 7) << 5)])


def ext_adv_pdu(adv_mode, adva=None, adi=None, auxptr=None, txpower=None,
                adv_data=b'', txadd=1):
    flags = 0
    fields = b''
    if adva is not None:
        flags |= 0x01
        fields += addr_bytes(adva)
    if adi is not None:
        flags |= 0x08
        fields += struct.pack('<H', adi)
    if auxptr is not None:
        flags |= 0x10
        fields += auxptr
    if txpower is not None:
        flags |= 0x40
        fields += struct.pack('<b', txpower)
    ext_hdr = bytes([flags]) + fields if flags else b''
    body = bytes([(len(ext_hdr) & 0x3F) | (adv_mode << 6)]) + ext_hdr + adv_data
    return adv_header(0x7, len(body), txadd=txadd if adva else 0) + body


# ---------------------------------------------------------------------------
# PHY: bit streams
# ---------------------------------------------------------------------------


def frame_bits_uncoded(aa, pdu, crc_init, chan_idx, phy='1M'):
    """Preamble + AA + whitened(PDU + CRC) for LE 1M / 2M."""
    crc = crc24(crc_init, pdu)
    body = whiten_bits(bytes_to_bits(pdu + crc), chan_idx)
    aa_bits = u32_bits(aa)
    first = aa_bits[0]
    nbits = 8 if phy == '1M' else 16
    # preamble alternates and its last bit differs from the AA's first bit
    pre = [(first ^ 1 ^ ((nbits - 1 - i) & 1)) for i in range(nbits)]
    return pre + aa_bits + body


def _conv_encode(bits):
    # K = 4, G0 = 1 + D + D^2 + D^3, G1 = 1 + D^2 + D^3
    s = [0, 0, 0]
    out = []
    for b in bits:
        out.append(b ^ s[0] ^ s[1] ^ s[2])
        out.append(b ^ s[1] ^ s[2])
        s = [b, s[0], s[1]]
    return out


def _pattern_map(coded, s):
    if s == 2:
        return coded
    out = []
    for c in coded:
        out += [1, 1, 0, 0] if c else [0, 0, 1, 1]
    return out


def frame_bits_coded(aa, pdu, crc_init, chan_idx, s=8):
    """LE Coded PHY symbol stream (1 Msym/s)."""
    pre = [0, 0, 1, 1, 1, 1, 0, 0] * 10
    ci = [0, 0] if s == 8 else [1, 0]
    blk1 = _pattern_map(_conv_encode(u32_bits(aa) + ci + [0, 0, 0]), 8)
    crc = crc24(crc_init, pdu)
    body = whiten_bits(bytes_to_bits(pdu + crc), chan_idx)
    blk2 = _pattern_map(_conv_encode(body + [0, 0, 0]), s)
    return pre + blk1 + blk2


# ---------------------------------------------------------------------------
# PHY: GFSK
# ---------------------------------------------------------------------------


def _gauss_taps(sps, bt=0.5, span=3):
    t = np.arange(-span * sps / 2, span * sps / 2 + 1) / sps
    alpha = math.sqrt(math.log(2) / 2) / bt
    h = (math.sqrt(math.pi) / alpha) * np.exp(-(math.pi * t / alpha) ** 2)
    return h / h.sum()


def gfsk(bits, fs, rate, h=0.5, bt=0.5):
    """Complex baseband GFSK at sample rate fs for symbol rate `rate`."""
    sps = fs / rate
    n = int(round(len(bits) * sps))
    idx = np.minimum((np.arange(n) / sps).astype(int), len(bits) - 1)
    nrz = np.array(bits, dtype=np.float64)[idx] * 2 - 1
    taps = _gauss_taps(int(round(sps)), bt)
    freq = np.convolve(nrz, taps, mode='same') * (h * rate / 2)
    phase = 2 * np.pi * np.cumsum(freq) / fs
    return np.exp(1j * phase)


# ---------------------------------------------------------------------------
# scenario timeline
# ---------------------------------------------------------------------------

PHY_RATE = {'1M': 1e6, '2M': 2e6, 'S8': 1e6, 'S2': 1e6}


class Timeline:
    def __init__(self, fc_mhz, fs):
        self.fc = fc_mhz * 1e6
        self.fs = fs
        self.events = []  # (t_us, chan_idx, phy, aa, pdu, crc_init, note)

    def add(self, t_us, chan_idx, aa, pdu, crc_init, phy='1M', note=''):
        f = chan_idx_to_mhz(chan_idx) * 1e6
        off = f - self.fc
        if abs(off) + 1.5e6 > self.fs / 2:
            raise ValueError('channel %d (%d MHz) outside the %.1f MHz window'
                             % (chan_idx, f / 1e6, self.fs / 1e6))
        self.events.append(dict(t_us=t_us, chan=chan_idx, phy=phy, aa=aa,
                                pdu=pdu, crc_init=crc_init, note=note))

    def duration_us(self, pdu_len, phy):
        if phy == '1M':
            return (1 + 4 + pdu_len + 3) * 8
        if phy == '2M':
            return (2 + 4 + pdu_len + 3) * 4
        s = 8 if phy == 'S8' else 2
        return 80 + 256 + 16 + 24 + (pdu_len + 3) * 8 * s + 3 * s

    def render(self, tail_us=2000, amplitude=0.5):
        end = max(e['t_us'] + self.duration_us(len(e['pdu']), e['phy'])
                  for e in self.events) + tail_us
        n = int(end * 1e-6 * self.fs)
        return self._render(self.events, 0, n, amplitude)

    def render_to_file(self, path, tail_us=2000, amplitude=0.5, chunk_s=1.0):
        """Render in chunks of chunk_s seconds (for long scenarios)."""
        end = max(e['t_us'] + self.duration_us(len(e['pdu']), e['phy'])
                  for e in self.events) + tail_us
        total = int(end * 1e-6 * self.fs)
        chunk = int(chunk_s * self.fs)
        evs = sorted(self.events, key=lambda e: e['t_us'])
        guard = int(3e-3 * self.fs)  # longest packet
        written = 0
        with open(path, 'wb') as f:
            i0 = 0
            for start in range(0, total, chunk):
                n = min(chunk, total - start)
                while i0 < len(evs) and evs[i0]['t_us'] * 1e-6 * self.fs < start - guard:
                    i0 += 1
                sel = [e for e in evs[i0:] if e['t_us'] * 1e-6 * self.fs < start + n]
                self._render(sel, start, n, amplitude).tofile(f)
                written += n
        return written

    def _render(self, events, offset, n, amplitude):
        iq = np.zeros(n, dtype=np.complex64)
        for e in events:
            phy = e['phy']
            if phy in ('1M', '2M'):
                bits = frame_bits_uncoded(e['aa'], e['pdu'], e['crc_init'],
                                          e['chan'], phy)
            else:
                bits = frame_bits_coded(e['aa'], e['pdu'], e['crc_init'],
                                        e['chan'], 8 if phy == 'S8' else 2)
            # ramp up/down over guard symbols outside the packet, as a
            # real transmitter does, so the first and last bits are intact
            guard = 4 if phy in ('1M', 'S8', 'S2') else 8
            sig = gfsk([bits[0]] * guard + bits + [bits[-1]] * guard,
                       self.fs, PHY_RATE[phy])
            g = int(round(guard * self.fs / PHY_RATE[phy]))
            w = np.ones(len(sig))
            w[:g] = np.linspace(0, 1, g)
            w[-g:] = np.linspace(1, 0, g)
            sig = sig * w
            pre = g  # the packet starts g samples into sig
            start = int(round(e['t_us'] * 1e-6 * self.fs)) - pre
            off = chan_idx_to_mhz(e['chan']) * 1e6 - self.fc
            t = (np.arange(len(sig)) + start) / self.fs
            sig = sig * np.exp(2j * np.pi * off * t)
            a = start - offset           # position in this chunk
            lo, hi = max(0, a), min(n, a + len(sig))
            if hi > lo:
                iq[lo:hi] += sig[lo - a:hi - a]
        iq *= amplitude * 127
        out = np.empty(2 * n, dtype=np.int8)
        out[0::2] = np.clip(np.round(iq.real), -127, 127)
        out[1::2] = np.clip(np.round(iq.imag), -127, 127)
        return out

    def truth(self):
        rows = []
        for e in sorted(self.events, key=lambda e: e['t_us']):
            rows.append(dict(t_us=e['t_us'], chan=e['chan'],
                             mhz=chan_idx_to_mhz(e['chan']), phy=e['phy'],
                             aa='%08x' % e['aa'], pdu=e['pdu'].hex(),
                             crc=crc24(e['crc_init'], e['pdu']).hex(),
                             note=e['note']))
        return rows


# ---------------------------------------------------------------------------
# scenarios
# ---------------------------------------------------------------------------

ADV_CHAN = 38
WINDOW_DATA_CHANNELS = list(range(10, 17))  # 2424..2438 MHz


def chm_from_list(chans):
    m = 0
    for c in chans:
        m |= 1 << c
    return m


def scenario_adv(tl, args):
    pdu = adv_ind(args.adva, ad_flags() + ad_name(args.name))
    t = 1000
    for i in range(args.count):
        tl.add(t, ADV_CHAN, ADV_AA, pdu, ADV_CRC_INIT, note='ADV_IND %d' % i)
        t += args.adv_interval_us


def scenario_extadv(tl, args):
    """ADV_EXT_IND on ch38 pointing at AUX_ADV_IND on a data channel."""
    t = 1000
    aux_chans = WINDOW_DATA_CHANNELS
    adv_data = ad_flags() + ad_name(args.name)
    for i in range(args.count):
        aux_chan = aux_chans[i % len(aux_chans)]
        offset_us = 600
        ptr = aux_ptr(aux_chan, offset_us, phy=0)
        adi = (0x1 << 12) | (i & 0xFFF)  # SID 1, DID i
        p = ext_adv_pdu(0, adi=adi, auxptr=ptr)
        tl.add(t, ADV_CHAN, ADV_AA, p, ADV_CRC_INIT, note='ADV_EXT_IND %d' % i)
        aux = ext_adv_pdu(0, adva=args.adva, adi=adi, txpower=-10,
                          adv_data=adv_data)
        # AUX offset is measured from the start of the ADV_EXT_IND packet;
        # the receiver window starts at the offset, so transmit 20 us late.
        tl.add(t + (offset_us // 30) * 30 + 20, aux_chan, ADV_AA, aux,
               ADV_CRC_INIT, note='AUX_ADV_IND %d -> ch%d' % (i, aux_chan))
        t += args.adv_interval_us


def scenario_coded(tl, args):
    """Legacy-style payload carried in ADV_EXT_IND on the Coded PHY."""
    adv_data = ad_flags() + ad_name(args.name)
    t = 1000
    for i in range(args.count):
        p = ext_adv_pdu(0, adva=args.adva, adv_data=adv_data)
        tl.add(t, ADV_CHAN, ADV_AA, p, ADV_CRC_INIT,
               phy='S8' if args.coding == 8 else 'S2',
               note='coded ADV_EXT_IND %d' % i)
        t += args.adv_interval_us


def scenario_conn(tl, args):
    """A complete connection whose channel map fits in the HackRF window.

    Timeline: a few ADV_IND, CONNECT_IND, data events with traffic,
    optional LL_CONNECTION_UPDATE_IND, LL_CHANNEL_MAP_IND, encryption
    start, and LL_PHY_UPDATE_IND to 2M (after which packets are 2M).
    """
    rnd = random.Random(args.seed)
    aa = args.aa if args.aa is not None else rnd.getrandbits(32)
    crc_init = rnd.getrandbits(24)
    chans = ([int(c) for c in args.chm.split(',')] if args.chm
             else WINDOW_DATA_CHANNELS)
    chm = chm_from_list(chans)
    hop = args.hop
    interval = args.interval  # units of 1.25 ms
    win_size, win_offset = 2, 1
    chsel = 1 if args.csa == 2 else 0

    t = 1000.0
    # CSA#2 is used when both the advertiser and the initiator set ChSel
    for i in range(3):
        tl.add(t, ADV_CHAN, ADV_AA,
               adv_ind(args.adva, ad_flags() + ad_name(args.name), chsel=chsel),
               ADV_CRC_INIT, note='ADV_IND')
        t += 20000
    # The CONNECT_IND is sent on the same advertising event right after
    # an ADV_IND (T_IFS later), as a real initiator would.
    adv_pdu = adv_ind(args.adva, ad_flags() + ad_name(args.name), chsel=chsel)
    tl.add(t, ADV_CHAN, ADV_AA, adv_pdu, ADV_CRC_INIT, note='ADV_IND')
    t += tl.duration_us(len(adv_pdu), '1M') + 150
    ci = connect_ind(args.inita, args.adva, aa, crc_init, win_size,
                     win_offset, interval, 0, args.timeout, chm, hop,
                     chsel=chsel)
    tl.add(t, ADV_CHAN, ADV_AA, ci, ADV_CRC_INIT,
           note='CONNECT_IND aa=%08x crc=%06x csa=%d hop=%d int=%d' %
           (aa, crc_init, args.csa, hop, interval))
    ci_end = t + tl.duration_us(len(ci), '1M')

    # first anchor: inside the transmit window
    anchor = ci_end + 1250 + win_offset * 1250 + 300
    interval_us = interval * 1250
    sn_m = nesn_m = sn_s = nesn_s = 0
    last_unmapped = 0
    counter = 0
    phy = '1M'
    encrypted = False
    pending = []  # (instant, kind, data)

    def m_s(payload_m, payload_s, llid_m=1, llid_s=1, note=''):
        nonlocal sn_m, nesn_m, sn_s, nesn_s
        pm = data_pdu(llid_m, payload_m, nesn_m, sn_m)
        ps = data_pdu(llid_s, payload_s, nesn_s, sn_s)
        sn_m ^= 1
        nesn_s ^= 1
        sn_s ^= 1
        nesn_m ^= 1
        return pm, ps

    def mic(p):
        return p + bytes(rnd.getrandbits(8) for _ in range(4)) if encrypted and p else p

    script = {}  # counter -> (master payload, llid, slave payload, llid, note)
    c = 3
    # some L2CAP / ATT style traffic
    script[c] = (bytes.fromhex('0700040010010010ff0028'), 2, b'', 1, 'ATT Read By Group Type')
    script[c + 1] = (b'', 1, bytes.fromhex('1400040011060100050000180600080001180a00ffff0f18'), 2, 'ATT rsp')
    upd_instant = None
    if args.conn_update:
        upd_instant = c + 8
        new_int = args.conn_update
        script[c + 2] = (bytes([0x00, 1]) + struct.pack('<HHHHH', 0, new_int, 0,
                                                          args.timeout,
                                                          upd_instant),
                         3, b'', 1, 'LL_CONNECTION_UPDATE_IND int=%d @%d'
                         % (new_int, upd_instant))
        pending.append((upd_instant, 'conn', new_int))
    if args.chmap_update:
        new_chans = chans[:args.chmap_update]
        inst = c + 16
        script[c + 10] = (bytes([0x01]) + chm_from_list(new_chans).to_bytes(5, 'little')
                          + struct.pack('<H', inst), 3, b'', 1,
                          'LL_CHANNEL_MAP_IND %s @%d' % (new_chans, inst))
        pending.append((inst, 'chm', chm_from_list(new_chans)))
    enc_at = None
    if args.encrypt:
        enc_at = c + 20
        script[enc_at] = (bytes([0x03]) + bytes(rnd.getrandbits(8) for _ in range(22)),
                          3, b'', 1, 'LL_ENC_REQ')
        script[enc_at + 1] = (b'', 1, bytes([0x04]) + bytes(rnd.getrandbits(8) for _ in range(12)),
                              3, 'LL_ENC_RSP')
        script[enc_at + 2] = (b'', 1, bytes([0x05]), 3, 'LL_START_ENC_REQ')
        script[enc_at + 3] = (bytes([0x06]), 3, bytes([0x06]), 3,
                              'LL_START_ENC_RSP (encrypted)')
    if args.forge_slave_update:
        # a central-only PDU from the peripheral's slot (forged): the link
        # ignores it, and so must a follower
        script[c + 5] = (b'', 1, bytes([0x00, 1]) + struct.pack('<HHHHH', 0, 6, 0,
                                                                   args.timeout, c + 11),
                         3, 'forged LL_CONNECTION_UPDATE_IND from the peripheral')
    if args.enc_ctrl6 and args.encrypt:
        # an encrypted 6-byte LL control PDU (LL_UNKNOWN_RSP, LL_REJECT_IND,
        # ... have the length of LL_TERMINATE_IND) while traffic goes on
        script[c + 26] = (b'', 1, bytes([0x07, 0x00]), 3, 'encrypted 6-byte LL control')
    if args.enc_ctrl9 and args.encrypt:
        # encrypted 9-byte LL control PDUs from the central, the length of
        # LL_PHY_UPDATE_IND (LL_POWER_CHANGE_IND has it too), the PHY
        # unchanged; random bytes, as ciphertext is
        for k in (28, 32, 36):
            script[c + k] = (bytes(rnd.getrandbits(8) for _ in range(5)), 3, b'', 1,
                             'encrypted 9-byte LL control from the central')
    if args.phy2m:
        at = c + 30
        inst = at + 6
        script[at] = (bytes([0x18, 0x02, 0x02]) + struct.pack('<H', inst), 3,
                      b'', 1, 'LL_PHY_UPDATE_IND 2M @%d' % inst)
        pending.append((inst, 'phy', '2M'))

    fade = [int(x) for x in args.fade.split(':')] if args.fade else None
    while counter < args.events:
        # apply instants
        for inst, kind, val in list(pending):
            if counter == inst:
                if kind == 'conn':
                    anchor += 600  # inside the transmit window (WinOffset 0)
                    interval = val
                    interval_us = interval * 1250
                elif kind == 'chm':
                    chm = val
                elif kind == 'phy':
                    phy = val
                pending.remove((inst, kind, val))
        if args.csa == 2:
            ch = csa2(counter, aa, chm)
        else:
            last_unmapped, ch = csa1(last_unmapped, hop, chm)
        pm_pl, llid_m, ps_pl, llid_s, note = script.get(
            counter, (b'', 1, b'', 1, ''))
        if args.terminate and counter == args.terminate:
            pm_pl, llid_m, note = bytes([0x02, 0x13]), 3, 'LL_TERMINATE_IND'
        if encrypted:
            pm_pl, ps_pl = mic(pm_pl), mic(ps_pl)
        pm, ps = m_s(pm_pl, ps_pl, llid_m, llid_s)
        if fade and fade[0] <= counter < fade[1]:
            pass  # neither side heard (the link goes on; the receiver hears nothing)
        else:
            tl.add(anchor, ch, aa, pm, crc_init, phy=phy,
                   note='M ev=%d ch=%d %s' % (counter, ch, note))
            ts = anchor + tl.duration_us(len(pm), phy) + 150
            tl.add(ts, ch, aa, ps, crc_init, phy=phy,
                   note='S ev=%d ch=%d' % (counter, ch))
        if enc_at is not None and counter == enc_at + 2:
            encrypted = True
        anchor += interval_us
        counter += 1
        if args.terminate and counter == args.terminate + 1:
            break
    return dict(aa='%08x' % aa, crc_init='%06x' % crc_init, csa=args.csa,
                hop=hop, chm=chans)


def scenario_midconn(tl, args):
    """An established connection (no CONNECT_IND): for promiscuous
    recovery. Random event counter (CSA#1's phase follows it); the channel
    map is the channels inside the HackRF window (or --chm). With
    --conn-update, a plaintext LL_CONNECTION_UPDATE_IND at 85% of the
    duration, instant 8 events later: a follower that recovered the
    counter wrongly applies it at the wrong event."""
    rnd = random.Random(args.seed)
    aa = args.aa if args.aa is not None else rnd.getrandbits(32)
    crc_init = rnd.getrandbits(24)
    chans = ([int(c) for c in args.chm.split(',')] if args.chm
             else WINDOW_DATA_CHANNELS)
    chm = chm_from_list(chans)
    counter = rnd.getrandbits(16)
    # CSA#1's phase follows the counter: lastUnmappedChannel is 0 before
    # event 0
    last_unmapped = counter * args.hop % 37
    interval_us = args.interval * 1250
    t = 1000.0
    sn = nesn = 0
    first = None
    n = int(args.duration * 1e6 / interval_us)
    upd_at = int(n * 0.85) if args.conn_update else None
    upd_instant = None
    for k in range(n):
        c = (counter + k) & 0xffff
        if upd_instant is not None and c == upd_instant:
            # WinOffset 0: the transmit window opens at the anchor the old
            # interval gives; the central sends 600 us into it (as in conn)
            t += 600
            interval_us = args.conn_update * 1250
        if args.csa == 2:
            ch = csa2(c, aa, chm)
        else:
            last_unmapped, ch = csa1(last_unmapped, args.hop, chm)
        pl = bytes([rnd.getrandbits(8) for _ in range(rnd.choice([0, 0, 0, 4, 12]))])
        llid = 2 if pl else 1
        note = ''
        if k == upd_at:
            upd_instant = (c + 8) & 0xffff
            pl = bytes([0x00, 1]) + struct.pack('<HHHHH', 0, args.conn_update, 0,
                                                  args.timeout, upd_instant)
            llid = 3
            note = ' LL_CONNECTION_UPDATE_IND int=%d @%d' % (args.conn_update, upd_instant)
        pm = data_pdu(llid, pl, nesn, sn)
        ps = data_pdu(1, b'', sn ^ 1, nesn)
        tl.add(t, ch, aa, pm, crc_init, note='M ev=%d ch=%d%s' % (c, ch, note))
        tl.add(t + tl.duration_us(len(pm), '1M') + 150, ch, aa, ps, crc_init,
               note='S ev=%d ch=%d' % (c, ch))
        sn ^= 1
        nesn ^= 1
        if first is None:
            first = (c, ch)
        t += interval_us
    return dict(aa='%08x' % aa, crc_init='%06x' % crc_init, csa=args.csa,
                hop=args.hop, interval=args.interval, chm=chans,
                first_counter=first[0])


SCENARIOS = {
    'midconn': scenario_midconn,
    'adv': scenario_adv,
    'extadv': scenario_extadv,
    'coded': scenario_coded,
    'conn': scenario_conn,
}


def selftest():
    # CRC of a packet captured by an Ubertooth (ADV_NONCONN_IND, ch 37)
    pdu = bytes.fromhex('420e795a6212b3d307ff4c0012020003')
    assert crc24(ADV_CRC_INIT, pdu) == bytes.fromhex('f1b5ee'), crc24(ADV_CRC_INIT, pdu).hex()
    # CSA#2 sample data, Core Spec v5.x Vol 6 Part C 3.1
    aa = 0x8E89BED6
    assert ((aa >> 16) ^ aa) & 0xFFFF == 0x305F
    full = (1 << 37) - 1
    assert [csa2(c, aa, full) for c in range(4)] == [25, 20, 6, 21]
    part = chm_from_list([9, 10, 21, 22, 23, 33, 34, 35, 36])
    assert [csa2(c, aa, part) for c in (6, 7, 8)] == [23, 9, 34]
    # whitening of channel 37 matches the firmware's table
    # (whitening[] starting at whitening_index[37] = 112)
    assert whiten_bits([0] * 7, 37) == [1, 0, 1, 1, 0, 0, 0]
    print('selftest ok')


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('scenario', choices=sorted(SCENARIOS) + ['selftest'])
    ap.add_argument('-o', '--out', default='blesim.iq')
    ap.add_argument('--fc', type=float, default=2431.0, help='center MHz')
    ap.add_argument('--fs', type=float, default=20e6)
    ap.add_argument('--amplitude', type=float, default=0.5)
    ap.add_argument('--count', type=int, default=20)
    ap.add_argument('--adv-interval-us', type=int, default=50000)
    ap.add_argument('--adva', default='c0:ff:ee:00:00:01')
    ap.add_argument('--inita', default='c0:ff:ee:00:00:02')
    ap.add_argument('--name', default='blesim')
    ap.add_argument('--csa', type=int, choices=(1, 2), default=2)
    ap.add_argument('--hop', type=int, default=7)
    ap.add_argument('--interval', type=int, default=24, help='1.25 ms units')
    ap.add_argument('--timeout', type=int, default=200, help='10 ms units')
    ap.add_argument('--events', type=int, default=200)
    ap.add_argument('--conn-update', type=int, default=0,
                    help='new interval for an LL_CONNECTION_UPDATE_IND (conn; midconn: at 85%% '
                         'of the duration)')
    ap.add_argument('--chmap-update', type=int, default=0,
                    help='shrink channel map to this many channels')
    ap.add_argument('--encrypt', action='store_true')
    ap.add_argument('--terminate', type=int, default=0,
                    help='LL_TERMINATE_IND at this event, then silence')
    ap.add_argument('--forge-slave-update', action='store_true',
                    help='an LL_CONNECTION_UPDATE_IND from the peripheral (ignored by the link)')
    ap.add_argument('--enc-ctrl6', action='store_true',
                    help='with --encrypt: an encrypted 6-byte LL control PDU, traffic continuing')
    ap.add_argument('--enc-ctrl9', action='store_true',
                    help='with --encrypt: encrypted 9-byte LL control PDUs from the central '
                         '(the length of LL_PHY_UPDATE_IND) at events 31, 35 and 39, the PHY unchanged')
    ap.add_argument('--fade', default=None, metavar='A:B',
                    help='conn: no packets in events A to B-1, as in a fade')
    ap.add_argument('--phy2m', action='store_true')
    ap.add_argument('--coding', type=int, choices=(2, 8), default=8)
    ap.add_argument('--aa', type=lambda s: int(s, 16), default=None)
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--duration', type=float, default=60.0, help='midconn: seconds')
    ap.add_argument('--chm', default=None,
                    help='conn, midconn: comma separated data channels (they must fit in --fs)')
    args = ap.parse_args()

    if args.scenario == 'selftest':
        selftest()
        return

    tl = Timeline(args.fc, args.fs)
    meta = SCENARIOS[args.scenario](tl, args) or {}
    nsamp = tl.render_to_file(args.out, amplitude=args.amplitude)
    truth = dict(scenario=args.scenario, fc_mhz=args.fc, fs=args.fs,
                 meta=meta, packets=tl.truth())
    with open(os.path.splitext(args.out)[0] + '.json', 'w') as f:
        json.dump(truth, f, indent=1)
    print('%s: %d packets, %.3f s, %d bytes' % (
        args.out, len(tl.events), nsamp / args.fs, 2 * nsamp))
    print('transmit with: hackrf_transfer -t %s -f %d -s %d -x 0 -a 0'
          % (args.out, int(args.fc * 1e6), int(args.fs)))


if __name__ == '__main__':
    main()
