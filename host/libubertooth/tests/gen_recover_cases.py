#!/usr/bin/env python3
"""Generate observation files for test_le_recover from blesim's CSA code.

Each case simulates a connection, parks a monitor on channel c1 then c2
(as ubertooth-btle's recovery does), and records what an Ubertooth would
see: device-time anchors with jitter and clock drift, lost packets, and
dewhitened PDUs with CRC.
"""
import os, random, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../python/blesim'))
import blesim

def case(name, csa, interval, chans, seed, loss=0.1, ppm=40, t1=8.0, t2=6.0, master_loss=0.15):
    rnd = random.Random(seed)
    aa = rnd.getrandbits(32) | 0x01010101
    crc_init = rnd.getrandbits(24)
    hop = rnd.randint(5, 16)
    chm = blesim.chm_from_list(chans)
    used = blesim.used_channels(chm)
    c1 = rnd.choice(used)
    c2 = rnd.choice([c for c in used if c != c1])
    counter0 = rnd.getrandbits(16)
    # lastUnmappedChannel is 0 before event 0, so before event counter0
    # it is counter0 * hop mod 37
    last_unmapped = counter0 * hop % 37
    t = rnd.randint(0, 10**6)            # device ticks (100 ns)
    iv = interval * 12500
    lines = []
    ev = 0
    end1, end2 = t + t1 * 1e7, t + (t1 + t2) * 1e7
    truth = None
    while t < end2:
        cnt = (counter0 + ev) & 0xffff
        if csa == 2:
            ch = blesim.csa2(cnt, aa, chm)
        else:
            last_unmapped, ch = blesim.csa1(last_unmapped, hop, chm)
        mon = c1 if t < end1 else c2
        if ch == mon and rnd.random() > loss:
            jitter = rnd.randint(-15, 15)
            drop_master = rnd.random() < master_loss
            # the master's empty PDU, then the slave's reply T_IFS after
            # its end: (8 + 2) bytes at 1 us per bit, plus 150 us
            reply_delay = ((8 + 2) * 8 + 150) * 10
            for k, payload in enumerate([b'', bytes([rnd.getrandbits(8) for _ in range(rnd.randint(0, 8))])]):
                if k == 0 and drop_master:
                    continue  # missed master packet: only the slave's reply
                pdu = blesim.data_pdu(1 if not payload else 2, payload, k, ev & 1)
                pkt = pdu + blesim.crc24(crc_init, pdu)
                ts = int(t + jitter + k * reply_delay)
                lines.append('%d %d %s' % (ts & 0xffffffff, ch, pkt.hex()))
            # the solver anchors on the last event with an exact anchor
            # (master packet seen)
            if not drop_master:
                truth = (cnt, last_unmapped)
        ev += 1
        t += iv * (1 + ppm * 1e-6)
    with open(name + '.txt', 'w') as f:
        f.write('# aa crc csa interval hop counter unmapped (of the last event whose master packet was seen) chm\n')
        f.write('%08x %06x %d %d %d %d %d %x\n' % (aa, crc_init, csa, interval, hop, truth[0], truth[1], chm))
        f.write('\n'.join(lines) + '\n')

full = list(range(37))
case('csa2_full_7ms5', 2, 6, full, 1)
case('csa2_full_30ms', 2, 24, full, 2, t1=20, t2=10)
case('csa2_partial', 2, 12, [0, 1, 2, 5, 8, 9, 13, 17, 20, 21, 22, 25, 30, 31, 33, 36], 3, t1=12, t2=8)
case('csa2_lossy', 2, 8, full, 4, loss=0.4, t1=15, t2=10)
case('csa1_full', 1, 8, full, 5, t1=10, t2=6)
case('csa1_partial', 1, 16, [i for i in range(37) if i % 3], 6, t1=20, t2=12)
