#!/usr/bin/env python3
"""
blecoded - experimental LE Coded PHY (long range) receiver for the Ubertooth.

The CC2400 cannot synchronize on LE Coded packets, but LE Coded uses the
same 1 Msym/s GFSK as LE 1M, so its raw demodulated symbols can be
decoded on the host:

    ubertooth-util -c 2426                       # advertising channel 38
    ubertooth-dump -l -b > coded.bin             # raw LE symbols
    blecoded.py coded.bin --channel 38

1. correlate the symbol stream with the 80 us preamble followed by the
   access address, convolutionally coded and pattern mapped (S=8)
2. read the coding indicator (S=8 or S=2) and TERM1
3. pattern-demap FEC block 2, Viterbi-decode (K=4, rate 1/2, hard
   decisions), dewhiten, check the CRC

Input is the text written by ubertooth-dump -b ('0'/'1' characters, a
newline every 400 symbols) or one symbol per byte (0x00/0x01); the format
is detected. Output is one JSON object per packet.

This file is part of Project Ubertooth (GPL-2.0-or-later).
"""

import argparse
import json
import sys

import numpy as np

import blesim

G0, G1 = 0b1111, 0b1101   # K = 4: g0 = 1 + D + D^2 + D^3, g1 = 1 + D^2 + D^3


def _parity(v):
    return bin(v).count('1') & 1


# trellis: state = last 3 input bits (most recent in bit 0)
_NEXT = np.zeros((8, 2), dtype=np.int64)
_OUT = np.zeros((8, 2, 2), dtype=np.int64)
for _s in range(8):
    for _b in range(2):
        reg = (_s << 1) | _b                # b, s0, s1, s2 -> D^0..D^3
        reg_rev = int('{:04b}'.format(reg)[::-1], 2)
        # g applied to [b, s0, s1, s2] with b as the D^0 term
        taps = [(reg >> 0) & 1, (reg >> 1) & 1, (reg >> 2) & 1, (reg >> 3) & 1]
        a0 = taps[0] ^ taps[1] ^ taps[2] ^ taps[3]
        a1 = taps[0] ^ taps[2] ^ taps[3]
        _OUT[_s, _b] = (a0, a1)
        _NEXT[_s, _b] = reg & 0b111


def viterbi(soft_pairs, nbits):
    """Decode nbits from (a0, a1) pairs of soft values in [0, 1]."""
    inf = 1e9
    metric = np.full(8, inf)
    metric[0] = 0.0
    prev = np.zeros((nbits, 8), dtype=np.int8)
    inbit = np.zeros((nbits, 8), dtype=np.int8)
    for i in range(nbits):
        r0, r1 = soft_pairs[i]
        new = np.full(8, inf)
        for s in range(8):
            if metric[s] >= inf:
                continue
            for b in range(2):
                o0, o1 = _OUT[s, b]
                cost = metric[s] + abs(o0 - r0) + abs(o1 - r1)
                ns = _NEXT[s, b]
                if cost < new[ns]:
                    new[ns] = cost
                    prev[i, ns] = s
                    inbit[i, ns] = b
        metric = new
    s = int(np.argmin(metric))
    bits = []
    for i in range(nbits - 1, -1, -1):
        bits.append(int(inbit[i, s]))
        s = int(prev[i, s])
    return bits[::-1], float(np.min(metric))


def demap(symbols, ncoded, s):
    """Pattern demapper: S=8 maps a coded bit to 4 symbols (0 -> 0011,
    1 -> 1100); S=2 uses one symbol per coded bit. Returns soft values."""
    if s == 2:
        return [float(x) for x in symbols[:ncoded]]
    out = []
    for i in range(ncoded):
        q = [int(v) for v in symbols[4 * i:4 * i + 4]]
        if len(q) < 4:
            break
        # correlation with 1100 vs 0011
        v = (q[0] + q[1] - q[2] - q[3] + 2) / 4.0
        out.append(v)
    return out


def reference(aa):
    """Symbols of the preamble and the coded access address."""
    bits = blesim.frame_bits_coded(aa, b'\0\0', 0, 37, 8)
    return np.array(bits[:80 + 256], dtype=np.int8)


def decode_at(sym, pos, channel, aa, crc_init):
    """Decode a packet whose preamble starts at sym[pos]."""
    p = pos + 80
    # FEC block 1: AA (32) + CI (2) + TERM1 (3) bits, coded, S=8
    soft = demap(sym[p:], 2 * 37, 8)
    if len(soft) < 2 * 37:
        return None
    pairs = list(zip(soft[0::2], soft[1::2]))
    bits1, _ = viterbi(pairs, 37)
    got_aa = sum(b << i for i, b in enumerate(bits1[:32]))
    ci = bits1[32] | (bits1[33] << 1)
    s = 8 if ci == 0 else 2 if ci == 1 else None
    if s is None:
        return None
    p += 2 * 37 * 4
    # FEC block 2: decode the header first to learn the length
    def block2(nbits):
        ncoded = 2 * (nbits + 3)
        soft2 = demap(sym[p:], ncoded, s)
        if len(soft2) < ncoded:
            return None, None
        pairs2 = list(zip(soft2[0::2], soft2[1::2]))
        return viterbi(pairs2, nbits + 3)
    bits, _ = block2(16)
    if bits is None:
        return None
    hdr = blesim.whiten_bits(bits[:16], channel)
    length = sum(b << i for i, b in enumerate(hdr[8:16]))
    total = (2 + length + 3) * 8
    bits, cost = block2(total)
    if bits is None:
        return None
    data = blesim.whiten_bits(bits[:total], channel)
    pkt = bytes(sum(data[8 * i + j] << j for j in range(8)) for i in range(total // 8))
    pdu, crc = pkt[:-3], pkt[-3:]
    return dict(aa='%08x' % got_aa, coding='S=%d' % s, pdu=pdu.hex(), crc=crc.hex(),
                crc_ok=blesim.crc24(crc_init, pdu) == crc, path_cost=round(cost, 1))


def find_packets(sym, aa=blesim.ADV_AA, max_err=0.18):
    ref = reference(aa) * 2 - 1
    x = sym.astype(np.float32) * 2 - 1
    n = len(ref)
    if len(x) < n:
        return []
    corr = np.correlate(x, ref.astype(np.float32), mode='valid')
    thresh = n * (1 - 2 * max_err)
    hits = np.flatnonzero(corr >= thresh)
    out, last = [], -10**9
    for h in hits:
        if h - last < n:
            if corr[h] > corr[out[-1]]:
                out[-1] = h
            continue
        out.append(h)
        last = h
    return [int(h) for h in out]


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('symbols', help='ubertooth-dump -l -b output (or - for stdin)')
    ap.add_argument('--channel', type=int, default=38, help='BLE channel index')
    ap.add_argument('--max-err', type=float, default=0.18,
                    help='max fraction of symbol errors in the preamble+AA')
    args = ap.parse_args()

    raw = sys.stdin.buffer.read() if args.symbols == '-' else open(args.symbols, 'rb').read()
    buf = np.frombuffer(raw, dtype=np.uint8)
    if np.any(buf >= ord('0')):
        # text: keep only the symbol characters
        buf = buf[(buf == ord('0')) | (buf == ord('1'))] - ord('0')
    sym = buf & 1
    for pos in find_packets(sym, max_err=args.max_err):
        r = decode_at(sym, pos, args.channel, blesim.ADV_AA, blesim.ADV_CRC_INIT)
        if r:
            r['symbol'] = pos
            print(json.dumps(r))


if __name__ == '__main__':
    main()
