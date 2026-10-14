#!/usr/bin/env python3
"""
blerx - wideband software BLE receiver for HackRF IQ captures.

Decodes LE 1M and LE 2M packets in an int8 IQ capture (hackrf_transfer -r)
covering several BLE channels at once:

1. find bursts in the wideband power envelope (BLE is bursty, so most of
   the capture is skipped)
2. find which BLE channel(s) each burst occupies from its spectrum
3. mix the burst to baseband, filter, FM-demodulate and decode at 1M and
   2M; keep the decode whose CRC checks (or the longest plausible one)

Packets on advertising channels are matched against the advertising
access address. Data channel packets are matched against access
addresses given with -a (e.g. from an Ubertooth that saw the CONNECT_IND)
and, with --any-aa, against any access address following a valid
preamble (CRC unknown). CRCs are checked when the CRCInit is known.

Output is one JSON object per packet (stdout), sorted by time. Times are
microseconds from the start of the capture.

This is the HackRF half of the Ubertooth + HackRF pairing: the Ubertooth
follows the 1M control plane, the HackRF records everything in its
window, including LE 2M traffic the CC2400 cannot demodulate.

This file is part of Project Ubertooth (GPL-2.0-or-later).
"""

import argparse
import json
import sys

import numpy as np
from scipy import signal

import blesim

SPS = 4  # samples per symbol after channelization
CUTOFF = 0.7  # channel filter cutoff, in units of the symbol rate


def load_iq(path, count=None, offset=0):
    raw = np.fromfile(path, dtype=np.int8, count=-1 if count is None else 2 * count,
                      offset=2 * offset)
    return (raw[0::2].astype(np.float32) + 1j * raw[1::2].astype(np.float32)) / 128.0


def find_bursts(iq, fs, fc, chans, thresh_db, min_us, gap_us=4.0):
    """Per-channel bursts: (start, stop, [channels]) from a short-time FFT.

    Narrowband detection: each channel is compared with its own noise
    floor, so bursts far below the wideband noise power are found.
    """
    nfft = 1 << int(round(np.log2(fs / 1e6)))      # ~1 MHz bins
    nfr = len(iq) // nfft
    frames = iq[:nfr * nfft].reshape(nfr, nfft)
    spec = np.abs(np.fft.fftshift(np.fft.fft(frames, axis=1), axes=1)) ** 2
    bin_hz = fs / nfft
    freqs = (np.arange(nfft) - nfft // 2) * bin_hz + fc
    frame_us = nfft / fs * 1e6
    smooth = max(1, int(round(8.0 / frame_us)))
    k = np.ones(smooth, dtype=np.float32) / smooth
    per_chan = {}
    for ch in chans:
        f = blesim.chan_idx_to_mhz(ch) * 1e6
        sel = np.abs(freqs - f) <= bin_hz * 0.75
        pw = np.convolve(spec[:, sel].sum(axis=1), k, mode='same')
        floor = float(np.median(pw)) + 1e-12
        on = pw > floor * 10 ** (thresh_db / 10)
        per_chan[ch] = (on, pw)
    # union of all channels' activity, merged across short gaps
    any_on = np.zeros(nfr, dtype=bool)
    for on, _ in per_chan.values():
        any_on |= on
    edges = np.flatnonzero(np.diff(any_on.astype(np.int8)))
    starts = [e + 1 for e in edges if any_on[e + 1]]
    stops = [e + 1 for e in edges if not any_on[e + 1]]
    if any_on[0]:
        starts.insert(0, 0)
    if any_on[-1]:
        stops.append(nfr)
    gap = max(1, int(gap_us / frame_us))
    merged = []
    for a, b in zip(starts, stops):
        if merged and a - merged[-1][1] < gap:
            merged[-1][1] = b
        else:
            merged.append([a, b])
    out = []
    min_fr = int(min_us / frame_us)
    for a, b in merged:
        if b - a < min_fr:
            continue
        active = [ch for ch, (on, _) in per_chan.items() if on[a:b].sum() >= min_fr]
        if active:
            out.append((a * nfft, b * nfft, active))
    return out


def demod(seg, fs, off_hz, rate):
    """Mix to baseband, low-pass, resample to SPS samples/symbol, FM demod."""
    n = np.arange(len(seg))
    x = seg * np.exp(-2j * np.pi * off_hz * n / fs).astype(np.complex64)
    out_fs = rate * SPS
    taps = signal.firwin(65, CUTOFF * rate / (fs / 2))
    x = signal.lfilter(taps, 1.0, x)
    delay = (len(taps) - 1) / 2
    ratio = out_fs / fs
    from fractions import Fraction
    fr = Fraction(ratio).limit_denominator(64)
    y = signal.resample_poly(x, fr.numerator, fr.denominator)
    freq = np.angle(y[1:] * np.conj(y[:-1]))
    # integrate over a symbol: a better decision statistic than one sample
    freq = np.convolve(freq, np.ones(SPS) / SPS, mode='same')
    # carrier offset from where the signal is present (the FM output of
    # the noise around the packet is meaningless): midpoint of the two
    # frequency deviation levels, robust to unbalanced 0s and 1s
    mag = np.abs(y[1:])
    present = mag > 0.5 * np.percentile(mag, 95)
    if present.sum() > 4 * SPS:
        lo, hi = np.percentile(freq[present], [10, 90])
        freq = freq - (lo + hi) / 2
    return freq, out_fs, delay * ratio


def bits_to_bytes(bits):
    n = len(bits) // 8
    if n == 0:
        return b''
    b = np.asarray(bits[:n * 8], dtype=np.uint8).reshape(n, 8)
    return bytes((b << np.arange(8, dtype=np.uint8)).sum(axis=1).astype(np.uint8))


def aa_search(bits, patterns, max_err=1):
    win = np.lib.stride_tricks.sliding_window_view(bits, 32)
    for aa, pattern in patterns:
        err = np.count_nonzero(win != pattern, axis=1)
        hits = np.flatnonzero(err <= max_err)
        if len(hits):
            return int(hits[0]), aa
    return None


def preamble_search(bits, nbits):
    """Position after a run of nbits alternating bits (unknown AA)."""
    alt = (bits[1:] != bits[:-1]).astype(np.int8)
    run = np.convolve(alt, np.ones(nbits - 1, dtype=np.int8), mode='valid')
    hits = np.flatnonzero(run >= nbits - 1)
    if not len(hits):
        return None
    return int(hits[0]) + nbits


def decode(freq, chan, phy, patterns, crc_inits, any_aa):
    best = None
    nb_pre = 8 if phy == '1M' else 16
    for phase in range(SPS):
        s = freq[phase::SPS]
        if len(s) < 48:
            continue
        bits = (s > 0).astype(np.uint8)
        m = aa_search(bits, patterns)
        known = m is not None
        if not known:
            if not any_aa or chan >= 37:
                continue
            pos = preamble_search(bits, nb_pre)
            if pos is None or pos + 32 + 16 > len(bits):
                continue
            aa = int(sum(int(b) << i for i, b in enumerate(bits[pos:pos + 32])))
        else:
            pos, aa = m
        body = bits[pos + 32:]
        if len(body) < 16:
            continue
        hdr = bits_to_bytes(blesim.whiten_bits(list(body[:16]), chan))
        total = (2 + hdr[1] + 3) * 8
        if len(body) < total:
            continue
        pkt = bits_to_bytes(blesim.whiten_bits(list(body[:total]), chan))
        pdu, crc = pkt[:-3], pkt[-3:]
        ci = crc_inits.get(aa)
        ok = None if ci is None else blesim.crc24(ci, pdu) == crc
        rank = (ok is True, known, len(pdu))
        cand = (rank, phase, pos, aa, pdu, crc, ok)
        if best is None or cand[0] > best[0]:
            best = cand
        if ok:
            break
    return best


def decode_capture(iq_path, fc_mhz, fs=20e6, aa_specs=(), any_aa=False,
                   phys=('1M', '2M'), thresh_db=5.0, channels=None, chunk_s=1.0):
    """Decode a HackRF capture; returns a list of packet dicts sorted by time.

    aa_specs: data channel access addresses as ints or (aa, crc_init) pairs.
    """
    patterns = [(blesim.ADV_AA, np.array(blesim.u32_bits(blesim.ADV_AA), np.uint8))]
    crc_inits = {blesim.ADV_AA: blesim.ADV_CRC_INIT}
    for spec in aa_specs:
        aa, ci = spec if isinstance(spec, tuple) else (spec, None)
        patterns.append((aa, np.array(blesim.u32_bits(aa), np.uint8)))
        if ci is not None:
            crc_inits[aa] = ci

    fc = fc_mhz * 1e6
    chans = [c for c in (range(40) if channels is None else channels)
             if abs(blesim.chan_idx_to_mhz(c) * 1e6 - fc) + 1.2e6 < fs / 2]

    total = len(np.memmap(iq_path, dtype=np.int8, mode='r')) // 2
    chunk = int(chunk_s * fs)
    overlap = int(3e-3 * fs)  # longest packet is ~2.1 ms
    out = []
    seen = set()
    for start in range(0, total, chunk):
        iq = load_iq(iq_path, min(chunk + overlap, total - start), start)
        bursts = find_bursts(iq, fs, fc, chans, thresh_db, 30)
        pad = int(12e-6 * fs)
        for a, b, active in bursts:
            a0, b0 = max(0, a - pad), min(len(iq), b + pad)
            if a0 >= chunk and start + chunk < total:
                continue  # belongs to the next chunk
            seg = iq[a0:b0]
            for ch in active:
                off = blesim.chan_idx_to_mhz(ch) * 1e6 - fc
                pats = [p for p in patterns if ch < 37 or p[0] == blesim.ADV_AA]
                results = []
                for phy in phys:
                    rate = 1e6 if phy == '1M' else 2e6
                    freq, out_fs, delay = demod(seg, fs, off, rate)
                    res = decode(freq, ch, phy, pats, crc_inits, any_aa)
                    if res:
                        results.append((res[0], phy, out_fs, delay, res))
                if not results:
                    continue
                results.sort(key=lambda r: r[0], reverse=True)
                _, phy, out_fs, delay, (_, phase, pos, aa, pdu, crc, ok) = results[0]
                nb_pre = 8 if phy == '1M' else 16
                sym = 1.0 if phy == '1M' else 0.5
                t_us = ((start + a0) / fs * 1e6 + (phase + pos * SPS - delay) / out_fs * 1e6
                        - nb_pre * sym)
                key = (round(t_us / 20), ch)
                if key in seen:
                    continue
                seen.add(key)
                pw = 10 * np.log10(np.mean(np.abs(seg) ** 2) + 1e-12)
                out.append(dict(t_us=round(t_us, 2), chan=ch, phy=phy,
                                aa='%08x' % aa, pdu=pdu.hex(), crc=crc.hex(),
                                crc_ok=ok, power_db=round(float(pw), 1)))
    out.sort(key=lambda r: r['t_us'])
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('iq')
    ap.add_argument('--fc', type=float, required=True, help='center MHz')
    ap.add_argument('--fs', type=float, default=20e6)
    ap.add_argument('-a', '--aa', action='append', default=[],
                    help='data channel access address[:crcinit] (hex)')
    ap.add_argument('--any-aa', action='store_true',
                    help='also decode data packets with unknown access addresses')
    ap.add_argument('--phy', choices=('1M', '2M', 'both'), default='both')
    ap.add_argument('--thresh-db', type=float, default=5.0)
    ap.add_argument('--channels', default=None,
                    help='comma separated channel indices (default: all in window)')
    ap.add_argument('--chunk-s', type=float, default=1.0,
                    help='process the capture in chunks of this many seconds')
    args = ap.parse_args()

    specs = []
    for s in args.aa:
        aa_s, _, ci_s = s.partition(':')
        specs.append((int(aa_s, 16), int(ci_s, 16) if ci_s else None))
    for r in decode_capture(args.iq, args.fc, args.fs, specs, args.any_aa,
                            ['1M', '2M'] if args.phy == 'both' else [args.phy],
                            args.thresh_db,
                            None if args.channels is None else
                            [int(c) for c in args.channels.split(',')],
                            args.chunk_s):
        print(json.dumps(r))


if __name__ == '__main__':
    main()
