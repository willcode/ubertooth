#!/usr/bin/env python3
"""Write a synthetic BR channelizer word stream holding LE 1M frames, and the
packets the HackRF LE search must find in it (test_hackrf_le).

Words: 16-bit, little endian, one per symbol; bit k is the decision for
center + k MHz (k >= 9 meaning k - 16), random where no frame is sent.
Expected: one line per packet, "freq aa_word slave aux pdu_hex" (aa_word is
the word of the first access address bit, pdu_hex the dewhitened PDU).
test_hackrf_le drops bank DROP_BANK, as the backend drops banks at a retune."""
import os
import random
import struct
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '../../python/blesim'))
import blesim

CENTER = 2409
NWORDS = 400 * 90
DROP_BANK = 80
CONN_AA, CONN_CRC = 0x50654a1b, 0x2c1d3e
T_IFS = 150

rng = random.Random(7)
bins = [[rng.getrandbits(1) for _ in range(NWORDS)] for _ in range(16)]
expected = []


def bin_of(freq):
    off = freq - CENTER
    return off if off >= 0 else off + 16


def chan_idx(freq):
    return {2402: 37, 2426: 38, 2480: 39}.get(
        freq, (freq - 2404) // 2 if freq < 2426 else (freq - 2428) // 2 + 11)


def send(freq, start, aa, pdu, crc_init, aa_flips=(), body_flips=(),
         found=True, slave=0):
    """Put a frame whose preamble starts at word `start`; returns its end."""
    bits = blesim.frame_bits_uncoded(aa, pdu, crc_init, chan_idx(freq))
    for i in aa_flips:              # access address bit errors
        bits[8 + i] ^= 1
    for i in body_flips:            # bit errors after the access address
        bits[40 + i] ^= 1
    col = bins[bin_of(freq)]
    col[start:start + len(bits)] = bits
    if found:
        expected.append((freq, start + 8, slave, int(aa == blesim.ADV_AA and
                                                     chan_idx(freq) < 37), pdu))
    return start + len(bits)


adv = blesim.adv_ind('c0:11:22:33:44:55', blesim.ad_flags() + blesim.ad_name('hle'))


def send_adv(freq, start, pdu, **kw):
    return send(freq, start, blesim.ADV_AA, pdu, blesim.ADV_CRC_INIT, **kw)



send_adv(2402, 1000, adv)
send_adv(2402, 1590, adv)                  # across a bank boundary
send_adv(2402, 2500, adv, aa_flips=(3,))
send_adv(2402, 3300, adv, aa_flips=(0, 31))
send_adv(2402, 4100, adv, aa_flips=(1, 9, 20), found=False)
send_adv(2402, 4900, adv, body_flips=(17,))        # corrected
send_adv(2402, 5300, adv, body_flips=(17, 60), found=False)
send_adv(2402, 6100, adv, body_flips=(12,), found=False)   # the length byte
send_adv(2412, 6900, adv, body_flips=(16 + 8 * 14 + 5,))   # a CRC bit: corrected
send_adv(2412, 5700, adv)                  # secondary channel: aux
# a connection's packet before its CONNECT_IND: not searched for yet
send(2404, 6500, CONN_AA, blesim.data_pdu(1, b'', 0, 0), CONN_CRC, found=False)

conn = blesim.connect_ind('c0:aa:bb:cc:dd:ee', 'c0:11:22:33:44:55', CONN_AA,
                          CONN_CRC, 2, 1, 6, 0, 100, 0x1fffffffff, 7)
send_adv(2402, 7400, conn)

# events: a master packet, the slave's reply T_IFS later
end = send(2404, 9000, CONN_AA, blesim.data_pdu(2, b'\x03\x00\x04\x00\x0a\x01\x00', 0, 0),
           CONN_CRC)
send(2404, end + T_IFS, CONN_AA, blesim.data_pdu(1, b'', 1, 0), CONN_CRC, slave=1)
long_payload = bytes(range(100))                         # LE_PACKET_CONT territory
end = send(2410, 14000, CONN_AA, blesim.data_pdu(2, long_payload, 1, 1), CONN_CRC,
           aa_flips=(12,))
send(2410, end + T_IFS, CONN_AA, blesim.data_pdu(1, b'', 0, 1), CONN_CRC, slave=1)
send(2416, 19000, CONN_AA, blesim.data_pdu(1, b'', 0, 0), CONN_CRC, body_flips=(5,))
# not T_IFS after the previous packet: a new event, from the master
send(2416, 21000, CONN_AA, blesim.data_pdu(1, b'', 0, 0), CONN_CRC)
send(2416, 22000, CONN_AA, blesim.data_pdu(1, b'', 1, 0), CONN_CRC)
# the connection's access address on the advertising channel: not searched
send(2402, 24000, CONN_AA, blesim.data_pdu(1, b'', 0, 0), CONN_CRC, found=False)
# the largest PDU: 255 bytes of payload
send(2408, 25000, CONN_AA, blesim.data_pdu(2, bytes(range(255)), 0, 1), CONN_CRC)
# a master packet with two bit errors in its first header byte fails its CRC;
# its reply is still the slave's
end = send(2412, 28000, CONN_AA, blesim.data_pdu(1, b'', 1, 1), CONN_CRC, body_flips=(3, 5),
           found=False)
send(2412, end + T_IFS, CONN_AA, blesim.data_pdu(1, b'', 1, 0), CONN_CRC, slave=1)
# a packet across the dropped bank is lost; the search restarts after the gap
send(2414, 400 * DROP_BANK - 50, CONN_AA, blesim.data_pdu(1, b'', 0, 0), CONN_CRC, found=False)
send(2414, 400 * DROP_BANK + 500, CONN_AA, blesim.data_pdu(1, b'', 0, 0), CONN_CRC)

words = [0] * NWORDS
for k in range(16):
    if k == 8:
        continue
    for i, b in enumerate(bins[k]):
        words[i] |= b << k
for i in range(0, NWORDS, 400):
    words[i] |= 0x100

with open(sys.argv[1], 'wb') as f:
    f.write(struct.pack('<%dH' % NWORDS, *words))
with open(sys.argv[2], 'w') as f:
    for freq, word, slave, aux, pdu in expected:
        f.write('%d %d %d %d %s\n' % (freq, word, slave, aux, pdu.hex()))
