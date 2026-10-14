"""
Minimal pcapng reader/writer for Bluetooth LE link layer captures.

Reads LINKTYPE_BLUETOOTH_LE_LL (251) and LINKTYPE_BLUETOOTH_LE_LL_WITH_PHDR
(256) as written by libbtbb / ubertooth-btle, and writes
LINKTYPE_BLUETOOTH_LE_LL_WITH_PHDR with the PHY and CRC flags set so
Wireshark decodes LE 2M packets.

This file is part of Project Ubertooth (GPL-2.0-or-later).
"""

import struct

LINKTYPE_BLUETOOTH_LE_LL = 251
LINKTYPE_BLUETOOTH_LE_LL_WITH_PHDR = 256

# LE LL with PHDR flags
F_DEWHITENED = 0x0001
F_SIGNAL_VALID = 0x0002
F_NOISE_VALID = 0x0004
F_REF_AA_VALID = 0x0010
F_PDU_DATA_M2S = 0x0100   # PDU type field (bits 7-9) = 2: data, master to slave
F_PDU_DATA_S2M = 0x0180   # PDU type field (bits 7-9) = 3: data, slave to master
F_CRC_CHECKED = 0x0400
F_CRC_VALID = 0x0800
F_PHY_2M = 0x4000         # PHY field (bits 14-15) = 1

ADV_AA = 0x8E89BED6


def read(path):
    """Yield dicts: ts_ns, rf_channel (0-39 RF index), aa, pdu (bytes incl. CRC), flags."""
    with open(path, 'rb') as f:
        data = f.read()
    pos = 0
    endian = '<'
    linktypes = []
    tsres = []
    while pos + 12 <= len(data):
        btype, blen = struct.unpack_from(endian + 'II', data, pos)
        if btype == 0x0A0D0D0A:
            bom = struct.unpack_from('<I', data, pos + 8)[0]
            endian = '<' if bom == 0x1A2B3C4D else '>'
            blen = struct.unpack_from(endian + 'I', data, pos + 4)[0]
            linktypes, tsres = [], []
        elif btype == 1:  # IDB
            lt = struct.unpack_from(endian + 'H', data, pos + 8)[0]
            res = 1e-6
            opos = pos + 16
            while opos + 4 <= pos + blen - 4:
                code, olen = struct.unpack_from(endian + 'HH', data, opos)
                if code == 0:
                    break
                if code == 9 and olen >= 1:
                    v = data[opos + 4]
                    res = 2 ** -(v & 0x7f) if v & 0x80 else 10 ** -v
                opos += 4 + ((olen + 3) & ~3)
            linktypes.append(lt)
            tsres.append(res)
        elif btype == 6:  # EPB
            iface, hi, lo, caplen, _ = struct.unpack_from(endian + 'IIIII', data, pos + 8)
            pkt = data[pos + 28:pos + 28 + caplen]
            ts_ns = int(((hi << 32) | lo) * tsres[iface] * 1e9)
            lt = linktypes[iface]
            if lt == LINKTYPE_BLUETOOTH_LE_LL_WITH_PHDR and len(pkt) >= 14:
                rf, sig, noise, offenses, ref_aa, flags = struct.unpack_from('<BbbBIH', pkt, 0)
                body = pkt[10:]
                yield dict(ts_ns=ts_ns, rf_channel=rf, signal=sig, aa=struct.unpack_from('<I', body)[0],
                           pdu=body[4:], flags=flags)
            elif lt == LINKTYPE_BLUETOOTH_LE_LL and len(pkt) >= 4:
                yield dict(ts_ns=ts_ns, rf_channel=None, signal=None,
                           aa=struct.unpack_from('<I', pkt)[0], pdu=pkt[4:], flags=0)
        if blen < 12:
            break
        pos += blen


def rf_index_to_chan(rf):
    """RF channel index (0-39, 2402 + 2*rf MHz) to BLE channel index."""
    mhz = 2402 + 2 * rf
    if mhz == 2402:
        return 37
    if mhz == 2426:
        return 38
    if mhz == 2480:
        return 39
    return (mhz - 2404) // 2 if mhz < 2426 else (mhz - 2428) // 2 + 11


def chan_to_rf_index(ch):
    from blesim import chan_idx_to_mhz
    return (chan_idx_to_mhz(ch) - 2402) // 2


class Writer:
    def __init__(self, path):
        self.f = open(path, 'wb')
        shb_body = struct.pack('<IHHq', 0x1A2B3C4D, 1, 0, -1)
        self._block(0x0A0D0D0A, shb_body)
        # IDB: linktype, reserved, snaplen, option if_tsresol = 9 (ns)
        opts = struct.pack('<HHB3x', 9, 1, 9) + struct.pack('<HH', 0, 0)
        self._block(1, struct.pack('<HHI', LINKTYPE_BLUETOOTH_LE_LL_WITH_PHDR, 0, 65535) + opts)

    def _block(self, btype, body):
        body += b'\0' * ((4 - len(body) % 4) % 4)
        blen = 12 + len(body)
        self.f.write(struct.pack('<II', btype, blen) + body + struct.pack('<I', blen))

    def packet(self, ts_ns, chan, aa, pdu_with_crc, phy='1M', crc_ok=None, signal=None,
               ref_aa=None, slave=None):
        """slave: None for advertising, False/True for data PDUs."""
        flags = F_DEWHITENED
        if ref_aa is not None:
            flags |= F_REF_AA_VALID
        if slave is not None:
            flags |= F_PDU_DATA_S2M if slave else F_PDU_DATA_M2S
        if crc_ok is not None:
            flags |= F_CRC_CHECKED | (F_CRC_VALID if crc_ok else 0)
        if signal is not None:
            flags |= F_SIGNAL_VALID
        if phy == '2M':
            flags |= F_PHY_2M
        phdr = struct.pack('<BbbBIH', chan_to_rf_index(chan),
                           int(max(-128, min(127, signal or 0))), 0, 0,
                           ref_aa or 0, flags)
        pkt = phdr + struct.pack('<I', aa) + pdu_with_crc
        hdr = struct.pack('<IIIII', 0, ts_ns >> 32, ts_ns & 0xffffffff, len(pkt), len(pkt))
        self._block(6, hdr + pkt)

    def close(self):
        self.f.close()
