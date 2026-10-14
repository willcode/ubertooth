#!/usr/bin/env python3
"""Write a synthetic ubertooth-btle raw dump (4-byte BE systime + 64-byte
usb_pkt_rx records) exercising LE_PACKET_CONT reassembly and LE_STATE."""
import os, struct, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '../../python/blesim'))
import blesim

LE_PACKET, LE_STATE, LE_PACKET_CONT = 1, 7, 8
AA, CRC_INIT = 0x2265b1f5, 0x91b758

def rec(pkt_type, flags, ts, chan_mhz, data, counter=0, rssi=-40):
    hdr = struct.pack('<BBBBIbbbB2s', pkt_type, 0, (chan_mhz - 2402) & 0xff, flags, ts,
                      rssi, rssi, rssi, 4 if pkt_type == LE_PACKET else 0,
                      struct.pack('<H', counter))
    data = data.ljust(50, b'\0')
    return struct.pack('>I', 1700000000) + hdr + data

def le_packet(ts, chan_mhz, aa, pdu, flags, counter):
    stream = struct.pack('<I', aa) + pdu + blesim.crc24(CRC_INIT if aa == AA else blesim.ADV_CRC_INIT, pdu)
    out = []
    total = len(stream)
    if total > 50:
        flags |= 0x08
    out.append(rec(LE_PACKET, flags, ts, chan_mhz, stream[:50], counter))
    for i, off in enumerate(range(50, total, 50), 1):
        out.append(rec(LE_PACKET_CONT, i, ts, chan_mhz, stream[off:off + 50]))
    return out

recs = []
# LE_STATE CONN_START
conn = struct.pack('<IIHHH5sBBBBBH', AA, CRC_INIT, 24, 0, 200, b'\xff\xff\xff\xff\x1f', 7, 2, 0, 1, 5, 1)
recs.append(rec(LE_STATE, 0, 1000, 2402, bytes([0]) + conn))
# a long L2CAP PDU (100 bytes) from the master, CRC ok, counter 5
payload = bytes(range(100))
recs += le_packet(20000, 2404, AA, blesim.data_pdu(2, payload, 0, 1), 0x01 | 0x10 | 0x40 | 0x80, 5)
# the slave's empty reply, encrypted connection flag set
recs += le_packet(21500, 2404, AA, blesim.data_pdu(1, b'', 1, 1), 0x01 | 0x02 | 0x04 | 0x10 | 0x40 | 0x80, 5)
# LE_STATE PHY update, then CONN_LOST(PHY)
recs.append(rec(LE_STATE, 0, 30000, 2402, bytes([5, 2, 2, 0x20, 0x00])))
recs.append(rec(LE_STATE, 0, 40000, 2402, bytes([1, 2, 0x20, 0x00, 2, 2])))
# a maximal 251 byte payload (6 chunks)
recs += le_packet(50000, 2410, AA, blesim.data_pdu(2, bytes(251), 0, 0), 0x01 | 0x10 | 0x40 | 0x80, 7)
open(sys.argv[1], 'wb').write(b''.join(recs))
print('%d records' % len(recs))
