# blesim: HackRF + Ubertooth BLE tools

Python tools (numpy only) for pairing a HackRF with an Ubertooth.

| Tool | What it does |
|---|---|
| `blesim.py` | Synthesizes bit-exact BLE scenarios as int8 IQ for `hackrf_transfer -t`, plus a ground-truth JSON file |
| `blerx.py` | Wideband LE 1M/2M receiver for HackRF IQ (`hackrf_transfer -r`) |
| `blefuse.py` | Fuses an Ubertooth connection capture with a HackRF recording (recovers LE 2M traffic) |
| `blecoded.py` | LE Coded (long range) receiver for raw Ubertooth symbols |
| `lepcapng.py` | pcapng reader/writer for LE link-layer captures (LE 2M, direction, CRC flags) |

The defaults assume the HackRF is tuned to 2431 MHz at 20 Msps. That covers
advertising channel 38 (2426 MHz) and data channels 10–16.

## Test transmitter

```bash
python3 blesim.py conn --csa 2 --encrypt --phy2m -o conn.iq  # also writes conn.json
hackrf_transfer -t conn.iq -f 2431000000 -s 20000000 -x 16 -a 0
```

The scenarios are `adv`, `extadv` (ADV_EXT_IND with an AuxPtr), `coded`
(LE Coded advertising), `conn` (CONNECT_IND, then a hopping connection; add
`--conn-update`, `--chmap-update`, `--encrypt` and `--phy2m` for the LL
procedures, and `--forge-slave-update`, `--enc-ctrl6`, `--enc-ctrl9`,
`--terminate` and `--fade` for what a follower must not be misled by), and
`midconn` (a connection that is already running, for `ubertooth-btle -P`;
`--conn-update` adds a plaintext update late in it). `selftest` checks the
encoder against the specification's test vectors.

`hackrf_transfer` must keep up with 2 bytes per sample, 40 MB/s at 20 Msps.
On a busy host it falls behind and the HackRF sends silence in the middle
of packets, which looks like bad reception (its summary then shows less
than 40 MB/second); the start of a transmission can be lost the same way.
Use a lower rate with data channels that fit it, for example
`--fc 2428 --fs 8e6 --chm 11,12` and `hackrf_transfer -f 2428000000 -s 8000000`,
and start the receiver first.

The HackRF is a transmitter. Use low gain (`-x`), short runs, and only where
transmitting in the 2.4 GHz ISM band is permitted.

## LE 2M traffic (Ubertooth + HackRF)

Run both radios at the same time, then fuse the two captures:

```bash
ubertooth-btle -f -A38 -r ut.pcapng
hackrf_transfer -r rx.iq -f 2431000000 -s 20000000
python3 blefuse.py ut.pcapng rx.iq --fc 2431 -o fused.pcapng
```

## LE Coded

```bash
ubertooth-util -c2426
ubertooth-dump -l -b > coded.txt
python3 blecoded.py coded.txt --channel 38
```
