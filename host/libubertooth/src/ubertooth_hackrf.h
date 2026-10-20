/*
 * HackRF Pro backend: a virtual Ubertooth for Bluetooth BR and LE 1M.
 *
 * With the BR channelizer gateware loaded (from hackrf-ubertooth), a HackRF Pro
 * receives 15 BR channels at once and libubertooth delivers them as
 * ordinary BR_PACKET usb_pkt_rx packets, one per channel per 400 symbols.
 * For ubertooth-btle -n or -f it finds LE 1M packets on the window's even
 * channels and delivers them as the firmware does in follow mode.
 *
 * A qualifying HackRF Pro (firmware with the RAM gateware loader, and the
 * gateware pack installed) is listed after the Ubertooths: with N
 * Ubertooths attached it is device N, selected with -U N. Selecting it loads
 * the gateware into the FPGA from RAM; closing it, exiting (normally, by a
 * signal, or a crash) switches back to the stock image in flash. Options:
 *   UBERTOOTH_HACKRF_PACK     the gateware pack (default: installed
 *                             share/ubertooth/br_rx.pack)
 *   UBERTOOTH_HACKRF_GAIN     "lna,vga" (default "16,16")
 *   UBERTOOTH_HACKRF_SCAN_MS  dwell per 15 MHz window when scanning (default
 *                             100). The band is scanned when a tool asks the
 *                             Ubertooth to sweep (no channel given, e.g.
 *                             ubertooth-rx without -c), in AFH mode, or when
 *                             this is set and no channel is given
 *   UBERTOOTH_HACKRF_CENTER   initial window center in MHz (default 2441), for
 *                             tools without a channel option
 *   UBERTOOTH_HACKRF_LEAK     leak suppression margin in 6 dB steps (1-6,
 *                             default 3; 0 turns it off): a channel near the
 *                             strongest one and this far below it is muted
 *   UBERTOOTH_HACKRF_LE_CENTER     LE window center in MHz, instead of the
 *                                  advertising channel's placement (unset)
 *   UBERTOOTH_HACKRF_LE_AA_ERRORS  LE access address bits that may differ,
 *                                  0-3 (default 2)
 *   UBERTOOTH_HACKRF_LE_CORRECT    1 corrects one bit error per LE packet
 *                                  from its CRC, 0 does not (default 1)
 *   UBERTOOTH_HACKRF_LE_THRESHOLD  1 selects the gateware's slower threshold
 *                                  in LE mode, 0 keeps BR's (default 1)
 *
 * This file is part of Project Ubertooth (GPL-2.0-or-later).
 */
#ifndef UBERTOOTH_HACKRF_H
#define UBERTOOTH_HACKRF_H

#include <stddef.h>
#include <stdint.h>

struct ubertooth_t_;

/* qualifying HackRF Pros attached (0 without the gateware pack) */
int ubertooth_hackrf_count(void);
/* a line describing the n-th one; 0 on success */
int ubertooth_hackrf_describe(int n, char* buf, size_t len);
/* nonzero while a HackRF backend is open in this process */
int ubertooth_hackrf_active(void);

/* load the gateware into the n-th one, open and configure it; 0 on
 * success. ut is a ubertooth_t*. */
int ubertooth_hackrf_open(void* ut, int n);
/* switch back to the stock gateware now (for a program's fast exit) */
void ubertooth_hackrf_emergency_restore(void);
void ubertooth_hackrf_close(void* ut);

/* emulated control request; the return value follows libusb_control_transfer */
int ubertooth_hackrf_control(uint8_t request_type, uint8_t request,
                             uint16_t value, uint16_t index,
                             unsigned char* data, uint16_t length);

/* streaming, driven by the bulk calls */
int ubertooth_hackrf_stream_start(void);
void ubertooth_hackrf_stream_stop(void);

#endif
