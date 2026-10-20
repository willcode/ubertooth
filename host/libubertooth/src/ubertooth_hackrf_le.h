/*
 * LE packet search over the HackRF Pro BR channelizer's decisions.
 *
 * The BR gateware makes one decision per symbol for each 1 MHz channel of
 * its window. LE channels lie on the same raster (the LE channel at
 * 2402 + 2k MHz is BR channel 2k) and LE 1M has the same symbol rate: the
 * decisions of the window's even channels are LE bits. This module searches
 * them for the advertising access address and for the access addresses of
 * connections whose CONNECT_IND it saw, dewhitens, checks the CRC and hands
 * each packet over in the layout the Ubertooth firmware sends.
 *
 * It does not use libhackrf.
 *
 * This file is part of Project Ubertooth (GPL-2.0-or-later).
 */
#ifndef __UBERTOOTH_HACKRF_LE_H__
#define __UBERTOOTH_HACKRF_LE_H__

#include <stdint.h>

#define HLE_ADV_AA        0x8E89BED6u
#define HLE_MAX_BYTES     (4 + 2 + 255 + 3)   /* AA + header + payload + CRC */

typedef struct {
	uint32_t aa;            /* the access address searched for (errors corrected) */
	int freq;               /* MHz */
	int chan_idx;           /* LE channel index, 0-39 */
	uint64_t word;          /* word (symbol) index of the first access address bit */
	uint8_t data[HLE_MAX_BYTES]; /* AA (LSB first), dewhitened PDU and CRC */
	unsigned len;           /* bytes in data */
	int aa_errors;          /* access address bits that differed */
	int is_data;            /* a PDU of a connection from a seen CONNECT_IND */
	int slave;              /* is_data: sent by the slave (inferred from T_IFS spacing) */
	int aux;                /* advertising access address on a data channel */
	int corrected;          /* bit after the access address the CRC corrected, or -1 */
} hle_packet;

typedef struct {
	unsigned long matches;      /* access address matches */
	unsigned long crc_ok;       /* packets with a good CRC (handed over) */
	unsigned long corrected;    /* ... of which one bit was corrected */
	unsigned long crc_bad;      /* candidates whose CRC failed, copies on neighboring
	                               channels included */
	unsigned long aa_err_hist[4]; /* good packets by access address errors */
	unsigned long connections;  /* CONNECT_INDs taken up */
} hle_stats;

typedef void (*hle_emit_fn)(void* user, const hle_packet* p);

/*
 * max_aa_errors: access address bits that may differ (0-3).
 * correct: nonzero to correct a single bit error from the CRC. The gateware's
 * threshold follows the data, and the bit after a long run of equal bits is
 * sometimes wrong even on a clean signal; one wrong bit fails the CRC. On a
 * primary advertising channel only a PDU of a legacy length (6-37 bytes) is
 * corrected.
 * Up to 8 connections are searched; each is dropped twice its supervision
 * timeout (at least 1 s) after its last good packet.
 */
void hle_init(hle_emit_fn emit, void* user, int max_aa_errors, int correct);

/*
 * Search one bank of words: bit k of a word is the decision for the channel
 * at center + k MHz (k >= 9 meaning k - 16; bit 8 is not a channel).
 * start_word is the index of words[0]. A gap in the word indices or a new
 * center restarts every channel's search.
 */
void hle_bank(const uint16_t* words, int n, uint64_t start_word, int center);

void hle_get_stats(hle_stats* s);

/* LE channel index (0-39) of a frequency in MHz, or -1 */
int hle_chan_idx(int freq);

#endif /* __UBERTOOTH_HACKRF_LE_H__ */
