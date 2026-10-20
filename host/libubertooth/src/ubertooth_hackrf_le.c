/*
 * LE packet search over the HackRF Pro BR channelizer's decisions (see
 * ubertooth_hackrf_le.h).
 *
 * Each LE channel of the window keeps the last 32 decisions. When they are
 * within max_aa_errors bits of a known access address (the advertising one
 * on every channel, a seen connection's on data channels), the following
 * bits are dewhitened and collected: the header gives the length, and the
 * CRC over header and payload is checked against the 24 bits after them.
 * The preamble is not checked: the gateware's per-channel threshold settles
 * at the start of a transmission, where most bit errors fall.
 *
 * A strong packet can also decode on a neighboring channel of the window;
 * there it is dewhitened with that channel's sequence and fails the CRC.
 *
 * This file is part of Project Ubertooth (GPL-2.0-or-later).
 */
#include <string.h>

#include "ubertooth_hackrf_le.h"

#define NBINS          16
#define ADV_CRC_INIT   0x555555u
#define MAX_CONNS      8
#define PDU_CONNECT    0x5      /* CONNECT_IND, AUX_CONNECT_REQ */
#define CONNECT_LEN    34
#define T_IFS_MIN      145      /* words (us) from a packet's end to the next one's preamble */
#define T_IFS_MAX      158
#define LEGACY_MIN     6        /* primary advertising channels: legacy PDU lengths */
#define LEGACY_MAX     37
#define MAX_PENDING    64       /* packets of one bank, handed over in time order */

typedef struct {
	uint32_t aa;
	uint32_t crc_init;
	uint64_t last_word;     /* end of the last good packet (or the CONNECT_IND) */
	uint64_t expire_words;  /* forgotten this long after last_word */
	/* per data channel: the end of the last candidate and its direction.
	 * A copy that a strong packet leaves on a neighboring channel then
	 * cannot break the alternation on its own channel. */
	uint64_t ch_end[37];
	uint8_t ch_slave[37];
	int used;
} conn_t;

typedef struct {
	int freq;               /* MHz */
	int chan_idx;           /* -1: not an LE channel or outside the band */
	uint32_t sr;            /* last 32 decisions, the newest in bit 31 */
	int seen;               /* decisions since the search restarted, up to 32 */

	int collecting;
	int conn;               /* index into conns, or -1 for advertising */
	uint32_t aa;
	uint32_t crc;           /* CRC register over the PDU (bit-reversed form) */
	uint32_t rx_crc;
	uint64_t aa_word;
	int aa_errors;
	uint8_t whiten;         /* 7-bit LFSR, position i in bit i */
	unsigned nbits;         /* bits collected after the access address */
	unsigned pdu_bits;      /* header + payload bits (16 until the header is in) */
	uint8_t buf[HLE_MAX_BYTES - 4];
} bin_t;

static struct {
	hle_emit_fn emit;
	void* user;
	int max_aa_errors;
	int correct;
	int center;
	uint64_t next_word;
	int started;
	bin_t bin[NBINS];
	conn_t conns[MAX_CONNS];
	hle_packet pending[MAX_PENDING];
	int npending;
	hle_stats st;
} G;

int hle_chan_idx(int freq)
{
	if (freq == 2402)
		return 37;
	if (freq == 2426)
		return 38;
	if (freq == 2480)
		return 39;
	if (freq < 2404 || freq > 2478 || (freq - 2402) % 2)
		return -1;
	return freq < 2426 ? (freq - 2404) / 2 : (freq - 2428) / 2 + 11;
}

static int popcount32(uint32_t v)
{
	return __builtin_popcount(v);
}

static uint32_t rev24(uint32_t v)
{
	uint32_t r = 0;
	int i;
	for (i = 0; i < 24; i++)
		r |= ((v >> i) & 1u) << (23 - i);
	return r;
}

void hle_init(hle_emit_fn emit, void* user, int max_aa_errors, int correct)
{
	memset(&G, 0, sizeof(G));
	G.emit = emit;
	G.user = user;
	G.max_aa_errors = max_aa_errors < 0 ? 0 : max_aa_errors > 3 ? 3 : max_aa_errors;
	G.correct = correct;
}

/* the CRC register after one more PDU bit d */
static uint32_t crc_step(uint32_t crc, int d)
{
	int nb = (int)((crc ^ (uint32_t)d) & 1u);
	crc >>= 1;
	return nb ? (crc | (1u << 23)) ^ 0x5A6000u : crc;
}

/*
 * The bit whose error explains the CRC mismatch (computed ^ received), or -1.
 * An error in a received CRC bit flips that bit of the mismatch alone. The
 * CRC is linear: an error in PDU bit i changes the computed CRC by an impulse
 * at i carried through the remaining bits, which is found from the last PDU
 * bit backward, one zero step per bit. The length byte is left out: an error
 * there would have changed which bits are the CRC.
 */
static int correctable_bit(uint32_t syndrome, unsigned pdu_bits)
{
	uint32_t e = crc_step(0, 1);
	int i;

	if (popcount32(syndrome) == 1)
		return (int)pdu_bits + __builtin_ctz(syndrome);
	for (i = (int)pdu_bits - 1; i >= 0; i--) {
		if (e == syndrome)
			return i >= 8 && i < 16 ? -1 : i;
		e = crc_step(e, 0);
	}
	return -1;
}

void hle_get_stats(hle_stats* s)
{
	*s = G.st;
}

static void restart(int center)
{
	int k;
	G.center = center;
	for (k = 0; k < NBINS; k++) {
		bin_t* b = &G.bin[k];
		int off = k < 8 ? k : k - 16;
		memset(b, 0, sizeof(*b));
		b->freq = center + off;
		b->chan_idx = k == 8 ? -1 : hle_chan_idx(b->freq);
	}
}

static void start_candidate(bin_t* b, uint32_t aa, uint32_t crc_init, int conn,
                            uint64_t word, int errors)
{
	b->collecting = 1;
	b->aa = aa;
	b->conn = conn;
	b->aa_word = word - 31;
	b->aa_errors = errors;
	b->crc = rev24(crc_init);
	b->rx_crc = 0;
	b->nbits = 0;
	b->pdu_bits = 16;
	/* whitening: position 0 is 1, positions 1-6 the channel index MSB first */
	b->whiten = 1;
	{
		int i;
		for (i = 1; i <= 6; i++)
			b->whiten |= (uint8_t)(((b->chan_idx >> (6 - i)) & 1) << i);
	}
	memset(b->buf, 0, sizeof(b->buf));
	G.st.matches++;
}

static conn_t* conn_for(uint32_t aa)
{
	int i;
	for (i = 0; i < MAX_CONNS; i++)
		if (G.conns[i].used && G.conns[i].aa == aa)
			return &G.conns[i];
	return NULL;
}

/* a CONNECT_IND (or AUX_CONNECT_REQ) seen: search its access address */
static void take_connect(const uint8_t* pdu, uint64_t end_word)
{
	const uint8_t* ll = pdu + 2 + 12;      /* after the header, InitA and AdvA */
	uint32_t aa = ll[0] | ll[1] << 8 | ll[2] << 16 | (uint32_t)ll[3] << 24;
	uint32_t crc_init = ll[4] | ll[5] << 8 | ll[6] << 16;
	unsigned timeout_10ms = ll[14] | ll[15] << 8;
	conn_t* c = conn_for(aa);
	int i;

	if (aa == HLE_ADV_AA)
		return;
	if (c == NULL) {
		c = &G.conns[0];
		for (i = 0; i < MAX_CONNS; i++) {
			if (!G.conns[i].used) {
				c = &G.conns[i];
				break;
			}
			if (G.conns[i].last_word < c->last_word)
				c = &G.conns[i];
		}
		G.st.connections++;
	}
	memset(c, 0, sizeof(*c));
	c->used = 1;
	c->aa = aa;
	c->crc_init = crc_init;
	c->last_word = end_word;
	/* twice the supervision timeout, at least a second: the window sees
	 * only some of the connection's channels */
	c->expire_words = 2ull * timeout_10ms * 10000;
	if (c->expire_words < 1000000)
		c->expire_words = 1000000;
}

/*
 * Direction of a connection's packet: one that starts T_IFS after the
 * previous one on the same channel answers it, and the direction alternates
 * within an event. Any other opens an event and is the master's. Candidates
 * with a bad CRC take part: a damaged master packet still makes its reply the
 * slave's. A master packet not heard at all does not.
 */
static int direction(conn_t* c, const bin_t* b, uint64_t end_word)
{
	uint64_t start = b->aa_word - 8;   /* preamble */
	int k = b->chan_idx, slave = 0;

	if (c->ch_end[k] && start >= c->ch_end[k] + T_IFS_MIN && start <= c->ch_end[k] + T_IFS_MAX)
		slave = !c->ch_slave[k];
	c->ch_end[k] = end_word;
	c->ch_slave[k] = (uint8_t)slave;
	return slave;
}

static void hand_over(const hle_packet* p)
{
	if (G.npending < MAX_PENDING)
		G.pending[G.npending++] = *p;
	else if (G.emit)
		G.emit(G.user, p);
}

static void finish(bin_t* b)
{
	unsigned pdu_bytes = b->pdu_bits / 8, i;
	uint64_t end_word = b->aa_word + 32 + b->pdu_bits + 24;
	int err = b->aa_errors > 3 ? 3 : b->aa_errors, fix = -1, slave = 0;
	hle_packet p;

	b->collecting = 0;
	if (b->conn >= 0)
		slave = direction(&G.conns[b->conn], b, end_word);
	if (b->crc != b->rx_crc) {
		fix = G.correct ? correctable_bit(b->crc ^ b->rx_crc, b->pdu_bits) : -1;
		/* on a primary advertising channel, only a legacy length: random
		 * decisions after a chance access address match would otherwise
		 * pass now and then */
		if (b->conn < 0 && b->chan_idx >= 37 &&
		    (b->buf[1] < LEGACY_MIN || b->buf[1] > LEGACY_MAX))
			fix = -1;
		if (fix < 0) {
			G.st.crc_bad++;
			return;
		}
		b->buf[fix >> 3] ^= (uint8_t)(1u << (fix & 7));
		G.st.corrected++;
	}
	G.st.crc_ok++;
	G.st.aa_err_hist[err]++;

	memset(&p, 0, sizeof(p));
	p.corrected = fix;
	p.aa = b->aa;
	p.freq = b->freq;
	p.chan_idx = b->chan_idx;
	p.word = b->aa_word;
	p.aa_errors = b->aa_errors;
	for (i = 0; i < 4; i++)
		p.data[i] = (uint8_t)(b->aa >> (8 * i));
	memcpy(p.data + 4, b->buf, pdu_bytes + 3);
	p.len = 4 + pdu_bytes + 3;

	if (b->conn < 0) {
		p.aux = b->chan_idx < 37;
		if ((b->buf[0] & 0x0f) == PDU_CONNECT && b->buf[1] == CONNECT_LEN)
			take_connect(b->buf, end_word);
	} else {
		p.is_data = 1;
		p.slave = slave;
		G.conns[b->conn].last_word = end_word;
	}
	hand_over(&p);
}

static void collect(bin_t* b, int bit)
{
	unsigned i = b->nbits++;
	int w = (b->whiten >> 6) & 1;
	int d = bit ^ w;

	b->whiten = (uint8_t)(((b->whiten << 1) & 0x7f) | w);
	if (w)
		b->whiten ^= 1u << 4;

	if (d)
		b->buf[i >> 3] |= (uint8_t)(1u << (i & 7));
	if (i < b->pdu_bits) {
		b->crc = crc_step(b->crc, d);
		if (i == 15)
			b->pdu_bits = 16 + 8u * b->buf[1];
	} else {
		b->rx_crc |= (uint32_t)d << (i - b->pdu_bits);
		if (i == b->pdu_bits + 23)
			finish(b);
	}
}

static void search(bin_t* b, uint64_t word)
{
	uint32_t win = b->sr;
	int e = popcount32(win ^ HLE_ADV_AA), i, best = -1, best_e;

	if (e <= G.max_aa_errors) {
		start_candidate(b, HLE_ADV_AA, ADV_CRC_INIT, -1, word, e);
		return;
	}
	if (b->chan_idx >= 37)
		return;
	best_e = G.max_aa_errors + 1;
	for (i = 0; i < MAX_CONNS; i++) {
		conn_t* c = &G.conns[i];
		if (!c->used)
			continue;
		if (word > c->last_word + c->expire_words) {
			c->used = 0;
			continue;
		}
		e = popcount32(win ^ c->aa);
		if (e < best_e) {
			best_e = e;
			best = i;
		}
	}
	if (best >= 0)
		start_candidate(b, G.conns[best].aa, G.conns[best].crc_init, best, word, best_e);
}

void hle_bank(const uint16_t* words, int n, uint64_t start_word, int center)
{
	int s, k;

	if (!G.started || start_word != G.next_word || center != G.center) {
		restart(center);
		G.started = 1;
	}
	G.next_word = start_word + (uint64_t)n;

	for (k = 0; k < NBINS; k++) {
		bin_t* b = &G.bin[k];
		if (b->chan_idx < 0)
			continue;
		for (s = 0; s < n; s++) {
			int bit = (words[s] >> k) & 1;
			uint64_t word = start_word + (uint64_t)s;
			b->sr = (b->sr >> 1) | ((uint32_t)bit << 31);
			if (b->collecting) {
				collect(b, bit);
				continue;
			}
			if (b->seen < 32 && ++b->seen < 32)
				continue;
			search(b, word);
		}
	}

	/* the bins ran one after another: hand the bank's packets over in time
	 * order */
	for (k = 1; k < G.npending; k++) {
		hle_packet p = G.pending[k];
		for (s = k; s > 0 && G.pending[s - 1].word > p.word; s--)
			G.pending[s] = G.pending[s - 1];
		G.pending[s] = p;
	}
	for (k = 0; k < G.npending; k++)
		if (G.emit)
			G.emit(G.user, &G.pending[k]);
	G.npending = 0;
}
