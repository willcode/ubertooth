/*
 * Copyright 2026 Project Ubertooth
 *
 * This file is part of Project Ubertooth.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 */

#include <stdlib.h>
#include <string.h>

#include "ubertooth_le_recover.h"

#define TICKS_PER_USEC 10
#define INTERVAL_UNIT_TICKS (1250 * TICKS_PER_USEC)

// packets closer than this on the same channel are one connection event
// (the shortest connection interval is 7.5 ms)
#define SAME_EVENT_TICKS (5000 * TICKS_PER_USEC)

void le_recover_init(le_recover_t *r, uint32_t access_address) {
	memset(r, 0, sizeof(*r));
	r->access_address = access_address;
}

static uint32_t reverse24(uint32_t v) {
	uint32_t r = 0;
	int i;
	for (i = 0; i < 24; ++i)
		r |= ((v >> i) & 1) << (23 - i);
	return r;
}

/*
 * The CRC LFSR (Core Spec Vol 6 Part B 3.1.1), kept bit reversed so data
 * bits are shifted in LSB first:
 *   nb = (state ^ bit) & 1; state >>= 1; if (nb) state ^= 0xda6000
 * (0x5a6000 plus the feedback into bit 23). Running it backwards from the
 * transmitted CRC recovers the initial state, i.e. CRCInit.
 */
uint32_t le_recover_crc_init(const uint8_t *pdu, unsigned len, const uint8_t crc[3]) {
	uint32_t state = crc[0] | (crc[1] << 8) | (crc[2] << 16);
	int i, j;

	for (i = (int)len - 1; i >= 0; --i) {
		for (j = 7; j >= 0; --j) {
			unsigned bit = (pdu[i] >> j) & 1;
			unsigned nb = (state >> 23) & 1;
			if (nb)
				state ^= 0xda6000;
			// before the shift, bit 0 was nb ^ bit
			state = ((state << 1) | (nb ^ bit)) & 0xffffff;
		}
	}

	return reverse24(state);
}

static void crc_vote(le_recover_t *r, uint32_t crc_init) {
	unsigned i, weakest = 0;

	for (i = 0; i < LE_RECOVER_CRC_SLOTS; ++i) {
		if (r->crc[i].votes && r->crc[i].crc_init == crc_init) {
			++r->crc[i].votes;
			return;
		}
		if (r->crc[i].votes < r->crc[weakest].votes)
			weakest = i;
	}
	r->crc[weakest].crc_init = crc_init;
	r->crc[weakest].votes = 1;
}

void le_recover_packet(le_recover_t *r, uint32_t timestamp, uint8_t channel_idx,
		const uint8_t *pdu, unsigned len) {
	le_recover_event_t *last;

	if (channel_idx > 36)
		return;

	if (len >= 5 && pdu[1] + 5u == len) {
		uint32_t ci = le_recover_crc_init(pdu, len - 3, pdu + len - 3), best;
		crc_vote(r, ci);
		++r->crc_packets;
		// once a CRCInit dominates, a packet that disagrees is a false
		// sync or corrupted: it must not create an event
		if (le_recover_get_crc(r, &best) >= 3 && ci != best)
			return;
	}

	last = r->num_events ? &r->events[r->num_events - 1] : NULL;
	if (last && last->channel == channel_idx &&
			(uint32_t)(timestamp - last->anchor) < SAME_EVENT_TICKS) {
		++last->packets;
		return;
	}

	if (r->num_events == LE_RECOVER_MAX_EVENTS) {
		// keep the most recent half
		memmove(r->events, r->events + LE_RECOVER_MAX_EVENTS / 2,
				sizeof(r->events[0]) * (LE_RECOVER_MAX_EVENTS / 2));
		r->num_events = LE_RECOVER_MAX_EVENTS / 2;
	}
	r->events[r->num_events].anchor = timestamp;
	r->events[r->num_events].channel = channel_idx;
	r->events[r->num_events].packets = 1;
	++r->num_events;
}

unsigned le_recover_get_crc(const le_recover_t *r, uint32_t *crc_init) {
	unsigned i, best = 0;

	for (i = 1; i < LE_RECOVER_CRC_SLOTS; ++i)
		if (r->crc[i].votes > r->crc[best].votes)
			best = i;
	if (crc_init)
		*crc_init = r->crc[best].crc_init;
	return r->crc[best].votes;
}

unsigned le_recover_channels_seen(const le_recover_t *r) {
	uint64_t seen = 0;
	unsigned i, n = 0;

	for (i = 0; i < r->num_events; ++i)
		seen |= 1ull << r->events[i].channel;
	for (i = 0; i < 37; ++i)
		n += (seen >> i) & 1;
	return n;
}

// tolerance for a gap: timestamp jitter plus 100 ppm of combined drift
static uint32_t gap_tolerance(uint32_t gap) {
	return 60 * TICKS_PER_USEC + gap / 10000;
}

// number of intervals in a gap, or 0 if the gap is not a multiple
static uint32_t gap_events(uint32_t gap, uint32_t interval_ticks) {
	uint32_t k = (gap + interval_ticks / 2) / interval_ticks;
	uint32_t err;

	if (k == 0)
		return 0;
	err = gap > k * interval_ticks ? gap - k * interval_ticks : k * interval_ticks - gap;
	return err <= gap_tolerance(gap) ? k : 0;
}

/*
 * An event with two or more packets starts with the master's packet, so
 * its anchor is exact. A single packet may be the slave's reply to a
 * missed master packet: 80 + 150 us to ~2.5 ms after the anchor.
 */
#define SLAVE_DELAY_MIN (230 * TICKS_PER_USEC)
#define SLAVE_DELAY_MAX (2500 * TICKS_PER_USEC)

static int certain(const le_recover_event_t *e) {
	return e->packets >= 2;
}

uint16_t le_recover_interval(const le_recover_t *r, unsigned min_events) {
	unsigned n, i, num_certain = 0;
	int use_all;

	if (r->num_events < min_events || r->num_events < 2)
		return 0;

	// use events with exact anchors when there are enough of them (a
	// slave that never answers leaves only single packet events)
	for (i = 0; i < r->num_events; ++i)
		num_certain += certain(&r->events[i]);
	use_all = num_certain < min_events;

	// every gap between consecutive events is a whole number of
	// intervals: find the largest interval that divides (nearly) all of
	// them; a few gaps may be broken by false syncs or bad timestamps
	for (n = 3200; n >= 6; --n) {
		uint32_t interval_ticks = n * INTERVAL_UNIT_TICKS;
		unsigned gaps = 0, bad = 0, prev = r->num_events;
		for (i = 0; i < r->num_events; ++i) {
			if (!use_all && !certain(&r->events[i]))
				continue;
			if (prev != r->num_events) {
				uint32_t gap = r->events[i].anchor - r->events[prev].anchor;
				++gaps;
				bad += gap_events(gap, interval_ticks) == 0;
			}
			prev = i;
		}
		if (gaps > 0 && bad * 10 <= gaps)
			return n;
	}
	return 0;
}

uint16_t le_recover_csa2_prn_e(uint16_t counter, uint16_t channel_id) {
	uint16_t prn = counter ^ channel_id;
	int i;

	for (i = 0; i < 3; ++i) {
		// permute: bit-reverse each byte
		uint16_t lo = prn & 0xff, hi = prn >> 8, rlo = 0, rhi = 0;
		int b;
		for (b = 0; b < 8; ++b) {
			rlo |= ((lo >> b) & 1) << (7 - b);
			rhi |= ((hi >> b) & 1) << (7 - b);
		}
		prn = (rhi << 8) | rlo;
		// multiply, add, modulo
		prn = (uint16_t)(17 * prn + channel_id);
	}

	return prn ^ channel_id;
}

// event index of each observed event relative to the first one. Events
// that are not on the event grid of the previous good event (false
// syncs) get offset UINT32_MAX and are ignored by the searches.
#define OFF_GRID UINT32_MAX
static int event_offsets(const le_recover_t *r, uint16_t interval, uint32_t *offsets) {
	uint32_t interval_ticks = interval * INTERVAL_UNIT_TICKS;
	unsigned i, last = 0, bad = 0;

	offsets[0] = 0;
	for (i = 1; i < r->num_events; ++i) {
		uint32_t gap = r->events[i].anchor - r->events[last].anchor;
		uint32_t k = gap_events(gap, interval_ticks);
		if (k == 0 && !certain(&r->events[i])) {
			// a slave packet after a missed master packet: its anchor
			// is up to SLAVE_DELAY_MAX earlier
			uint32_t d;
			for (d = SLAVE_DELAY_MIN; d <= SLAVE_DELAY_MAX && k == 0;
					d += 100 * TICKS_PER_USEC)
				k = gap > d ? gap_events(gap - d, interval_ticks) : 0;
		}
		if (k == 0) {
			offsets[i] = OFF_GRID;
			++bad;
			continue;
		}
		offsets[i] = offsets[last] + k;
		// only exact anchors become the reference for later events
		if (certain(&r->events[i]) || !certain(&r->events[last]))
			last = i;
	}
	return bad * 10 <= r->num_events;
}

typedef struct {
	long best, runner_up;      // scores
	unsigned matched, predicted; // of the best candidate
	uint32_t a, b;             // solution parameters
} search_t;

static void search_update(search_t *s, long score, unsigned matched,
		unsigned predicted, uint32_t a, uint32_t b) {
	if (score > s->best) {
		s->runner_up = s->best;
		s->best = score;
		s->matched = matched;
		s->predicted = predicted;
		s->a = a;
		s->b = b;
	} else if (score > s->runner_up) {
		s->runner_up = score;
	}
}

/*
 * Monitoring windows: runs of consecutive events on the same channel.
 * The monitor listened to that channel for the whole run, so a correct
 * solution must not predict (direct) events in the run that were not
 * observed, beyond packet loss.
 *
 * score = matched - (predicted - matched)
 */
typedef struct {
	uint8_t channel;
	uint32_t lo, hi;   // event offsets of the first and last event
} run_t;

static unsigned find_runs(const le_recover_t *r, const uint32_t *offsets, run_t *runs) {
	unsigned i, n = 0;
	for (i = 0; i < r->num_events; ++i) {
		if (offsets[i] == OFF_GRID)
			continue;
		if (n && runs[n - 1].channel == r->events[i].channel) {
			runs[n - 1].hi = offsets[i];
		} else {
			runs[n].channel = r->events[i].channel;
			runs[n].lo = runs[n].hi = offsets[i];
			++n;
		}
	}
	return n;
}

// number of y in [a, a + len) (mod 65536) with unmapped[y] == channel,
// using the prefix sums of that channel
static uint32_t prefix_count(const uint32_t *prefix, uint32_t a, uint32_t len) {
	uint32_t total = (len >> 16) * prefix[65536];
	a &= 0xffff;
	len &= 0xffff;
	if (a + len <= 65536)
		return total + prefix[a + len] - prefix[a];
	return total + prefix[65536] - prefix[a] + prefix[a + len - 65536];
}

typedef struct {
	uint8_t in_use[37];
	uint8_t used[37];
	unsigned num_used;
} chmap_t;

static void chmap_parse(const uint8_t chm[5], chmap_t *m) {
	unsigned i;
	m->num_used = 0;
	for (i = 0; i < 37; ++i) {
		m->in_use[i] = (chm[i / 8] >> (i % 8)) & 1;
		if (m->in_use[i])
			m->used[m->num_used++] = i;
	}
}

// CSA#2: find the event counter of the first observed event
static search_t search_csa2(const le_recover_t *r, const uint32_t *offsets,
		const chmap_t *map) {
	uint16_t channel_id = (r->access_address >> 16) ^ (r->access_address & 0xffff);
	search_t s = { -1000000, -1000000, 0, 0, 0, 0 };
	uint8_t *unmapped = malloc(65536);
	run_t *runs = malloc(sizeof(run_t) * r->num_events);
	uint32_t *prefix[37] = { 0 };
	unsigned num_runs, i;
	uint32_t c;

	if (unmapped == NULL || runs == NULL)
		goto out;
	// the channel of every counter value (named "unmapped" for history:
	// with a full map they are the same)
	for (c = 0; c < 65536; ++c) {
		uint16_t prn_e = le_recover_csa2_prn_e(c, channel_id);
		uint8_t u = prn_e % 37;
		unmapped[c] = map->in_use[u] ? u : map->used[(map->num_used * prn_e) >> 16];
	}

	num_runs = find_runs(r, offsets, runs);
	for (i = 0; i < num_runs; ++i) {
		uint8_t ch = runs[i].channel;
		if (prefix[ch])
			continue;
		prefix[ch] = malloc(sizeof(uint32_t) * 65537);
		if (prefix[ch] == NULL)
			goto out;
		prefix[ch][0] = 0;
		for (c = 0; c < 65536; ++c)
			prefix[ch][c + 1] = prefix[ch][c] + (unmapped[c] == ch);
	}

	for (c = 0; c < 65536; ++c) {
		unsigned matched = 0, predicted = 0;
		for (i = 0; i < r->num_events; ++i)
			matched += offsets[i] != OFF_GRID &&
				unmapped[(c + offsets[i]) & 0xffff] == r->events[i].channel;
		for (i = 0; i < num_runs; ++i)
			predicted += prefix_count(prefix[runs[i].channel], c + runs[i].lo,
					runs[i].hi - runs[i].lo + 1);
		search_update(&s, 2 * (long)matched - predicted, matched, predicted, c, 0);
	}

out:
	for (i = 0; i < 37; ++i)
		free(prefix[i]);
	free(runs);
	free(unmapped);
	return s;
}

// CSA#1: find the hop increment and the unmapped channel of the first event
static search_t search_csa1(const le_recover_t *r, const uint32_t *offsets,
		const chmap_t *map) {
	search_t s = { -1000000, -1000000, 0, 0, 0, 0 };
	run_t *runs = malloc(sizeof(run_t) * r->num_events);
	unsigned hop, u0, i, k, num_runs;

	if (runs == NULL)
		return s;
	num_runs = find_runs(r, offsets, runs);

	for (hop = 5; hop <= 16; ++hop) {
		for (u0 = 0; u0 < 37; ++u0) {
			unsigned matched = 0, predicted = 0;
			uint8_t chan[37];   // the channel of event e, by e mod 37
			for (k = 0; k < 37; ++k) {
				unsigned u = (u0 + k * hop) % 37;
				chan[k] = map->in_use[u] ? u : map->used[u % map->num_used];
			}
			for (i = 0; i < r->num_events; ++i)
				if (offsets[i] != OFF_GRID)
					matched += chan[offsets[i] % 37] == r->events[i].channel;
			// a run predicts its channel on every event it spans: count
			// whole cycles of 37 at once, so that the cost does not grow
			// with the run's length
			for (i = 0; i < num_runs; ++i) {
				uint32_t len = runs[i].hi >= runs[i].lo ? runs[i].hi - runs[i].lo + 1 : 0;
				unsigned in_cycle = 0, r0 = runs[i].lo % 37;
				for (k = 0; k < 37 && k < len; ++k)
					if (chan[(r0 + k) % 37] == runs[i].channel) {
						in_cycle += len / 37 + (k < len % 37);
					}
				predicted += in_cycle;
			}
			search_update(&s, 2 * (long)matched - predicted, matched, predicted, hop, u0);
		}
	}
	free(runs);
	return s;
}

// a solution must explain enough events (at least 30% of them: with an
// assumed full map, remapped events are not explained), lose no more
// than half of the events it predicts, and clearly beat the runner up
static int acceptable(const search_t *s, unsigned events) {
	return s->matched >= 8 && s->matched * 10 >= events * 3 &&
		s->predicted <= 2 * s->matched &&
		s->best > 0 && s->runner_up * 2 < s->best;
}

static int solve(const le_recover_t *r, uint16_t interval, const uint8_t chm[5],
		int map_assumed, le_follow_params_t *p, le_recover_quality_t *q);

int le_recover_solve(const le_recover_t *r, uint16_t interval,
		le_follow_params_t *p, le_recover_quality_t *q) {
	static const uint8_t full[5] = { 0xff, 0xff, 0xff, 0xff, 0x1f };
	return solve(r, interval, full, 1, p, q);
}

int le_recover_solve_map(const le_recover_t *r, uint16_t interval,
		const uint8_t chm[5], le_follow_params_t *p, le_recover_quality_t *q) {
	return solve(r, interval, chm, 0, p, q);
}

/*
 * map_assumed: the map is a guess (full). CSA#1 remapped events recur
 * with the same 37 event period as direct ones, so with an unknown
 * partial map a wrong phase can fit as well as the right one: only
 * accept CSA#1 if nearly every event is explained.
 */
static int solve(const le_recover_t *r, uint16_t interval, const uint8_t chm[5],
		int map_assumed, le_follow_params_t *p, le_recover_quality_t *q) {
	chmap_t map;
	uint32_t *offsets;
	uint16_t candidates[2];
	unsigned num_candidates = 0, ci, n = r->num_events;
	int solved = 0;
	unsigned anchor_event = 0;
	double best_fraction = 0;
	le_recover_quality_t quality = { 0, 0, n, 0 };

	if (n < 4 || interval < 6)
		return 0;
	chmap_parse(chm, &map);
	if (map.num_used < 2)
		return 0;

	// with CSA#1 and a full map, a channel recurs every 37 events, so a
	// single channel suggests 37x the real interval
	candidates[num_candidates++] = interval;
	if (interval % 37 == 0 && interval / 37 >= 6)
		candidates[num_candidates++] = interval / 37;

	offsets = malloc(sizeof(uint32_t) * n);
	if (offsets == NULL)
		return 0;

	for (ci = 0; ci < num_candidates; ++ci) {
		uint16_t iv = candidates[ci];
		search_t s2, s1;

		if (!event_offsets(r, iv, offsets))
			continue;
		// the last event on the grid, and the number of good events
		unsigned last = n - 1, good = 0, k;
		while (last > 0 && (offsets[last] == OFF_GRID || !certain(&r->events[last])))
			--last;
		if (offsets[last] == OFF_GRID)
			continue;
		for (k = 0; k < n; ++k)
			good += offsets[k] != OFF_GRID;

		s2 = search_csa2(r, offsets, &map);
		if (acceptable(&s2, good) && (double)s2.matched / good > best_fraction) {
			uint32_t counter = (s2.a + offsets[last]) & 0xffff;
			best_fraction = (double)s2.matched / good;
			anchor_event = last;
			memset(p, 0, sizeof(*p));
			p->csa = 2;
			p->counter = counter;
			p->interval = iv;
			quality.csa = 2;
			quality.matched = s2.matched;
			quality.runner_up = s2.runner_up > 0 ? s2.runner_up : 0;
			solved = 1;
		}

		// CSA#1 needs at least two channels to tell the hop increment
		if (le_recover_channels_seen(r) >= 2) {
			s1 = search_csa1(r, offsets, &map);
			if (acceptable(&s1, good) && (!map_assumed || s1.matched * 10 >= good * 9) &&
					(double)s1.matched / good > best_fraction) {
				best_fraction = (double)s1.matched / good;
				anchor_event = last;
				memset(p, 0, sizeof(*p));
				p->csa = 1;
				p->hop = s1.a;
				p->unmapped = (s1.b + (uint64_t)offsets[last] * s1.a) % 37;
				// only the counter mod 37 is known; instants fix the rest
				p->counter = offsets[last];
				p->flags |= LE_FOLLOW_COUNTER_UNKNOWN;
				p->interval = iv;
				quality.csa = 1;
				quality.matched = s1.matched;
				quality.runner_up = s1.runner_up > 0 ? s1.runner_up : 0;
				solved = 1;
			}
		}
	}

	free(offsets);

	if (solved) {
		p->access_address = r->access_address;
		uint32_t crc_init = 0;
		le_recover_get_crc(r, &crc_init);
		p->crc_init = crc_init;
		p->anchor = r->events[anchor_event].anchor;
		p->timeout = 0;
		memcpy(p->chm, chm, 5);
		p->chm[4] &= 0x1f;
		// an assumed full map is refined by the firmware from misses
		if (map.num_used == 37)
			p->flags |= LE_FOLLOW_MAP_LEARN;
		p->sca = 0; // assume the worst clock: widest windows
	}
	if (q)
		*q = quality;
	return solved;
}
