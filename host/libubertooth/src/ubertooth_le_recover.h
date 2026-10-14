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

/*
 * Recover the parameters of an established BLE connection from packets
 * observed on one or more fixed data channels (UBERTOOTH_LE_MONITOR),
 * so the firmware can be told to follow it (UBERTOOTH_LE_FOLLOW_PARAMS).
 *
 * Works for both channel selection algorithms:
 *  - CRCInit: reversed from the CRC of observed PDUs (majority vote)
 *  - interval: largest multiple of 1.25 ms that divides every gap
 *    between consecutive observed connection events
 *  - CSA#2: brute force search of the 16 bit event counter that maps
 *    the observed events to the observed channels
 *  - CSA#1: search of the hop increment and unmapped channel; needs
 *    events seen on at least two channels
 *
 * The channel map is assumed full; the firmware learns unused channels
 * from misses (LE_FOLLOW_MAP_LEARN). Times are device ticks (100 ns).
 */

#ifndef __UBERTOOTH_LE_RECOVER_H__
#define __UBERTOOTH_LE_RECOVER_H__

#include <stdint.h>
#include "ubertooth_interface.h"

#define LE_RECOVER_MAX_EVENTS 256
#define LE_RECOVER_CRC_SLOTS  8

typedef struct {
	uint32_t anchor;   // timestamp of the first packet of the event
	uint8_t  channel;  // data channel index
	uint8_t  packets;
} le_recover_event_t;

typedef struct {
	uint32_t access_address;

	le_recover_event_t events[LE_RECOVER_MAX_EVENTS];
	unsigned num_events;

	struct { uint32_t crc_init; unsigned votes; } crc[LE_RECOVER_CRC_SLOTS];
	unsigned crc_packets;
} le_recover_t;

typedef struct {
	int      csa;          // 1 or 2, 0 if unsolved
	unsigned matched;      // events explained by the solution
	unsigned considered;   // events used
	unsigned runner_up;    // matches of the second best candidate
} le_recover_quality_t;

void le_recover_init(le_recover_t *r, uint32_t access_address);

/*
 * Add a packet with the monitored access address. pdu is the dewhitened
 * PDU: header, payload and the 3 CRC bytes; len includes the CRC.
 */
void le_recover_packet(le_recover_t *r, uint32_t timestamp, uint8_t channel_idx,
		const uint8_t *pdu, unsigned len);

/* CRCInit from the PDU and its CRC (both on-air byte order) */
uint32_t le_recover_crc_init(const uint8_t *pdu, unsigned len_without_crc,
		const uint8_t crc[3]);

/* best CRCInit so far; returns the number of votes (0 if none) */
unsigned le_recover_get_crc(const le_recover_t *r, uint32_t *crc_init);

/*
 * Connection interval in 1.25 ms units, 0 if not yet determined.
 * min_events: minimum number of events on a single channel to trust it.
 */
uint16_t le_recover_interval(const le_recover_t *r, unsigned min_events);

/*
 * Solve for follow parameters. Tries CSA#2, then CSA#1. Returns 1 and
 * fills params on success. q (optional) reports the solution quality.
 */
int le_recover_solve(const le_recover_t *r, uint16_t interval,
		le_follow_params_t *params, le_recover_quality_t *q);

/* as le_recover_solve, with a known (e.g. surveyed) channel map */
int le_recover_solve_map(const le_recover_t *r, uint16_t interval,
		const uint8_t chm[5], le_follow_params_t *params, le_recover_quality_t *q);

/* number of distinct channels with observed events */
unsigned le_recover_channels_seen(const le_recover_t *r);

/* CSA#2 unmapped-channel PRN, exposed for tests */
uint16_t le_recover_csa2_prn_e(uint16_t counter, uint16_t channel_id);

#endif /* __UBERTOOTH_LE_RECOVER_H__ */
