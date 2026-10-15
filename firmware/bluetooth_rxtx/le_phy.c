/*
 * Copyright 2017 Mike Ryan
 *
 * This file is part of Project Ubertooth and is released under the
 * terms of the GPL. Refer to COPYING for more information.
 */

#include <stdlib.h>
#include <string.h>

#include "ubertooth.h"
#include "ubertooth_clock.h"
#include "ubertooth_dma.h"
#include "ubertooth_usb.h"
#include "bluetooth_le.h"
#include "le_phy.h"
#include "queue.h"

// current time, from timer1
#define NOW T1TC
#define USEC(X) ((X)*10)
#define MSEC(X) ((X)*10000)
#define  SEC(X) ((X)*10000000)
// LE 1M: 8 us preamble + 32 us AA + 8 us per PDU/CRC byte
#define PACKET_DURATION(X) (USEC(40 + (X)->size * 8))

#define ADVERTISING_AA (0x8e89bed6)
#define ADVERTISING_CRC_INIT_REV (0xaaaaaa) // 0x555555 bit reversed

///////////////////////
// time constants

// time for the radio to warmup + some timing slack
#define RX_WARMUP_TIME USEC(300)

// max inter-frame space between packets in a connection event
#define IFS_TIMEOUT USEC(300)

// longest LE 1M packet: 1 + 4 + 2 + 255 + 3 bytes
#define MAX_PACKET_TIME USEC(2120)

// a slave transmits no earlier than anchor + 80 + 150 us, so a single
// packet this close to the expected anchor must be the master's
#define MASTER_DETECT_MAX USEC(150)

// our own clock accuracy, added to the master's SCA for window widening
#define OUR_PPM 50

// events with no packets, with CSA chosen by guess, before trying the
// other channel selection algorithm
#define CSA_AUTO_EVENTS 6

// consecutive misses on a directly mapped channel before map learning
// removes it from the channel map
#define MAP_LEARN_MISSES 6

// A connection is lost if not established within 6 intervals (Core Spec
// Vol 6 Part B 4.5.2); allowing both channel selection algorithms a try,
// a connection never heard by then was a failed or forged CONNECT_IND
#define NEVER_SYNCED_EVENTS (2 * CSA_AUTO_EVENTS + 4)

// halfway between the latest start of the central's packet and the
// earliest start of the peripheral's (used while the widening is smaller)
#define SLAVE_FIRST_MARGIN USEC(115)

// LL_TERMINATE_IND (or an encrypted PDU of its length) ends the link only
// when silence follows: TERMINATE_DEAF_EVENTS events without packets.
// Traffic for TERMINATE_WAIT_EVENTS events means it was something else,
// or forged.
#define TERMINATE_DEAF_EVENTS 3

// Events after an encrypted PDU that looks like LL_PHY_UPDATE_IND within
// which silence means a PHY change: the central gives the peripheral at
// least 6 events it listens to before the instant (latency + 1 apart),
// with some to spare
#define ENC_PHY_SPARE_EVENTS 16
#define TERMINATE_WAIT_EVENTS 8

// transmitWindowDelay (Core Spec Vol 6 Part B 4.5.3)
#define TX_WINDOW_DELAY_LEGACY USEC(1250)
#define TX_WINDOW_DELAY_AUX    USEC(2500)

#define ADV_HOP_DWELL_DEFAULT MSEC(100)
// after an advertising PDU, give a CONNECT_IND or SCAN_REQ (which
// would start T_IFS later) a chance before hopping
#define HOP_ON_ADV_DELAY USEC(250)

//////////////////////
// global state

extern le_state_t le; // FIXME - refactor this struct
volatile uint16_t rf_channel;
uint8_t le_dma_dest[2];

extern volatile uint8_t mode;
extern volatile uint8_t requested_mode;
extern volatile uint16_t le_adv_channel;
extern volatile int cancel_follow;


////////////////////
// buffers

// packet buffers live in a pool. a minimum of one buffer is always
// being used, either waiting to receive or actively receiving a packet
// (current_rxbuf). once a packet is received, it is placed into the
// packet queue. the main loop pulls packets from this queue and
// processes them, and then returns the buffers back to the pool by
// calling buffer_release()

#define LE_BUFFER_POOL_SIZE 6
typedef struct _le_rx_t {
	uint8_t data[2 + 255 + 3];  // header + PDU + CRC
	unsigned size;              // total data length (known after header rx)
	unsigned pos;               // current input byte offset
	uint32_t timestamp;         // start of preamble
	unsigned channel;           // physical channel
	uint32_t access_address;    // access address the radio was tuned to
	volatile int available;     // 1 if available, 0 in use
	int8_t rssi_min, rssi_max;  // min and max RSSI observed values
	int rssi_sum;               // running sum of RSSI samples
	uint8_t rssi_count;         // number of RSSI samples
	uint8_t header0;            // dewhitened first header byte
	uint8_t is_conn;            // data PDU of the followed connection
	uint8_t event_pos;          // 1-based position within the connection event
	uint8_t aux;                // received on a secondary adv channel
	uint16_t counter;           // connection event counter
	int8_t crc_ok;              // -1 unchecked, 0 bad, 1 good
} le_rx_t;

// sample RSSI only during the first bytes: bit-banged SPI is slow
#define RSSI_SAMPLES 4

// pool of all buffers
static le_rx_t le_buffer_pool[LE_BUFFER_POOL_SIZE] AHB_BSS;

// buffer waiting for or actively receiving packet
static le_rx_t *volatile current_rxbuf = NULL;

// received packets, waiting to be processed
queue_t packet_queue;

// packets dropped because the USB queue or packet queue was full
static volatile uint32_t drops = 0;

/////////////////////
// radio state

static volatile uint32_t rx_aa = ADVERTISING_AA; // AA the radio is tuned to
static volatile uint8_t rx_channel_idx = 37;     // channel index the radio is tuned to
static volatile int rx_busy = 0;                 // a packet is being received
static volatile int radio_conn = 0;              // tuned for a connection event

/////////////////////
// connections

// this system is architected so that following multiple connections may
// be possible in the future. all connection state lives in an le_conn_t
// struct. at present only one such structure exists.

typedef struct _le_conn_t {
	uint32_t access_address;
	uint32_t crc_init;
	uint32_t crc_init_reversed;
	uint16_t channel_id;     // CSA#2 channel identifier
	uint8_t  csa;            // channel selection algorithm, 1 or 2
	uint8_t  csa_auto;       // CSA was a guess: try the other one if deaf
	uint8_t  hop_increment;
	uint8_t  last_unmapped;  // CSA#1 lastUnmappedChannel
	uint8_t  channel_idx;    // data channel of the current event
	uint8_t  direct;         // current channel is the event's unmapped channel
	uint8_t  sca;
	uint8_t  flags;          // le_state_conn_t flags

	// raw parameters, for reporting
	uint16_t interval_units;
	uint16_t timeout_units;
	uint16_t latency;
	uint8_t  win_size_units;
	uint16_t win_offset_units;
	uint8_t  chm[5];

	uint32_t conn_interval;       // in units of 100 ns
	uint32_t supervision_timeout; // in units of 100 ns
	uint32_t sca_ppm;

	// transmit window, used while the anchor is not known
	uint32_t win_start;
	uint32_t win_size;

	le_channel_remapping_t remapping;

	uint32_t last_anchor;      // anchor of the last event (observed or estimated)
	uint32_t last_obs_anchor;  // last anchor actually observed
	uint32_t expected_anchor;  // anchor of the current event
	uint32_t widening;         // window widening of the current event
	int      anchor_set;
	int      ever_synced;
	unsigned deaf_events;      // events without packets since start/switch
	uint32_t last_packet_ts;   // used to check supervision timeout

	uint16_t conn_event_counter; // event being scheduled/opened

	int      conn_update_pending;
	uint32_t upd_interval;
	uint32_t upd_timeout;
	uint32_t upd_win_size;
	uint32_t upd_win_offset;
	uint16_t upd_interval_units, upd_timeout_units, upd_latency;
	uint16_t conn_update_instant;

	int      channel_map_update_pending;
	uint16_t channel_map_update_instant;
	uint8_t  pending_chm[5];

	int      phy_update_pending;
	uint16_t phy_update_instant;
	uint8_t  phy_m2s, phy_s2m;

	int      encrypted;
	int      enc_phy_hint;     // encrypted PDU that looks like LL_PHY_UPDATE_IND seen
	uint16_t enc_phy_counter;  // ... in this event
	int      map_learn;
	int      counter_uncertain;
	int      terminate_events; // events since LL_TERMINATE_IND (0: none pending)
	int      monitor;          // fixed channel monitor, no connection
	uint8_t  miss_count[37];
} le_conn_t;
le_conn_t conn = { 0, };

static uint16_t enc_phy_window(void) {
	return 6 * (conn.latency + 1) + ENC_PHY_SPARE_EVENTS;
}

// connection event state
typedef enum { EV_IDLE, EV_WAIT_OPEN, EV_OPEN } ev_state_t;
typedef struct _le_conn_event_t {
	uint32_t anchor;       // timestamp of the first packet
	unsigned num_packets;
	uint8_t  md;           // more data bit seen in the current packet pair
	uint8_t  slave_only;   // the central's packet was missed: anchor not observed
	ev_state_t state;
	uint32_t timeout;      // event closes if no packet starts by then
} le_conn_event_t;
static volatile le_conn_event_t conn_event;

// advertising side timers, multiplexed on MR0 when not in a connection
typedef enum { ADV_TMR_NONE, ADV_TMR_HOP, ADV_TMR_AUX_OPEN, ADV_TMR_AUX_CLOSE } adv_timer_t;
static volatile adv_timer_t adv_timer = ADV_TMR_NONE;
static volatile uint16_t adv_flags = LE_ADV_FOLLOW_AUX;
static volatile uint32_t adv_dwell = ADV_HOP_DWELL_DEFAULT;
static volatile uint8_t adv_primary_idx = 37;
static volatile uint8_t aux_chan;
static volatile uint32_t aux_close;
static volatile int aux_active = 0;
// Following AuxPtrs must not stop the primary channel hopping: the hop
// deadline (adv_hop_at) survives an aux excursion, an aux that would open
// after it is not followed, and a chain is followed for AUX_CHAIN_MAX hops
// (AuxPtrs are easy to send).
#define AUX_CHAIN_MAX 8
static volatile uint32_t adv_hop_at;
static volatile uint8_t aux_chain = 0;

// pending requests from the host, applied by the main loop
static volatile int monitor_request = 0;
static volatile uint16_t monitor_channel;
static volatile uint32_t monitor_aa;
static volatile int follow_request = 0;
static le_follow_params_t follow_params;

// ChSel of recent ADV_IND / ADV_DIRECT_IND, by AdvA
#define CHSEL_CACHE_SIZE 8
static struct { uint8_t addr[6]; uint8_t chsel; uint8_t valid; } chsel_cache[CHSEL_CACHE_SIZE];
static unsigned chsel_next = 0;

/////////////////////
// state messages to the host

// queued from ISRs and the main loop, sent by the main loop
#define STATE_QUEUE_SIZE 8
typedef struct { uint8_t code, len; uint8_t data[DMA_SIZE - 1]; } le_state_msg_t;
static le_state_msg_t state_queue[STATE_QUEUE_SIZE];
static volatile unsigned state_head = 0, state_tail = 0;

static volatile int host_ext = 0;

void le_phy_set_host_ext(int on) {
	host_ext = on;
}

static void state_push(uint8_t code, const void *data, unsigned len) {
	if (!host_ext)
		return;
	uint32_t s = irq_save();
	unsigned next = (state_head + 1) % STATE_QUEUE_SIZE;
	if (next != state_tail) {
		if (len > sizeof(state_queue[0].data))
			len = sizeof(state_queue[0].data);
		state_queue[state_head].code = code;
		state_queue[state_head].len = len;
		memcpy(state_queue[state_head].data, data, len);
		state_head = next;
	}
	irq_restore(s);
}

static void state_push_counter(uint8_t code, uint8_t a, uint16_t counter) {
	uint8_t d[3] = { a, counter & 0xff, counter >> 8 };
	state_push(code, d, 3);
}

//////////////////////
// code

// pre-declarations
static void timer1_set_match(uint32_t match);
static void timer1_clear_match(void);
static void timer1_wait_fs_lock(void);
static void timer1_wait_buffer(void);
static void le_dma_init(void);
static void le_cc2400_init_rf(void);
static void le_cc2400_strobe_rx(void);
static void schedule_next_event(void);
static void adv_resume(void);

static inline int conn_active(void) {
	return conn.access_address != ADVERTISING_AA && !conn.monitor;
}

// resets the state of all available buffers
static void buffers_init(void) {
	int i;

	for (i = 0; i < LE_BUFFER_POOL_SIZE; ++i)
		le_buffer_pool[i].available = 1;
}

// clear a buffer for new data
static void buffer_clear(le_rx_t *buf) {
	buf->pos = 0;
	buf->size = 0;
	buf->rssi_min = INT8_MAX;
	buf->rssi_max = INT8_MIN;
	buf->rssi_sum = 0;
	buf->rssi_count = 0;
	buf->is_conn = 0;
	buf->event_pos = 0;
	buf->aux = 0;
	buf->counter = 0;
	buf->crc_ok = -1;
}

// get a packet buffer (ISR context)
// returns a pointer to a buffer if available
// returns NULL otherwise
static le_rx_t *buffer_get(void) {
	int i;

	for (i = 0; i < LE_BUFFER_POOL_SIZE; ++i) {
		if (le_buffer_pool[i].available) {
			le_buffer_pool[i].available = 0;
			buffer_clear(&le_buffer_pool[i]);
			return &le_buffer_pool[i];
		}
	}

	return NULL;
}

// release a buffer back to the pool
static void buffer_release(le_rx_t *buffer) {
	buffer->available = 1;
}

static void reset_conn(void) {
	memset(&conn, 0, sizeof(conn));
	conn.access_address = ADVERTISING_AA;
}

// clear a connection event
static void reset_conn_event(void) {
	conn_event.num_packets = 0;
	conn_event.md = 0;
	conn_event.slave_only = 0;
	conn_event.state = EV_IDLE;
}

//////////////////////
// radio control

// stop RX, DMA and pending FS lock / buffer waits
static void radio_stop(void) {
	cc2400_strobe(SRFOFF);
	ICER0 = ICER0_ICE_DMA;
	DMACC0Config = 0;
	DMACIntTCClear = (1 << 0);
	DMACIntErrClr = (1 << 0);
	DIO_SSP_DMACR &= ~SSPDMACR_RXDMAE;
	while (SSP1SR & SSPSR_RNE) {
		volatile uint8_t tmp = (uint8_t)DIO_SSP_DR;
		(void)tmp;
	}
	T1MCR &= ~(TMCR_MR2I | TMCR_MR3I);
	rx_busy = 0;
}

// tune to a channel index with the given access address and start RX
// once the synthesizer locks (ISR context or with interrupts disabled)
static void tune_radio(uint8_t channel_idx, uint32_t aa) {
	radio_stop();

	if (current_rxbuf == NULL)
		current_rxbuf = buffer_get();
	if (current_rxbuf != NULL)
		buffer_clear(current_rxbuf);

	rx_aa = aa;
	rx_channel_idx = channel_idx;
	rf_channel = btle_channel_index_to_phys(channel_idx);
	radio_conn = 0;
	le_cc2400_init_rf();

	if (current_rxbuf == NULL)
		timer1_wait_buffer();
	else
		timer1_wait_fs_lock();
}

static void start_rx(void) {
	le_dma_init();
	dio_ssp_start();
	ISER0 = ISER0_ISE_DMA;
	le_cc2400_strobe_rx();
}

//////////////////////
// connection events

static uint32_t widening_for(uint32_t elapsed) {
	// (SCA + ours) ppm of the time since the last observed anchor;
	// elapsed / 1000 keeps the product in 32 bits for up to 429 s
	uint32_t w = USEC(16) + (elapsed / 1000) * (conn.sca_ppm + OUR_PPM) / 1000;
	if (w > conn.conn_interval / 4)
		w = conn.conn_interval / 4;
	return w;
}

static void conn_lost(uint8_t reason, uint8_t m2s, uint8_t s2m) {
	uint8_t d[5] = { reason, conn.conn_event_counter & 0xff,
		conn.conn_event_counter >> 8, m2s, s2m };
	state_push(LE_STATE_CONN_LOST, d, reason == LE_LOST_PHY ? 5 : 3);
	reset_conn();
	reset_conn_event();
	timer1_clear_match();
	adv_resume();
}

static void map_learn_remove(uint8_t channel) {
	uint8_t chm[5];
	memcpy(chm, conn.chm, 5);
	chm[channel / 8] &= ~(1 << (channel % 8));
	le_channel_remapping_t r;
	le_parse_channel_map(chm, &r);
	if (r.total_channels < 2)
		return;
	memcpy(conn.chm, chm, 5);
	conn.remapping = r;
	memset(conn.miss_count, 0, sizeof(conn.miss_count));
	state_push(LE_STATE_MAP_LEARN, chm, 5);
}

// open a connection event: choose the channel and tune (ISR context)
static void open_event(void) {
	uint16_t counter = conn.conn_event_counter;
	uint8_t unmapped, channel;

	// CSA#1 state advances every event, so either algorithm can be
	// switched to at any time
	uint8_t unmapped1 = (conn.last_unmapped + conn.hop_increment) % 37;
	conn.last_unmapped = unmapped1;

	if (conn.csa == 2) {
		uint16_t prn_e = le_csa2_prn_e(counter, conn.channel_id);
		unmapped = prn_e % 37;
		if (conn.remapping.channel_in_use[unmapped])
			channel = unmapped;
		else
			channel = conn.remapping.remapping_index[
				(conn.remapping.total_channels * prn_e) >> 16];
	} else {
		unmapped = unmapped1;
		channel = le_map_channel(unmapped, &conn.remapping);
	}

	conn.direct = (channel == unmapped);
	conn.channel_idx = channel;
	conn_event.state = EV_OPEN;
	conn_event.num_packets = 0;
	conn_event.md = 0;

	tune_radio(channel, conn.access_address);
	radio_conn = 1;
	timer1_set_match(conn_event.timeout);
}

// finish a connection event (ISR context)
//
// 1) update the anchor point: two or more packets, or one packet close
//    to the expected anchor, give an observed anchor (the master always
//    transmits first). otherwise the anchor is estimated.
// 2) learn the channel map / auto-detect CSA from misses
// 3) increment connection event counter
// 4) check supervision timeout, cancel and termination
// 5) schedule the next event
static void finish_conn_event(void) {
	unsigned n = conn_event.num_packets;
	uint32_t obs = conn_event.anchor;
	uint8_t ch = conn.channel_idx;

	radio_stop();
	radio_conn = 0;

	if (n > 0) {
		conn.last_packet_ts = NOW;
		conn.deaf_events = 0;
		if (!conn.anchor_set) {
			conn.last_anchor = obs;
			conn.last_obs_anchor = obs;
			conn.anchor_set = 1;
			if (!conn.ever_synced) {
				uint8_t d[6] = { obs, obs >> 8, obs >> 16, obs >> 24,
					conn.conn_event_counter & 0xff, conn.conn_event_counter >> 8 };
				state_push(LE_STATE_ANCHOR, d, 6);
			}
			conn.ever_synced = 1;
		} else {
			int32_t delta = (int32_t)(obs - conn.expected_anchor);
			int32_t tol = conn.widening + USEC(16);
			if (tol > MASTER_DETECT_MAX)
				tol = MASTER_DETECT_MAX;
			if (conn_event.slave_only) {
				conn.last_anchor = conn.expected_anchor;
			} else if (n >= 2 || (delta > -tol && delta < tol)) {
				conn.last_anchor = obs;
				conn.last_obs_anchor = obs;
			} else {
				conn.last_anchor = conn.expected_anchor;
			}
		}
		conn.csa_auto = 0;
		conn.miss_count[ch] = 0;
	} else {
		if (conn.anchor_set)
			conn.last_anchor = conn.expected_anchor;
		else
			conn.win_start += conn.conn_interval;

		if (conn.anchor_set && conn.map_learn && conn.direct &&
				++conn.miss_count[ch] >= MAP_LEARN_MISSES)
			map_learn_remove(ch);

		// an encrypted LL_PHY_UPDATE_IND followed by silence from its
		// instant: the link moved to a PHY we can't receive. The instant
		// is at least 6 events later; silence that starts after any likely
		// instant is a fade, and the hint expires
		if (conn.enc_phy_hint) {
			uint16_t since = conn.conn_event_counter - conn.enc_phy_counter;
			if ((uint16_t)(since - conn.deaf_events) > enc_phy_window()) {
				conn.enc_phy_hint = 0;
			} else if (conn.deaf_events + 1 >= 4 && since >= 6) {
				conn_lost(LE_LOST_PHY, 0, 0);
				return;
			}
		}
		++conn.deaf_events;

		if (conn.csa_auto && !conn.ever_synced &&
				conn.deaf_events >= CSA_AUTO_EVENTS) {
			conn.deaf_events = 0;
			conn.csa = conn.csa == 2 ? 1 : 2;
			state_push_counter(LE_STATE_CSA_SWITCH, conn.csa, conn.conn_event_counter);
		}
	}

	reset_conn_event();
	++conn.conn_event_counter;

	if (conn.terminate_events > 0) {
		if (conn.deaf_events >= TERMINATE_DEAF_EVENTS) {
			conn_lost(LE_LOST_TERMINATED, 0, 0);
			return;
		}
		if (++conn.terminate_events > TERMINATE_WAIT_EVENTS)
			conn.terminate_events = 0;
	}
	if (!conn.ever_synced && conn.conn_event_counter >= NEVER_SYNCED_EVENTS) {
		conn_lost(LE_LOST_NEVER_SYNCED, 0, 0);
	} else if (NOW - conn.last_packet_ts > conn.supervision_timeout) {
		conn_lost(conn.ever_synced ? LE_LOST_SUPERVISION : LE_LOST_NEVER_SYNCED, 0, 0);
	} else if (cancel_follow) {
		cancel_follow = 0;
		conn_lost(LE_LOST_CANCELED, 0, 0);
	} else {
		schedule_next_event();
	}
}

// has an instant been reached (or passed, if we missed the exact event)?
static int instant_reached(uint16_t instant, uint16_t counter) {
	return (uint16_t)(counter - instant) < 32768;
}

// apply procedures whose instant is the event about to be scheduled,
// then set the timer for it (ISR context, or interrupts disabled)
static void schedule_next_event(void) {
	uint16_t c = conn.conn_event_counter;
	uint32_t open;

	if (conn.channel_map_update_pending &&
			instant_reached(conn.channel_map_update_instant, c)) {
		memcpy(conn.chm, conn.pending_chm, 5);
		le_parse_channel_map(conn.chm, &conn.remapping);
		conn.channel_map_update_pending = 0;
	}

	if (conn.phy_update_pending && instant_reached(conn.phy_update_instant, c)) {
		conn.phy_update_pending = 0;
		// 0 means unchanged; bit 0 is LE 1M. we can only receive 1M, so
		// the connection is lost unless at least one direction stays 1M.
		int m2s_1m = conn.phy_m2s == 0 || conn.phy_m2s == 1;
		int s2m_1m = conn.phy_s2m == 0 || conn.phy_s2m == 1;
		if (!m2s_1m && !s2m_1m) {
			conn_lost(LE_LOST_PHY, conn.phy_m2s, conn.phy_s2m);
			return;
		}
	}

	if (conn.conn_update_pending && instant_reached(conn.conn_update_instant, c)) {
		// the transmit window starts transmitWindowOffset after the
		// anchor the event would have had with the old interval
		conn.win_start = conn.last_anchor + conn.conn_interval + conn.upd_win_offset;
		conn.win_size = conn.upd_win_size;
		conn.conn_interval = conn.upd_interval;
		conn.supervision_timeout = conn.upd_timeout;
		conn.interval_units = conn.upd_interval_units;
		conn.timeout_units = conn.upd_timeout_units;
		conn.latency = conn.upd_latency;
		conn.anchor_set = 0;
		conn.conn_update_pending = 0;
	}

	if (conn.anchor_set) {
		conn.expected_anchor = conn.last_anchor + conn.conn_interval;
		conn.widening = widening_for(conn.expected_anchor - conn.last_obs_anchor);
		open = conn.expected_anchor - conn.widening - RX_WARMUP_TIME;
		conn_event.timeout = conn.expected_anchor + conn.widening +
			MAX_PACKET_TIME + USEC(200);
	} else {
		conn.expected_anchor = conn.win_start;
		conn.widening = USEC(32);
		open = conn.win_start - conn.widening - RX_WARMUP_TIME;
		conn_event.timeout = conn.win_start + conn.win_size + conn.widening +
			MAX_PACKET_TIME + USEC(200);
	}

	conn_event.state = EV_WAIT_OPEN;
	timer1_set_match(open);
}

//////////////////////
// DMA

// end of a packet (DMA ISR context)
static void end_of_packet(le_rx_t *buf) {
	// stop the CC2400 before flushing SSP
	cc2400_strobe(SFSON);

	// stop DMA on this channel and flush SSP
	DMACC0Config = 0;
	DMACIntTCClear = (1 << 0); // if we don't clear a second time, data is corrupt
	DIO_SSP_DMACR &= ~SSPDMACR_RXDMAE;
	while (SSP1SR & SSPSR_RNE) {
		volatile uint8_t tmp = (uint8_t)DIO_SSP_DR;
		(void)tmp;
	}
	ICER0 = ICER0_ICE_DMA;
	rx_busy = 0;

	int close_event = 0;
	if (radio_conn && conn_event.state == EV_OPEN) {
		++conn_event.num_packets;
		// the first packet heard is the peripheral's if the central's was
		// missed: the central's starts within the widening of the anchor,
		// the peripheral's at least 230 us (an empty PDU and T_IFS) later
		if (conn_event.num_packets == 1 && conn.anchor_set &&
				conn.widening < SLAVE_FIRST_MARGIN &&
				(int32_t)(buf->timestamp - conn.expected_anchor) > SLAVE_FIRST_MARGIN) {
			conn_event.num_packets = 2;
			conn_event.anchor = conn.expected_anchor;
			conn_event.slave_only = 1;
		}
		buf->is_conn = 1;
		buf->event_pos = conn_event.num_packets;
		buf->counter = conn.conn_event_counter;
		if (conn_event.num_packets == 1)
			conn_event.anchor = buf->timestamp;
		conn_event.md |= buf->header0 & 0x10;
		if ((conn_event.num_packets & 1) == 0) {
			if (!conn_event.md || conn_event.num_packets >= 32)
				close_event = 1;
			conn_event.md = 0;
		}
	} else if (rx_aa == ADVERTISING_AA && rx_channel_idx < 37) {
		buf->aux = 1;
	}

	if (!queue_insert(&packet_queue, buf)) {
		buffer_release(buf);
		++drops;
	}

	current_rxbuf = buffer_get();

	if (close_event) {
		finish_conn_event();
		return;
	}

	if (radio_conn) {
		// wait for the next packet of this event
		timer1_set_match(NOW + IFS_TIMEOUT);
	} else if (!conn_active() && !conn.monitor && !aux_active &&
			(adv_flags & LE_ADV_HOP_ON_ADV) && rx_channel_idx >= 37) {
		switch (buf->header0 & 0xf) {
			case 0x0: case 0x1: case 0x2: case 0x6: case 0x7:
				adv_timer = ADV_TMR_HOP;
				adv_hop_at = NOW + HOP_ON_ADV_DELAY;
				timer1_set_match(adv_hop_at);
				break;
		}
	}

	if (current_rxbuf == NULL)
		timer1_wait_buffer();
	else
		timer1_wait_fs_lock();
}

// DMA handler
// called once per byte. handles all incoming data, but only minimally
// processes received data. at the end of a packet, it enqueues the
// received packet, fetches a new buffer, and restarts RX.
void le_DMA_IRQHandler(void) {
	unsigned pos;
	uint32_t timestamp = NOW; // sampled early for most accurate measurement
	le_rx_t *buf = current_rxbuf;

	if (!(DMACIntStat & (1 << 0)))
		return;

	if (DMACIntErrStat & (1 << 0))
		DMACIntErrClr = (1 << 0);

	if (!(DMACIntTCStat & (1 << 0)))
		return;
	DMACIntTCClear = (1 << 0);

	if (buf == NULL) {
		// no buffer: should not happen, stop until one is available
		radio_stop();
		timer1_wait_buffer();
		return;
	}

	pos = buf->pos;
	if (pos < sizeof(buf->data))
		buf->data[pos] = le_dma_dest[pos & 1]; // dirty hack
	pos += 1;
	buf->pos = pos;

	if (pos <= RSSI_SAMPLES) {
		int8_t rssi = (int8_t)(cc2400_get(RSSI) >> 8);
		buf->rssi_sum += rssi;
		++buf->rssi_count;
		if (rssi < buf->rssi_min) buf->rssi_min = rssi;
		if (rssi > buf->rssi_max) buf->rssi_max = rssi;
	}

	if (pos == 1) {
		// first DMA byte arrives after preamble + AA + 8 bits
		buf->timestamp = timestamp - USEC(8 + 32 + 8);
		buf->channel = rf_channel;
		buf->access_address = rx_aa;
		rx_busy = 1;

		// packet started: cancel the event / IFS timeout, a new timer
		// is set at the end of the packet
		if (radio_conn && conn_event.state == EV_OPEN)
			timer1_clear_match();
	}

	// get header and length
	if (pos == 2) {
		buf->header0 = le_dewhiten_byte(buf->channel, 0, buf->data[0]);
		uint8_t length = le_dewhiten_byte(buf->channel, 1, buf->data[1]);
		buf->size = length + 2 + 3; // two bytes for header and three for CRC
	}

	// finished packet - state transition
	if (pos > 2 && pos >= buf->size)
		end_of_packet(buf);
}

static void le_dma_init(void) {
	int i;

	// DMA linked list items
	typedef struct {
		uint32_t src;
		uint32_t dest;
		uint32_t next_lli;
		uint32_t control;
	} dma_lli;
	static dma_lli le_dma_lli[2];

	for (i = 0; i < 2; ++i) {
		le_dma_lli[i].src = (uint32_t)&(DIO_SSP_DR);
		le_dma_lli[i].dest = (uint32_t)&le_dma_dest[i];
		le_dma_lli[i].next_lli = (uint32_t)&le_dma_lli[1-i]; // two elements pointing back at each other
		le_dma_lli[i].control = 1 |
				(0 << 12) |        // source burst size = 1
				(0 << 15) |        // destination burst size = 1
				(0 << 18) |        // source width 8 bits
				(0 << 21) |        // destination width 8 bits
				DMACCxControl_I;   // terminal count interrupt enable
	}

	// clear stale interrupts from a previous channel
	DMACIntTCClear = (1 << 0);
	DMACIntErrClr = (1 << 0);

	// configure DMA channel 0
	DMACC0SrcAddr = le_dma_lli[0].src;
	DMACC0DestAddr = le_dma_lli[0].dest;
	DMACC0LLI = le_dma_lli[0].next_lli;
	DMACC0Control = le_dma_lli[0].control;
	DMACC0Config =
			DIO_SSP_SRC |
			(0x2 << 11) |     // peripheral to memory
			DMACCxConfig_IE | // allow error interrupts
			DMACCxConfig_ITC; // allow terminal count interrupts
}

// initalize USB, SSP, and DMA
static void le_sys_init(void) {
	usb_queue_init(); // USB FIFO FIXME replace with safer queue
	dio_ssp_init();   // init SSP and raise !CS (self-routed GPIO)
	le_dma_init();    // prepare DMA + interrupts
	dio_ssp_start();  // enable SSP + DMA
}

// program the radio for rf_channel / rx_aa and strobe FSON
static void le_cc2400_init_rf(void) {
	u16 grmdm, mdmctrl;
	uint32_t sync = rbit(rx_aa);

	mdmctrl = 0x0040; // 250 kHz frequency deviation
	grmdm = 0x44E1; // un-buffered mode, packet w/ sync word detection
	// 0 10 00 1 001 11 0 00 0 1
	//   |  |  | |   |  +--------> CRC off
	//   |  |  | |   +-----------> sync word: 32 MSB bits of SYNC_WORD
	//   |  |  | +---------------> 1 preamble byte of 01010101
	//   |  |  +-----------------> packet mode
	//   |  +--------------------> un-buffered mode
	//   +-----------------------> sync error bits: 2

	cc2400_set(MANAND,  0x7ffe);
	cc2400_set(LMTST,   0x2b22);

	cc2400_set(MDMTST0, 0x124b);
	// 1      2      4b
	// 00 0 1 0 0 10 01001011
	//    | | | | |  +---------> AFC_DELTA = ??
	//    | | | | +------------> AFC settling = 4 pairs (8 bit preamble)
	//    | | | +--------------> no AFC adjust on packet
	//    | | +----------------> do not invert data
	//    | +------------------> TX IF freq 1 0Hz
	//    +--------------------> PRNG off
	//
	// ref: CC2400 datasheet page 67
	// AFC settling explained page 41/42

	cc2400_set(GRMDM,   grmdm);

	cc2400_set(SYNCL,   sync & 0xffff);
	cc2400_set(SYNCH,   (sync >> 16) & 0xffff);

	cc2400_set(FSDIV,   rf_channel - 1); // 1 MHz IF
	cc2400_set(MDMCTRL, mdmctrl);

	// XOSC16M should always be stable, but leave this test anyway
	cc2400_wait_status(XOSC16M_STABLE, 1);

	cc2400_strobe(SFSON);
}

// strobe RX and enable PA
static void le_cc2400_strobe_rx(void) {
	cc2400_strobe(SRX);
#ifdef UBERTOOTH_ONE
	PAEN_SET;
	HGM_SET;
#endif
}

///////
// advertising channels

static void adv_tune_primary(void) {
	tune_radio(adv_primary_idx, ADVERTISING_AA);
}

// return to the primary advertising channel and restart the hop timer
static void adv_resume(void) {
	aux_active = 0;
	aux_chain = 0;
	adv_tune_primary();
	if (adv_flags & LE_ADV_HOP) {
		adv_timer = ADV_TMR_HOP;
		adv_hop_at = NOW + adv_dwell;
		timer1_set_match(adv_hop_at);
	} else {
		adv_timer = ADV_TMR_NONE;
		timer1_clear_match();
	}
}

// back from an aux channel: keep the primary channel's hop deadline
static void adv_return_from_aux(void) {
	aux_active = 0;
	aux_chain = 0;
	if (!(adv_flags & LE_ADV_HOP)) {
		adv_tune_primary();
		adv_timer = ADV_TMR_NONE;
		timer1_clear_match();
		return;
	}
	if ((int32_t)(adv_hop_at - NOW) <= 0) {
		adv_primary_idx = adv_primary_idx >= 39 ? 37 : adv_primary_idx + 1;
		adv_hop_at = NOW + adv_dwell;
	}
	adv_tune_primary();
	adv_timer = ADV_TMR_HOP;
	timer1_set_match(adv_hop_at);
}

static void adv_timer_fired(void) {
	switch (adv_timer) {
		case ADV_TMR_HOP:
			if (rx_busy) {
				timer1_set_match(NOW + USEC(400));
				break;
			}
			adv_primary_idx = adv_primary_idx >= 39 ? 37 : adv_primary_idx + 1;
			adv_tune_primary();
			if (adv_flags & LE_ADV_HOP) {
				adv_hop_at = NOW + adv_dwell;
				timer1_set_match(adv_hop_at);
			} else {
				adv_timer = ADV_TMR_NONE;
				timer1_clear_match();
			}
			break;

		case ADV_TMR_AUX_OPEN:
			tune_radio(aux_chan, ADVERTISING_AA);
			aux_active = 1;
			adv_timer = ADV_TMR_AUX_CLOSE;
			timer1_set_match(aux_close);
			break;

		case ADV_TMR_AUX_CLOSE:
			if (rx_busy) {
				timer1_set_match(NOW + USEC(300));
				break;
			}
			adv_return_from_aux();
			break;

		default:
			timer1_clear_match();
			break;
	}
}

///////
// timer stuff

static void timer1_start(void) {
	T1TCR = TCR_Counter_Reset;
	T1PR = 4; // 100 ns
	T1MCR = 0;
	T1IR = 0x3f;
	T1TCR = TCR_Counter_Enable;

	// set up interrupt handler
	ISER0 = ISER0_ISE_TIMER1;
}

static void timer1_stop(void) {
	T1TCR = TCR_Counter_Reset;
	T1MCR = 0;

	// clear interrupt handler
	ICER0 = ICER0_ICE_TIMER1;
}

// MR0 only interrupts when TC equals MR0 exactly, so a match in the
// past would not fire until T1TC wraps (429 s): never set one
static void timer1_set_match(uint32_t match) {
	uint32_t s = irq_save();
	if ((int32_t)(match - NOW) < (int32_t)USEC(2))
		match = NOW + USEC(2);
	T1MR0 = match;
	T1MCR |= TMCR_MR0I;
	irq_restore(s);
}

static void timer1_clear_match(void) {
	uint32_t s = irq_save();
	T1MCR &= ~TMCR_MR0I;
	irq_restore(s);
}

static void timer1_wait_fs_lock(void) {
	uint32_t s = irq_save();
	T1MR2 = NOW + USEC(3);
	T1MCR |= TMCR_MR2I;
	irq_restore(s);
}

static void timer1_wait_buffer(void) {
	uint32_t s = irq_save();
	T1MR3 = NOW + USEC(100);
	T1MCR |= TMCR_MR3I;
	irq_restore(s);
}

void TIMER1_IRQHandler(void) {
	// MR0: connection events, advertising hops and aux windows
	if (T1IR & TIR_MR0_Interrupt) {
		// ack the interrupt
		T1IR = TIR_MR0_Interrupt;

		if (conn_active()) {
			if (conn_event.state == EV_WAIT_OPEN)
				open_event();
			else if (conn_event.state == EV_OPEN)
				finish_conn_event();
			else
				timer1_clear_match();
		} else if (!conn.monitor) {
			adv_timer_fired();
		} else {
			timer1_clear_match();
		}
	}

	// LEDs
	if (T1IR & TIR_MR1_Interrupt) {
		T1IR = TIR_MR1_Interrupt;
		T1MCR &= ~TMCR_MR1I;

		TXLED_CLR;
		RXLED_CLR;
		USRLED_CLR;
	}

	// check FS_LOCK
	if (T1IR & TIR_MR2_Interrupt) {
		T1IR = TIR_MR2_Interrupt;

		if (current_rxbuf == NULL) {
			T1MCR &= ~TMCR_MR2I;
			timer1_wait_buffer();
		} else if (cc2400_status() & FS_LOCK) {
			// if FS is locked, strobe RX and clear interrupt
			T1MCR &= ~TMCR_MR2I;
			start_rx();
		} else {
			// if FS is not locked, check again in 3 us
			T1MR2 = NOW + USEC(3);
		}
	}

	// check if an rxbuf is available and transition to FS_LOCK/RX
	if (T1IR & TIR_MR3_Interrupt) {
		T1IR = TIR_MR3_Interrupt;

		if (current_rxbuf == NULL)
			current_rxbuf = buffer_get();
		if (current_rxbuf == NULL) {
			T1MR3 = NOW + USEC(100);
		} else {
			T1MCR &= ~TMCR_MR3I;
			if (cc2400_status() & FS_LOCK)
				start_rx();
			else
				timer1_wait_fs_lock();
		}
	}
}

static void blink(int tx, int rx, int usr) {
	uint32_t s;

	if (tx)
		TXLED_SET;
	if (rx)
		RXLED_SET;
	if (usr)
		USRLED_SET;

	// blink for 10 ms
	s = irq_save();
	T1MR1 = NOW + MSEC(10);
	T1MCR |= TMCR_MR1I;
	irq_restore(s);
}

//////////////////////
// USB

// byte `i` of the AA + PDU stream sent to the host
static inline uint8_t le_usb_byte(le_rx_t *packet, unsigned i) {
	return i < 4 ? (uint8_t)(packet->access_address >> (8 * i)) : packet->data[i - 4];
}

// enqueue a packet for USB, splitting packets longer than one USB
// packet into LE_PACKET + LE_PACKET_CONT chunks
static void usb_enqueue_le(le_rx_t *packet) {
	unsigned total, off, i, chunk;
	usb_pkt_rx *f;
	uint8_t flags = 0;

	if (packet->size > sizeof(packet->data))
		packet->size = sizeof(packet->data);
	total = 4 + packet->size;

	if (packet->is_conn) {
		flags |= LE_FLAG_DATA | LE_FLAG_COUNTER_VALID;
		if ((packet->event_pos & 1) == 0)
			flags |= LE_FLAG_SLAVE;
		if (conn.encrypted)
			flags |= LE_FLAG_ENCRYPTED;
	}
	if (packet->aux)
		flags |= LE_FLAG_AUX;
	if (packet->crc_ok >= 0)
		flags |= LE_FLAG_CRC_CHECKED;
	if (packet->crc_ok > 0)
		flags |= LE_FLAG_CRC_OK;
	if (total > DMA_SIZE)
		flags |= LE_FLAG_TRUNCATED;
	if (!host_ext) {
		// as API 1.07 firmware: the first DMA_SIZE bytes, no flags
		flags = 0;
		if (total > DMA_SIZE)
			total = DMA_SIZE;
	}

	for (off = 0, chunk = 0; off < total; off += DMA_SIZE, ++chunk) {
		f = usb_enqueue_reserve();
		if (f == NULL) {
			++drops;
			return;
		}
		f->status = 0;
		f->clk100ns = packet->timestamp;
		f->channel = (uint8_t)((packet->channel - 2402) & 0xff);
		if (chunk == 0) {
			f->pkt_type = LE_PACKET;
			f->clkn_high = flags;
			f->rssi_count = packet->rssi_count;
			f->rssi_avg = packet->rssi_count ?
				packet->rssi_sum / (int)packet->rssi_count : 0;
			f->rssi_min = packet->rssi_min;
			f->rssi_max = packet->rssi_max;
			f->reserved[0] = packet->counter & 0xff;
			f->reserved[1] = packet->counter >> 8;
		} else {
			f->pkt_type = LE_PACKET_CONT;
			f->clkn_high = chunk;
			f->rssi_count = 0;
			f->rssi_avg = f->rssi_min = f->rssi_max = 0;
			f->reserved[0] = f->reserved[1] = 0;
		}
		for (i = 0; i < DMA_SIZE; ++i)
			f->data[i] = off + i < total ? le_usb_byte(packet, off + i) : 0;
		usb_enqueue_commit();
	}
}

static void usb_send_state(const le_state_msg_t *msg) {
	usb_pkt_rx *f = usb_enqueue_reserve();
	if (f == NULL) {
		++drops;
		return;
	}
	f->pkt_type = LE_STATE;
	f->status = 0;
	f->clkn_high = 0;
	f->clk100ns = NOW;
	f->channel = 0;
	f->rssi_count = 0;
	memset(f->data, 0, DMA_SIZE);
	f->data[0] = msg->code;
	memcpy(f->data + 1, msg->data, msg->len);
	usb_enqueue_commit();
}

static void drain_state_queue(void) {
	while (state_tail != state_head) {
		usb_send_state(&state_queue[state_tail]);
		state_tail = (state_tail + 1) % STATE_QUEUE_SIZE;
	}
}

//////////////////////
// packet processing (main loop)

static unsigned extract_field(le_rx_t *buf, size_t offset, unsigned size) {
	unsigned i, ret = 0;

	for (i = 0; i < size; ++i)
		ret |= buf->data[offset + i] << (i*8);

	return ret;
}

static void chsel_record(const uint8_t *adva, uint8_t chsel) {
	unsigned i;
	for (i = 0; i < CHSEL_CACHE_SIZE; ++i) {
		if (chsel_cache[i].valid && memcmp(chsel_cache[i].addr, adva, 6) == 0) {
			chsel_cache[i].chsel = chsel;
			return;
		}
	}
	memcpy(chsel_cache[chsel_next].addr, adva, 6);
	chsel_cache[chsel_next].chsel = chsel;
	chsel_cache[chsel_next].valid = 1;
	chsel_next = (chsel_next + 1) % CHSEL_CACHE_SIZE;
}

// returns 0/1 for a cached ChSel, -1 if the advertiser is unknown
static int chsel_lookup(const uint8_t *adva) {
	unsigned i;
	for (i = 0; i < CHSEL_CACHE_SIZE; ++i)
		if (chsel_cache[i].valid && memcmp(chsel_cache[i].addr, adva, 6) == 0)
			return chsel_cache[i].chsel;
	return -1;
}

static void send_conn_start(void) {
	le_state_conn_t s;
	memset(&s, 0, sizeof(s));
	s.access_address = conn.access_address;
	s.crc_init = conn.crc_init;
	s.interval = conn.interval_units;
	s.latency = conn.latency;
	s.timeout = conn.timeout_units;
	memcpy(s.chm, conn.chm, 5);
	s.hop = conn.hop_increment;
	s.csa = conn.csa;
	s.flags = conn.flags;
	s.win_size = conn.win_size_units;
	s.sca = conn.sca;
	s.win_offset = conn.win_offset_units;
	state_push(LE_STATE_CONN_START, &s, sizeof(s));
}

static const uint16_t sca_ppm_table[8] = { 500, 250, 150, 100, 75, 50, 30, 20 };

// CONNECT_IND or AUX_CONNECT_REQ
static void le_connect_handler(le_rx_t *buf, int aux) {
	uint32_t aa, crc_init, interval, timeout, win_size, win_offset;
	uint16_t latency;
	uint8_t hop, sca, csa, csa_auto = 0;
	le_channel_remapping_t remapping;
	uint32_t s;

	if (!le.do_follow || conn_active() || conn.monitor)
		return;

	if (buf->size != 2 + 6 + 6 + 22 + 3)
		return;

	aa         = extract_field(buf, 14, 4);
	crc_init   = extract_field(buf, 18, 3);
	win_size   = extract_field(buf, 21, 1);
	win_offset = extract_field(buf, 22, 2);
	interval   = extract_field(buf, 24, 2);
	latency    = extract_field(buf, 26, 2);
	timeout    = extract_field(buf, 28, 2);
	hop        = buf->data[35] & 0x1f;
	sca        = buf->data[35] >> 5;

	if (aa == ADVERTISING_AA || aa == 0)
		return;

	// conn interval must be [7.5 ms, 4.0s] in units of 1.25 ms
	if (interval < 6 || interval > 3200)
		return;

	// window offset is in range [0, conn_interval]
	if (win_offset > interval)
		return;

	// win size is in range [1.25 ms, MIN(10 ms, conn_interval - 1.25 ms)]
	if (win_size < 1 || win_size > 8 || win_size > interval - 1)
		return;

	// connSupervisionTimeout is in range [100 ms, 32 s]
	if (timeout < 10 || timeout > 3200)
		return;

	le_parse_channel_map(&buf->data[30], &remapping);
	if (remapping.total_channels < 2)
		return;

	// CSA#2 is used if both the advertiser and the initiator set ChSel.
	// AUX_CONNECT_REQ always uses CSA#2.
	if (aux) {
		csa = 2;
	} else if (!(buf->data[0] & 0x20)) {
		csa = 1;
	} else {
		// the advertiser's ChSel, if its ADV_IND was seen, is only the
		// first guess: an ADV_IND with its address and the other ChSel is
		// easy to send, and would otherwise lock in the wrong algorithm
		int adv_chsel = chsel_lookup(&buf->data[8]);
		csa = adv_chsel == 0 ? 1 : 2;
		csa_auto = 1;
	}

	// CSA#1 hop increment must be in [5, 16]
	if (hop < 5 || hop > 16) {
		if (csa == 1 && !csa_auto)
			return;
		csa = 2;
		csa_auto = 0;
	}

	s = irq_save();

	reset_conn();
	reset_conn_event();
	conn.access_address      = aa;
	conn.crc_init            = crc_init;
	conn.crc_init_reversed   = le_reverse_crc_init(crc_init);
	conn.channel_id          = (aa >> 16) ^ (aa & 0xffff);
	conn.csa                 = csa;
	conn.csa_auto            = csa_auto;
	conn.hop_increment       = hop;
	conn.sca                 = sca;
	conn.sca_ppm             = sca_ppm_table[sca];
	conn.interval_units      = interval;
	conn.timeout_units       = timeout;
	conn.latency             = latency;
	conn.win_size_units      = win_size;
	conn.win_offset_units    = win_offset;
	conn.flags               = (csa_auto ? 0x01 : 0) | (aux ? 0x02 : 0);
	memcpy(conn.chm, &buf->data[30], 5);
	conn.chm[4] &= 0x1f;
	conn.remapping           = remapping;
	conn.conn_interval       = interval * USEC(1250);
	conn.supervision_timeout = timeout * MSEC(10);
	conn.win_size            = win_size * USEC(1250);
	conn.win_start           = buf->timestamp + PACKET_DURATION(buf) +
		(aux ? TX_WINDOW_DELAY_AUX : TX_WINDOW_DELAY_LEGACY) +
		win_offset * USEC(1250);
	conn.last_packet_ts      = buf->timestamp;

	adv_timer = ADV_TMR_NONE;
	aux_active = 0;
	schedule_next_event();

	irq_restore(s);

	send_conn_start();
}

// follow an AuxPtr to a secondary advertising channel
static void schedule_aux(le_rx_t *buf, const uint8_t *ptr) {
	uint8_t chan = ptr[0] & 0x3f;
	uint32_t unit = (ptr[0] & 0x80) ? USEC(300) : USEC(30);
	uint32_t offset = ((ptr[1] | (ptr[2] << 8)) & 0x1fff) * unit;
	uint8_t phy = ptr[2] >> 5;
	uint32_t start, widen, s, open, offset_us = offset / 10;
	int32_t slack_us;
	uint8_t d[8];

	if (!(adv_flags & LE_ADV_FOLLOW_AUX) || chan > 36)
		return;

	// AuxOffset is from the start of the packet containing the AuxPtr
	start = buf->timestamp + offset;
	widen = unit + offset / 10000 + USEC(16); // offset unit + 100 ppm
	open = start - widen - RX_WARMUP_TIME;
	slack_us = (int32_t)(open - NOW) / 10;

	d[0] = chan;
	d[1] = phy;
	memcpy(&d[2], &offset_us, 4);
	d[6] = slack_us & 0xff;
	d[7] = (slack_us >> 8) & 0xff;
	state_push(LE_STATE_AUX, d, 8);

	// the CC2400 only receives LE 1M
	if (phy != 0)
		return;

	s = irq_save();
	if (!conn_active() && !conn.monitor && aux_chain < AUX_CHAIN_MAX &&
			!((adv_flags & LE_ADV_HOP) && !aux_active &&
			  (int32_t)(open - adv_hop_at) > 0)) {
		++aux_chain;
		aux_chan = chan;
		aux_close = start + widen + MAX_PACKET_TIME + USEC(250);
		adv_timer = ADV_TMR_AUX_OPEN;
		timer1_set_match(open);
	}
	irq_restore(s);
}

// common extended advertising payload: find AdvA and AuxPtr
static void ext_adv_parse(le_rx_t *buf, const uint8_t **adva, const uint8_t **auxptr) {
	unsigned len = buf->data[1], hdr_len, pos, end;
	uint8_t flags;

	*adva = NULL;
	*auxptr = NULL;
	if (len < 2)
		return;
	hdr_len = buf->data[2] & 0x3f;     // includes the flags byte
	if (hdr_len < 1 || hdr_len + 1 > len)
		return;
	end = 3 + hdr_len;                 // first byte after the extended header
	flags = buf->data[3];
	pos = 4;
	if (flags & 0x01) {                // AdvA
		if (pos + 6 > end) return;
		*adva = &buf->data[pos];
		pos += 6;
	}
	if (flags & 0x02) pos += 6;        // TargetA
	if (flags & 0x04) pos += 1;        // CTEInfo
	if (flags & 0x08) pos += 2;        // ADI
	if ((flags & 0x10) && pos + 3 <= end) // AuxPtr
		*auxptr = &buf->data[pos];
}

static void adv_packet_handler(le_rx_t *buf) {
	const uint8_t *adva, *auxptr;

	if (buf->crc_ok != 1)
		return;

	switch (buf->data[0] & 0xf) {
		// ADV_IND, ADV_DIRECT_IND: remember ChSel for CSA selection
		case 0x0:
		case 0x1:
			if (buf->data[1] >= 6)
				chsel_record(&buf->data[2], (buf->data[0] >> 5) & 1);
			break;

		// CONNECT_IND, or AUX_CONNECT_REQ on a secondary channel
		case 0x5:
			le_connect_handler(buf, buf->aux);
			break;

		// ADV_EXT_IND / AUX_ADV_IND / AUX_CHAIN_IND ...
		case 0x7:
			ext_adv_parse(buf, &adva, &auxptr);
			if (auxptr)
				schedule_aux(buf, auxptr);
			break;
	}
}

// guess what an encrypted LL control PDU is from its length
static uint8_t enc_ctrl_guess(unsigned len, int slave) {
	switch (len) {
		case 1 + 4:  return LE_ENC_GUESS_ONE_BYTE;
		case 12 + 4: return slave ? LE_ENC_GUESS_UNKNOWN : LE_ENC_GUESS_CONN_UPDATE;
		case 8 + 4:  return slave ? LE_ENC_GUESS_UNKNOWN : LE_ENC_GUESS_CHMAP;
		case 5 + 4:  return slave ? LE_ENC_GUESS_UNKNOWN : LE_ENC_GUESS_PHY_UPDATE;
		case 2 + 4:  return LE_ENC_GUESS_TERMINATE;
	}
	return LE_ENC_GUESS_UNKNOWN;
}

// when the event counter is only known mod 37 (CSA#1 recovered by the
// host), an instant tells us the rest: instants are 6 or a few more
// events after the PDU carrying them
static void counter_from_instant(le_rx_t *buf, uint16_t instant) {
	uint16_t delta = instant - buf->counter;
	uint16_t shift = 37 * ((uint16_t)(delta - 6) / 37);
	uint32_t s;

	if (!conn.counter_uncertain)
		return;
	s = irq_save();
	conn.conn_event_counter += shift;
	conn.counter_uncertain = 0;
	irq_restore(s);
	buf->counter += shift;
}

// an instant must be in the future, less than 32767 events away
static int instant_valid(le_rx_t *buf, uint16_t instant) {
	uint16_t delta;
	counter_from_instant(buf, instant);
	delta = instant - buf->counter;
	return delta != 0 && delta < 32767;
}

static void data_packet_handler(le_rx_t *buf) {
	unsigned llid = buf->data[0] & 0x3, len = buf->data[1];
	int slave = (buf->event_pos & 1) == 0;
	uint16_t instant;
	uint32_t s;

	if (buf->crc_ok != 1 || llid != 3 || len == 0)
		return;

	if (conn.encrypted) {
		// a plaintext LL_ENC_REQ means encryption was paused
		if (len == 23 && buf->data[2] == 0x03 && !slave) {
			conn.encrypted = 0;
			return;
		}
		uint8_t guess = enc_ctrl_guess(len, slave);
		uint8_t d[5] = { len, slave, guess, buf->counter & 0xff, buf->counter >> 8 };
		state_push(LE_STATE_ENC_CTRL, d, 5);
		// a channel map change we can't read: learn it from misses
		if (guess == LE_ENC_GUESS_CHMAP)
			conn.map_learn = 1;
		if (guess == LE_ENC_GUESS_TERMINATE && conn.terminate_events == 0)
			conn.terminate_events = 1;
		if (guess == LE_ENC_GUESS_PHY_UPDATE) {
			conn.enc_phy_hint = 1;
			conn.enc_phy_counter = buf->counter;
		}
		return;
	}

	switch (buf->data[2]) {
		// LL_CONNECTION_UPDATE_IND (central only, as are the channel map
		// and PHY updates: from the peripheral's slot it is forged)
		case 0x00: {
			if (len != 12 || slave)
				break;
			uint32_t win_size = buf->data[3];
			uint32_t win_offset = extract_field(buf, 4, 2);
			uint32_t interval = extract_field(buf, 6, 2);
			uint32_t latency = extract_field(buf, 8, 2);
			uint32_t timeout = extract_field(buf, 10, 2);
			instant = extract_field(buf, 12, 2);
			if (interval < 6 || interval > 3200 || win_size < 1 ||
					win_size > 8 || win_size > interval - 1 ||
					win_offset > interval || timeout < 10 || timeout > 3200)
				break;
			if (!instant_valid(buf, instant))
				break;
			s = irq_save();
			conn.upd_win_size = win_size * USEC(1250);
			conn.upd_win_offset = win_offset * USEC(1250);
			conn.upd_interval = interval * USEC(1250);
			conn.upd_timeout = timeout * MSEC(10);
			conn.upd_interval_units = interval;
			conn.upd_timeout_units = timeout;
			conn.upd_latency = latency;
			conn.conn_update_instant = instant;
			conn.conn_update_pending = 1;
			irq_restore(s);
			uint8_t d[6] = { interval & 0xff, interval >> 8, timeout & 0xff,
				timeout >> 8, instant & 0xff, instant >> 8 };
			state_push(LE_STATE_CONN_UPDATE, d, 6);
			break;
		}

		// LL_CHANNEL_MAP_IND
		case 0x01: {
			le_channel_remapping_t r;
			if (len != 8 || slave)
				break;
			instant = extract_field(buf, 8, 2);
			le_parse_channel_map(&buf->data[3], &r);
			if (r.total_channels < 2 || !instant_valid(buf, instant))
				break;
			s = irq_save();
			memcpy(conn.pending_chm, &buf->data[3], 5);
			conn.pending_chm[4] &= 0x1f;
			conn.channel_map_update_instant = instant;
			conn.channel_map_update_pending = 1;
			irq_restore(s);
			uint8_t d[7];
			memcpy(d, &buf->data[3], 5);
			d[5] = instant & 0xff;
			d[6] = instant >> 8;
			state_push(LE_STATE_CHMAP_UPDATE, d, 7);
			break;
		}

		// LL_TERMINATE_IND: the link ends once it is acknowledged
		case 0x02:
			if (len != 2)
				break;
			state_push_counter(LE_STATE_TERMINATE, buf->data[3], buf->counter);
			if (conn.terminate_events == 0)
				conn.terminate_events = 1;
			break;

		// LL_START_ENC_REQ: everything after it is encrypted
		case 0x05:
			if (len != 1)
				break;
			conn.encrypted = 1;
			{
				uint8_t d[2] = { buf->counter & 0xff, buf->counter >> 8 };
				state_push(LE_STATE_ENC_START, d, 2);
			}
			break;

		// LL_PHY_UPDATE_IND
		case 0x18:
			if (len != 5 || slave)
				break;
			instant = extract_field(buf, 5, 2);
			if ((buf->data[3] | buf->data[4]) == 0 || !instant_valid(buf, instant))
				break;
			s = irq_save();
			conn.phy_m2s = buf->data[3];
			conn.phy_s2m = buf->data[4];
			conn.phy_update_instant = instant;
			conn.phy_update_pending = 1;
			irq_restore(s);
			{
				uint8_t d[4] = { buf->data[3], buf->data[4], instant & 0xff, instant >> 8 };
				state_push(LE_STATE_PHY_UPDATE, d, 4);
			}
			break;
	}
}

// compare a BD addr against target with mask
static int bd_addr_cmp(const uint8_t *bd_addr) {
	unsigned i;
	for (i = 0; i < 6; ++i)
		if ((bd_addr[i] & le.target_mask[i]) != le.target[i])
			return 0;
	return 1;
}

static int filter_match(le_rx_t *buf) {
	const uint8_t *adva, *auxptr;

	if (!le.target_set)
		return 1;

	// allow all data channel packets
	if (buf->access_address != ADVERTISING_AA)
		return 1;

	switch (buf->data[0] & 0xf) {
		// ADV_IND, ADV_NONCONN_IND, ADV_SCAN_IND, SCAN_RSP
		case 0x00:
		case 0x02:
		case 0x06:
		case 0x04:
			// header + one address
			if (buf->size < 2 + 6)
				return 0;
			return bd_addr_cmp(&buf->data[2]);

		// ADV_DIRECT_IND, SCAN_REQ, CONNECT_REQ
		case 0x01:
		case 0x03:
		case 0x05:
			// header + two addresses
			if (buf->size < 2 + 6 + 6)
				return 0;
			return bd_addr_cmp(&buf->data[2]) ||
				   bd_addr_cmp(&buf->data[8]);

		// extended advertising: ADV_EXT_IND usually has no AdvA (it is
		// in the AUX_ADV_IND), so pass those through
		case 0x07:
			ext_adv_parse(buf, &adva, &auxptr);
			return adva == NULL || bd_addr_cmp(adva);

		default:
			break;
	}

	return 0;
}

static void process_packet(le_rx_t *packet) {
	le_dewhiten(packet->data, packet->size, packet->channel);

	if (packet->access_address == ADVERTISING_AA)
		packet->crc_ok = le_crc_ok(ADVERTISING_CRC_INIT_REV, packet->data, packet->size);
	else if (packet->is_conn)
		packet->crc_ok = le_crc_ok(conn.crc_init_reversed, packet->data, packet->size);

	if (!filter_match(packet))
		return;

	blink(0, 1, 0); // RX LED

	// act on time critical packets (CONNECT_IND, AuxPtr) before USB
	if (packet->access_address == ADVERTISING_AA)
		adv_packet_handler(packet);
	else if (packet->is_conn)
		data_packet_handler(packet);

	usb_enqueue_le(packet);
}

//////////////////////
// host requests

void le_phy_request_monitor(uint16_t channel_mhz, uint32_t access_address) {
	monitor_channel = channel_mhz;
	monitor_aa = access_address;
	monitor_request = 1;
}

void le_phy_request_follow(const le_follow_params_t *params) {
	memcpy(&follow_params, params, sizeof(follow_params));
	follow_request = 1;
}

void le_phy_set_adv_config(uint16_t flags, uint16_t dwell_ms) {
	adv_flags = flags;
	adv_dwell = MSEC(dwell_ms ? dwell_ms : 100);
}

static void apply_monitor(void) {
	uint32_t s = irq_save();
	reset_conn();
	reset_conn_event();
	adv_timer = ADV_TMR_NONE;
	aux_active = 0;
	timer1_clear_match();
	if (monitor_channel >= 2402 && monitor_channel <= 2480) {
		conn.monitor = 1;
		tune_radio(btle_channel_index(monitor_channel), monitor_aa);
	} else {
		adv_resume();
	}
	irq_restore(s);
}

static void apply_follow(void) {
	le_follow_params_t *p = &follow_params;
	le_channel_remapping_t r;
	uint32_t s, timeout;

	if (p->interval < 6 || p->interval > 3200 || (p->csa != 1 && p->csa != 2))
		return;
	le_parse_channel_map(p->chm, &r);
	if (r.total_channels < 2)
		return;

	timeout = p->timeout ? p->timeout * MSEC(10) : SEC(4);
	if (timeout < 6 * p->interval * USEC(1250))
		timeout = 6 * p->interval * USEC(1250);

	s = irq_save();
	reset_conn();
	reset_conn_event();
	conn.access_address      = p->access_address;
	conn.crc_init            = p->crc_init & 0xffffff;
	conn.crc_init_reversed   = le_reverse_crc_init(p->crc_init);
	conn.channel_id          = (p->access_address >> 16) ^ (p->access_address & 0xffff);
	conn.csa                 = p->csa;
	conn.hop_increment       = p->hop % 37;
	conn.last_unmapped       = p->unmapped % 37;
	conn.sca                 = p->sca & 7;
	conn.sca_ppm             = sca_ppm_table[p->sca & 7];
	conn.interval_units      = p->interval;
	conn.timeout_units       = timeout / MSEC(10);
	conn.flags               = 0x04;
	memcpy(conn.chm, p->chm, 5);
	conn.chm[4] &= 0x1f;
	conn.remapping           = r;
	conn.conn_interval       = p->interval * USEC(1250);
	conn.supervision_timeout = timeout;
	conn.map_learn           = !!(p->flags & LE_FOLLOW_MAP_LEARN);
	conn.counter_uncertain   = !!(p->flags & LE_FOLLOW_COUNTER_UNKNOWN);
	conn.last_anchor         = p->anchor;
	conn.last_obs_anchor     = p->anchor;
	conn.anchor_set          = 1;
	conn.ever_synced         = 1;
	conn.conn_event_counter  = p->counter + 1;
	conn.last_packet_ts      = NOW;

	// skip events that have already passed
	while ((int32_t)(conn.last_anchor + conn.conn_interval - NOW) <
			(int32_t)(RX_WARMUP_TIME + USEC(500))) {
		conn.last_anchor += conn.conn_interval;
		++conn.conn_event_counter;
		conn.last_unmapped = (conn.last_unmapped + conn.hop_increment) % 37;
	}

	adv_timer = ADV_TMR_NONE;
	aux_active = 0;
	radio_stop();
	schedule_next_event();
	irq_restore(s);

	send_conn_start();
}

void le_phy_main(void) {
	uint32_t saved_ipr0 = IPR0, saved_ipr6 = IPR6;
	uint32_t reported_drops = 0, last_drop_report = 0;

	// disable USB interrupts -- we poll them below
	// n.b., they should not be enabled but let's be careful
	ICER0 = ICER0_ICE_USB;
	// disable clkn and timer0
	clkn_disable();

	// TIMER1 and DMA at the same priority: neither preempts the other,
	// so radio SPI transactions and shared state are never interleaved
	IPR0 |= IPR0_IP_TIMER1;
	IPR6 |= IPR6_IP_DMA;

	buffers_init();
	queue_init(&packet_queue);
	reset_conn();
	reset_conn_event();
	state_head = state_tail = 0;
	monitor_request = follow_request = 0;
	drops = 0;
	adv_timer = ADV_TMR_NONE;
	aux_active = 0;
	memset(chsel_cache, 0, sizeof(chsel_cache));
	timer1_start();

	switch (le_adv_channel) {
		case 2426: adv_primary_idx = 38; break;
		case 2480: adv_primary_idx = 39; break;
		default:   adv_primary_idx = 37; break;
	}

	current_rxbuf = buffer_get();
	rf_channel = le_adv_channel;
	le_sys_init();
	{
		uint32_t s = irq_save();
		adv_resume();
		irq_restore(s);
	}

	cancel_follow = 0;

	while (requested_mode == MODE_BT_FOLLOW_LE) {
		le_rx_t *packet = NULL;
		while (queue_remove(&packet_queue, (void **)&packet)) {
			process_packet(packet);
			buffer_release(packet);
		}

		drain_state_queue();

		if (monitor_request) {
			monitor_request = 0;
			apply_monitor();
		}

		if (follow_request) {
			follow_request = 0;
			apply_follow();
		}

		// cancel while not following anything: nothing to do
		if (cancel_follow && !conn_active())
			cancel_follow = 0;

		if (drops != reported_drops && NOW - last_drop_report > MSEC(100)) {
			uint32_t d = drops;
			reported_drops = d;
			last_drop_report = NOW;
			state_push(LE_STATE_USB_DROPS, &d, 4);
		}

		// polled USB handling
		handle_usb(0);
	}

	timer1_stop();

	// turn off the radio, DMA and SSP
	radio_stop();
	dio_ssp_stop();
#ifdef UBERTOOTH_ONE
	PAEN_CLR;
	HGM_CLR;
#endif
	IPR0 = saved_ipr0;
	IPR6 = saved_ipr6;
	// radio_stop() masked the DMA interrupt; the other modes expect it
	// enabled as dma_poweron() left it
	DMACIntTCClear = (1 << 0);
	DMACIntErrClr = (1 << 0);
	ISER0 = ISER0_ISE_DMA;

	// reset state
	reset_conn();
	RXLED_CLR;
	TXLED_CLR;
	USRLED_CLR;
	clkn_init();
}
