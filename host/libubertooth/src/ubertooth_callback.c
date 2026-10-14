/*
 * Copyright 2015 Hannes Ellinger
 *
 * This file is part of Project Ubertooth.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "ubertooth_callback.h"

extern unsigned int packet_counter_max;

static int8_t cc2400_rssi_to_dbm( const int8_t rssi )
{
	/* models the cc2400 datasheet fig 22 for 1M as piece-wise linear */
	if (rssi < -48) {
		return -120;
	}
	else if (rssi <= -45) {
		return 6*(rssi+28);
	}
	else if (rssi <= 30) {
		return (int8_t) ((99*((int)rssi-62))/110);
	}
	else if (rssi <= 35) {
		return (int8_t) ((60*((int)rssi-35))/11);
	}
	else {
		return 0;
	}
}

static uint16_t get_le16(const uint8_t* p)
{
	return (uint16_t)(p[0] | (p[1] << 8));
}

static uint32_t get_le32(const uint8_t* p)
{
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
	       ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

#define RSSI_HISTORY_LEN 10

/* Ignore packets with a SNR lower than this in order to reduce
 * processor load.  TODO: this should be a command line parameter. */

static int8_t rssi_history[NUM_BREDR_CHANNELS][RSSI_HISTORY_LEN] = {{INT8_MIN}};

static void determine_signal_and_noise( usb_pkt_rx *rx, int8_t * sig, int8_t * noise )
{
	int8_t * channel_rssi_history = rssi_history[rx->channel];
	int8_t rssi;
	int i;

	/* Shift rssi max history and append current max */
	memmove(channel_rssi_history,
	        channel_rssi_history+1,
	        RSSI_HISTORY_LEN-1);
	channel_rssi_history[RSSI_HISTORY_LEN-1] = rx->rssi_max;

#if 0
	/* Signal starts in oldest bank, but may cross into second
	 * oldest bank.  Take the max or the 2 maxs. */
	rssi = MAX(channel_rssi_history[0], channel_rssi_history[1]);
#else
	/* Alternatively, use all banks in history. */
	rssi = channel_rssi_history[0];
	for (i = 1; i < RSSI_HISTORY_LEN; i++)
		rssi = MAX(rssi, channel_rssi_history[i]);
#endif
	*sig = cc2400_rssi_to_dbm( rssi );

	/* Noise is an IIR of averages */
	/* FIXME: currently bogus */
	*noise = cc2400_rssi_to_dbm( rx->rssi_avg );
}

static uint64_t now_ns( void )
{
	struct timespec ts = { 0, 0 };
	(void) clock_gettime( CLOCK_REALTIME, &ts );
	return (1000000000ull*(uint64_t) ts.tv_sec) + (uint64_t) ts.tv_nsec;
}

/*
 * Extend a wrapping 100 ns device counter to 64 bits and anchor it to
 * the host clock at the first packet. A timestamp slightly older than
 * the previous one (reordering between packet types) is not counted as
 * a wrap; only a jump back by more than half the period is.
 */
static uint64_t ts_track(ubertooth_ts_tracker* t, uint32_t ts, uint64_t wrap)
{
	uint64_t ticks;

	if (ts >= wrap)
		ts = (uint32_t)(ts % wrap);

	if (!t->started) {
		t->started = 1;
		t->start = ts;
		t->last = ts;
		t->wraps = 0;
		t->abs_start_ns = now_ns();
	} else if (ts < t->last && (uint64_t)(t->last - ts) > wrap / 2) {
		t->wraps++;
		t->last = ts;
	} else if (ts > t->last && (uint64_t)(ts - t->last) > wrap / 2) {
		/* late packet from before the most recent wrap */
		if (t->wraps > 0)
			return t->abs_start_ns +
			       100ull * ((t->wraps - 1) * wrap + ts - t->start);
	} else if (ts > t->last) {
		t->last = ts;
	}

	ticks = t->wraps * wrap + ts;
	if (ticks < t->start)  /* slightly before the first packet */
		return t->abs_start_ns - 100ull * (t->start - ticks);
	return t->abs_start_ns + 100ull * (ticks - t->start);
}

static int is_le_type(uint8_t pkt_type)
{
	return pkt_type == LE_PACKET || pkt_type == LE_PACKET_CONT ||
	       pkt_type == LE_STATE;
}

uint64_t ubertooth_packet_time_ns(ubertooth_t* ut, const usb_pkt_rx* rx)
{
	uint32_t ts = le32toh(rx->clk100ns);

	if (is_le_type(rx->pkt_type) && ut->le_timebase == UBERTOOTH_TIMEBASE_T1)
		return ts_track(&ut->ts_le, ts, 1ULL << 32);
	return ts_track(&ut->ts_br, ts, CLK100NS_WRAP);
}

/* Sniff for LAPs. If a piconet is provided, use the given LAP to
 * search for UAP.
 */
void cb_scan(ubertooth_t* ut, void* args __attribute__((unused)))
{
	btbb_packet* pkt = NULL;
	int8_t signal_level;
	int8_t noise_level;
	int8_t snr;
	int offset;
	uint32_t clkn;
	uint32_t clk100ns;

	usb_pkt_rx usb = fifo_pop(ut->fifo);
	usb_pkt_rx* rx = &usb;
	char syms[BANK_LEN];

	if (rx->pkt_type != BR_PACKET)
		goto out;

	/* Sanity check */
	if (rx->channel > (NUM_BREDR_CHANNELS-1))
		goto out;

	ubertooth_unpack_symbols((uint8_t*)rx->data, syms);
	clk100ns = le32toh(rx->clk100ns);

	determine_signal_and_noise( rx, &signal_level, &noise_level );
	snr = signal_level - noise_level;

	/* Pass packet-pointer-pointer so that
	 * packet can be created in libbtbb. */
	offset = btbb_find_ac(syms, BANK_LEN - 64, LAP_ANY, max_ac_errors, &pkt);
	if (offset < 0)
		goto out;

	/* Once offset is known for a valid packet, copy in symbols
	 * and other rx data. CLKN here is the 312.5us CLK27-0. The
	 * btbb library can shift it be CLK1 if needed. */
	clkn = (rx->clkn_high << 20) + (clk100ns + offset*10) / 3125;
	btbb_packet_set_data(pkt, syms + offset, BANK_LEN - offset,
	                     rx->channel, clkn);

	printf("systime=%u ch=%2d LAP=%06x err=%u clk100ns=%u clk1=%u s=%d n=%d snr=%d\n",
	       (int)time(NULL),
	       btbb_packet_get_channel(pkt),
	       btbb_packet_get_lap(pkt),
	       btbb_packet_get_ac_errors(pkt),
	       clk100ns,
	       btbb_packet_get_clkn(pkt),
	       signal_level,
	       noise_level,
	       snr);

	btbb_process_packet(pkt, NULL);

out:
	if (pkt)
		btbb_packet_unref(pkt);
}

void cb_afh_initial(ubertooth_t* ut, void* args)
{
	btbb_piconet* pn = (btbb_piconet*)args;
	btbb_packet* pkt = NULL;
	uint8_t channel;

	usb_pkt_rx usb = fifo_pop(ut->fifo);
	usb_pkt_rx* rx = &usb;
	char syms[BANK_LEN];

	if (rx->pkt_type != BR_PACKET || rx->channel >= NUM_BREDR_CHANNELS)
		goto out;

	ubertooth_unpack_symbols((uint8_t*)rx->data, syms);

	if( btbb_find_ac(syms, BANK_LEN - 64, btbb_piconet_get_lap(pn), max_ac_errors, &pkt) < 0 )
		goto out;

	/* detect AFH map
	 * set current channel as used channel and send updated AFH
	 * map to ubertooth */
	channel = rx->channel;
	if(btbb_piconet_set_channel_seen(pn, channel)) {

		/* Don't allow single unused channels */
		if (!btbb_piconet_get_channel_seen(pn, channel+1) &&
		    btbb_piconet_get_channel_seen(pn, channel+2))
		{
			printf("activating additional channel %d\n", channel+1);
		    btbb_piconet_set_channel_seen(pn, channel+1);
		}
		if (!btbb_piconet_get_channel_seen(pn, channel-1) &&
		    btbb_piconet_get_channel_seen(pn, channel-2))
		{
			printf("activating additional channel %d\n", channel-1);
			btbb_piconet_set_channel_seen(pn, channel-1);
		}

		cmd_set_afh_map(ut->devh, btbb_piconet_get_afh_map(pn));
		btbb_print_afh_map(pn);
	}
	cmd_hop(ut->devh);

out:
	if (pkt)
		btbb_packet_unref(pkt);
}

void cb_afh_monitor(ubertooth_t* ut, void* args)
{
	btbb_piconet* pn = (btbb_piconet*)args;
	btbb_packet* pkt = NULL;
	uint8_t channel;
	int i;

	usb_pkt_rx usb = fifo_pop(ut->fifo);
	usb_pkt_rx* rx = &usb;
	char syms[BANK_LEN];

	static unsigned long last_seen[NUM_BREDR_CHANNELS] = {0};
	static unsigned long counter = 0;

	if (rx->pkt_type != BR_PACKET || rx->channel >= NUM_BREDR_CHANNELS)
		goto out;

	ubertooth_unpack_symbols((uint8_t*)rx->data, syms);

	if( btbb_find_ac(syms, BANK_LEN - 64, btbb_piconet_get_lap(pn), max_ac_errors, &pkt) < 0 )
		goto out;

	counter++;
	channel = rx->channel;
	last_seen[channel]=counter;

	if(btbb_piconet_set_channel_seen(pn, channel)) {
		printf("+ channel %2d is used now\n", channel);
		btbb_print_afh_map(pn);
	// } else {
	// 	printf("channel %d is already used\n", channel);
	}

	for(i=0; i<NUM_BREDR_CHANNELS; i++) {
		if((counter - last_seen[i] >= packet_counter_max)) {
			if(btbb_piconet_clear_channel_seen(pn, i)) {
				printf("- channel %2d is not used any more\n", i);
				btbb_print_afh_map(pn);
			}
		}
	}
	cmd_hop(ut->devh);

out:
	if (pkt)
		btbb_packet_unref(pkt);
}

void cb_afh_r(ubertooth_t* ut, void* args)
{
	btbb_piconet* pn = (btbb_piconet*)args;
	btbb_packet* pkt = NULL;
	uint8_t channel;
	int i;

	usb_pkt_rx usb = fifo_pop(ut->fifo);
	usb_pkt_rx* rx = &usb;
	char syms[BANK_LEN];

	static unsigned long last_seen[NUM_BREDR_CHANNELS] = {0};
	static unsigned long counter = 0;

	if (rx->pkt_type != BR_PACKET || rx->channel >= NUM_BREDR_CHANNELS)
		goto out;

	ubertooth_unpack_symbols((uint8_t*)rx->data, syms);

	if( btbb_find_ac(syms, BANK_LEN - 64, btbb_piconet_get_lap(pn), max_ac_errors, &pkt) < 0 )
		goto out;

	counter++;
	channel = rx->channel;
	last_seen[channel]=counter;

	btbb_piconet_set_channel_seen(pn, channel);

	for(i=0; i<NUM_BREDR_CHANNELS; i++) {
		if((counter - last_seen[i] >= packet_counter_max)) {
			btbb_piconet_clear_channel_seen(pn, i);
		}
	}
	cmd_hop(ut->devh);

out:
	if (pkt)
		btbb_packet_unref(pkt);
}

/*
 * ---------------------------------------------------------------------
 * Bluetooth Low Energy
 * ---------------------------------------------------------------------
 */

/* access address + header + max payload + CRC */
#define LE_MAX_PACKET (4 + 2 + 255 + 3)

/* reassembly of LE_PACKET (LE_FLAG_TRUNCATED) + LE_PACKET_CONT chunks */
static struct {
	int active;
	usb_pkt_rx first;          /* metadata of the first chunk */
	size_t need;               /* total length (AA..CRC) */
	size_t have;               /* bytes received so far */
	uint8_t next_chunk;        /* expected clkn_high of the next CONT */
	uint8_t buf[LE_MAX_PACKET + DMA_SIZE];
} le_reasm;

static unsigned long le_reasm_errors = 0;

static size_t le_packet_len(const uint8_t* data)
{
	/* AA (4) + header (2) + payload + CRC (3) */
	size_t len = 4 + 2 + data[5] + 3;
	return len > LE_MAX_PACKET ? LE_MAX_PACKET : len;
}

static const char* le_phy_str(uint8_t phy)
{
	switch (phy) {
		case 0: return "none";
		case 1: return "1M";
		case 2: return "2M";
		case 3: return "1M|2M";
		case 4: return "Coded";
		case 5: return "1M|Coded";
		case 6: return "2M|Coded";
		case 7: return "1M|2M|Coded";
		default: return "?";
	}
}

static unsigned chm_count(const uint8_t* chm)
{
	unsigned i, n = 0;
	for (i = 0; i < 37; i++)
		if (chm[i / 8] & (1 << (i % 8)))
			n++;
	return n;
}

static void print_chm(const uint8_t* chm)
{
	printf("%02x%02x%02x%02x%02x (%u channels)",
	       chm[0], chm[1], chm[2], chm[3], chm[4], chm_count(chm));
}

static const char* le_lost_reason_str(uint8_t reason)
{
	switch (reason) {
		case LE_LOST_SUPERVISION:  return "supervision timeout";
		case LE_LOST_CANCELED:    return "canceled";
		case LE_LOST_PHY:          return "switched to an unsupported PHY";
		case LE_LOST_TERMINATED:   return "terminated";
		case LE_LOST_NEVER_SYNCED: return "never synchronized";
		default:                   return "unknown reason";
	}
}

static const char* le_enc_guess_str(uint8_t guess)
{
	switch (guess) {
		case LE_ENC_GUESS_CONN_UPDATE: return "LL_CONNECTION_UPDATE_IND?";
		case LE_ENC_GUESS_CHMAP:       return "LL_CHANNEL_MAP_IND?";
		case LE_ENC_GUESS_PHY_UPDATE:  return "LL_PHY_UPDATE_IND / LL_POWER_CHANGE_IND?";
		case LE_ENC_GUESS_TERMINATE:   return "LL_TERMINATE_IND?";
		case LE_ENC_GUESS_ONE_BYTE:    return "LL_START_ENC_RSP / LL_PING_* / LL_PAUSE_ENC_*?";
		default:                       return "unknown";
	}
}

/* set when the firmware reports LE_STATE_CONN_LOST */
static volatile int le_conn_lost_flag = 0;

int ubertooth_le_conn_lost(ubertooth_t* ut __attribute__((unused)))
{
	int f = le_conn_lost_flag;
	le_conn_lost_flag = 0;
	return f;
}

/* LE_STATE: firmware connection-following state change */
static void le_print_state(ubertooth_t* ut, const usb_pkt_rx* rx)
{
	const uint8_t* d = rx->data;
	uint64_t ns = ubertooth_packet_time_ns(ut, rx);

	printf("[%llu.%06llu] LE state: ", (unsigned long long)(ns / 1000000000ull),
	       (unsigned long long)((ns / 1000) % 1000000ull));

	switch (d[0]) {
	case LE_STATE_CONN_START: {
		le_state_conn_t c;
		memcpy(&c, &d[1], sizeof(c));
		printf("following connection AA=%08x CRCInit=%06x interval=%.2f ms latency=%u "
		       "timeout=%u ms hop=%u CSA#%u win_size=%.2f ms win_offset=%.2f ms sca=%u chm=",
		       le32toh(c.access_address), le32toh(c.crc_init) & 0xffffff,
		       le16toh(c.interval) * 1.25, le16toh(c.latency),
		       le16toh(c.timeout) * 10u, c.hop, c.csa,
		       c.win_size * 1.25, le16toh(c.win_offset) * 1.25, c.sca);
		print_chm(c.chm);
		if (c.flags & 0x01) printf(" [CSA guessed]");
		if (c.flags & 0x02) printf(" [AUX_CONNECT_REQ]");
		if (c.flags & 0x04) printf(" [host parameters]");
		printf("\n");
		break;
	}
	case LE_STATE_CONN_LOST:
		le_conn_lost_flag = 1;
		printf("connection lost: %s at event %u", le_lost_reason_str(d[1]), get_le16(&d[2]));
		if (d[1] == LE_LOST_PHY && (d[4] | d[5]))
			printf(" (M->S %s, S->M %s)", le_phy_str(d[4]), le_phy_str(d[5]));
		else if (d[1] == LE_LOST_PHY)
			printf(" (after an encrypted LL_PHY_UPDATE_IND; probably LE 2M)");
		printf("\n");
		break;
	case LE_STATE_CSA_SWITCH:
		printf("switched to CSA#%u at event %u\n", d[1], get_le16(&d[2]));
		break;
	case LE_STATE_CONN_UPDATE:
		printf("connection update: interval=%.2f ms timeout=%u ms at instant %u\n",
		       get_le16(&d[1]) * 1.25, get_le16(&d[3]) * 10u, get_le16(&d[5]));
		break;
	case LE_STATE_CHMAP_UPDATE:
		printf("channel map update: ");
		print_chm(&d[1]);
		printf(" at instant %u\n", get_le16(&d[6]));
		break;
	case LE_STATE_PHY_UPDATE:
		printf("PHY update: M->S %s, S->M %s at instant %u\n",
		       le_phy_str(d[1]), le_phy_str(d[2]), get_le16(&d[3]));
		break;
	case LE_STATE_ENC_START:
		printf("encryption started at event %u\n", get_le16(&d[1]));
		break;
	case LE_STATE_ENC_CTRL:
		printf("encrypted LL control PDU from %s, length %u, probably %s, event %u\n",
		       d[2] ? "slave" : "master", d[1], le_enc_guess_str(d[3]), get_le16(&d[4]));
		break;
	case LE_STATE_MAP_LEARN:
		printf("learned channel map: ");
		print_chm(&d[1]);
		printf("\n");
		break;
	case LE_STATE_TERMINATE:
		printf("LL_TERMINATE_IND, error 0x%02x, at event %u\n", d[1], get_le16(&d[2]));
		break;
	case LE_STATE_ANCHOR:
		printf("anchor acquired at %u (100 ns ticks), event %u\n",
		       get_le32(&d[1]), get_le16(&d[5]));
		break;
	case LE_STATE_USB_DROPS:
		printf("%u packet(s) dropped on the device (USB queue full)\n", get_le32(&d[1]));
		break;
	case LE_STATE_AUX:
		printf("AuxPtr to channel %u, %s, offset %u us, slack %d us%s\n", d[1],
		       d[2] == 0 ? "1M" : d[2] == 1 ? "2M" : d[2] == 2 ? "Coded" : "?",
		       get_le32(&d[3]), (int16_t)get_le16(&d[7]),
		       d[2] == 0 ? "" : " (not followed: CC2400 is 1M only)");
		break;
	default:
		printf("unknown state code %u\n", d[0]);
		break;
	}
}

/* legacy promiscuous-mode state reports */
static void le_print_promisc(const usb_pkt_rx* rx)
{
	uint8_t state = rx->data[0];
	const uint8_t* val = &rx->data[1];

	printf("--------------------\n");
	printf("LE Promisc - ");
	switch (state) {
		case 0:
			printf("Access Address: %08x\n", get_le32(val));
			break;
		case 1:
			printf("CRC Init: %06x\n", get_le32(val) & 0xffffff);
			break;
		case 2:
			printf("Hop interval: %g ms\n", get_le16(val) * 1.25);
			break;
		case 3:
			printf("Hop increment: %u\n", val[0]);
			break;
		default:
			printf("Unknown %u\n", state);
			break;
	}
	printf("\n");
}

/*
 * LE packet flags (API 0x0108) live in clkn_high. Only the LE follow
 * mode (T1 time base) fills them in; in promiscuous mode the field still
 * carries CLKN bits and must not be interpreted as flags.
 */
static uint8_t le_flags(const ubertooth_t* ut, const usb_pkt_rx* rx)
{
	if (rx->pkt_type != LE_PACKET || ut->le_timebase != UBERTOOTH_TIMEBASE_T1)
		return 0;
	return rx->clkn_high;
}

static void le_print_flags(const usb_pkt_rx* rx, uint8_t flags)
{
	if (flags == 0)
		return;

	printf(" [");
	if (flags & LE_FLAG_DATA)
		printf("%s", (flags & LE_FLAG_SLAVE) ? "S->M" : "M->S");
	else
		printf("adv");
	if (flags & LE_FLAG_COUNTER_VALID)
		printf(" event=%u", get_le16(rx->reserved));
	if (flags & LE_FLAG_ENCRYPTED)
		printf(" encrypted");
	if (flags & LE_FLAG_CRC_CHECKED)
		printf(" crc=%s", (flags & LE_FLAG_CRC_OK) ? "ok" : "BAD");
	if (flags & LE_FLAG_AUX)
		printf(" aux");
	printf("]");
}

/* decode, print and save one complete (or as complete as possible) LE packet */
static void le_process(ubertooth_t* ut, const usb_pkt_rx* rx, const uint8_t* buf,
                       size_t len, int incomplete, btle_options* opts)
{
	uint8_t flags = le_flags(ut, rx);
	static uint64_t prev_ns = 0;
	lell_packet* pkt = NULL;
	uint8_t stream[LE_MAX_PACKET + DMA_SIZE];
	uint32_t refAA;
	int8_t sig, noise;
	uint64_t nowns;
	size_t i, plen;

	/* the decoder may read a fixed number of bytes: pad with zeros */
	memset(stream, 0, sizeof(stream));
	memcpy(stream, buf, len);

	nowns = ubertooth_packet_time_ns(ut, rx);

#ifdef HAVE_LELL_ALLOCATE_AND_DECODE_LEN
	lell_allocate_and_decode_len(stream, len, rx->channel + 2402,
	                             le32toh(rx->clk100ns), &pkt);
#else
	lell_allocate_and_decode(stream, rx->channel + 2402,
	                         le32toh(rx->clk100ns), &pkt);
#endif
	if (pkt == NULL)
		return;

#ifdef HAVE_LELL_PACKET_SET_ENCRYPTED
	if (flags & LE_FLAG_ENCRYPTED)
		lell_packet_set_encrypted(pkt, 1);
#endif
#ifdef HAVE_LELL_PACKET_SET_DIRECTION
	/* direction and CRC status known to the firmware go into captures */
	if (flags & LE_FLAG_DATA)
		lell_packet_set_direction(pkt, (flags & LE_FLAG_SLAVE) ? 2 : 1);
	if (flags & LE_FLAG_CRC_CHECKED)
		lell_packet_set_crc_status(pkt, 1, !!(flags & LE_FLAG_CRC_OK));
#endif

	/* do nothing further if filtered due to bad AA */
	if (opts &&
	    (opts->allowed_access_address_errors <
	     lell_get_access_address_offenses(pkt))) {
		lell_packet_unref(pkt);
		return;
	}

	/* Dump to PCAP/PCAPNG if specified */
	refAA = lell_packet_is_data(pkt) ? 0 : 0x8e89bed6;
	sig = cc2400_rssi_to_dbm( rx->rssi_max );
	noise = INT8_MIN; // FIXME - keep track of this

	if (ut->h_pcap_le) {
		/* only one of these two will succeed, depending on
		 * whether PCAP was opened with DLT_PPI or not */
		lell_pcap_append_packet(ut->h_pcap_le, nowns,
					sig, noise,
					refAA, pkt);
		// read the above comment: this function may silently fail
		lell_pcap_append_ppi_packet(ut->h_pcap_le, nowns,
		                            rx->clkn_high,
		                            rx->rssi_min, rx->rssi_max,
		                            rx->rssi_avg, rx->rssi_count,
		                            pkt);
	}
	if (ut->h_pcapng_le) {
		lell_pcapng_append_packet(ut->h_pcapng_le, nowns,
		                          sig, noise,
		                          refAA, pkt);
	}

	printf("systime=%u freq=%d addr=%08x delta_t=%.03f ms rssi=%d",
	       systime, rx->channel + 2402, lell_get_access_address(pkt),
	       prev_ns ? (double)(int64_t)(nowns - prev_ns) / 1e6 : 0.0,
	       rx->rssi_min - 54);
	prev_ns = nowns;
	le_print_flags(rx, flags);
	if (incomplete)
		printf(" [incomplete: %zu bytes]", len);
	printf("\n");

	plen = len;
	for (i = 4; i < plen; ++i)
		printf("%02x ", buf[i]);
	printf("\n");

	lell_print(pkt);
	printf("\n");

	lell_packet_unref(pkt);
}

static void le_reasm_flush(ubertooth_t* ut, btle_options* opts)
{
	if (!le_reasm.active)
		return;
	le_reasm.active = 0;
	le_process(ut, &le_reasm.first, le_reasm.buf, le_reasm.have, 1, opts);
}

/*
 * Sniff Bluetooth Low Energy packets.
 */
void cb_btle(ubertooth_t* ut, void* args)
{
	btle_options* opts = (btle_options*) args;
	usb_pkt_rx usb = fifo_pop(ut->fifo);
	usb_pkt_rx* rx = &usb;

	if (infile == NULL)
		systime = time(NULL);

	/* Dump every raw USB packet (including continuations and state
	 * messages) so a dump file can be replayed through cb_btle */
	if (dumpfile && (is_le_type(rx->pkt_type) || rx->pkt_type == LE_PROMISC)) {
		uint32_t systime_be = htobe32(systime);
		if (fwrite(&systime_be, sizeof(systime_be), 1, dumpfile) != 1 ||
		    fwrite(rx, sizeof(usb_pkt_rx), 1, dumpfile) != 1) {
			perror("dump file");
			ut->stop_ubertooth = 1;
		}
	}

	switch (rx->pkt_type) {
	case LE_PROMISC:
		le_reasm_flush(ut, opts);
		le_print_promisc(rx);
		break;

	case LE_STATE:
		/* state messages never interrupt a packet's continuations */
		le_print_state(ut, rx);
		break;

	case LE_PACKET_CONT: {
		size_t off, n;
		if (!le_reasm.active ||
		    rx->clkn_high != le_reasm.next_chunk ||
		    rx->clk100ns != le_reasm.first.clk100ns) {
			if (le_reasm_errors++ == 0)
				fprintf(stderr, "LE: unexpected continuation packet (chunk %u), dropped\n",
				        rx->clkn_high);
			le_reasm_flush(ut, opts);
			break;
		}
		off = (size_t)DMA_SIZE * rx->clkn_high;
		n = le_reasm.need > off ? le_reasm.need - off : 0;
		if (n > DMA_SIZE)
			n = DMA_SIZE;
		memcpy(&le_reasm.buf[off], rx->data, n);
		le_reasm.have = off + n;
		le_reasm.next_chunk++;
		if (le_reasm.have >= le_reasm.need) {
			le_reasm.active = 0;
			le_process(ut, &le_reasm.first, le_reasm.buf, le_reasm.need, 0, opts);
		}
		break;
	}

	case LE_PACKET: {
		size_t len;

		/* a new packet ends any unfinished reassembly */
		le_reasm_flush(ut, opts);

		/* Sanity check */
		if (rx->channel > 78) /* 2402..2480 MHz */
			break;

		len = le_packet_len(rx->data);
		if ((le_flags(ut, rx) & LE_FLAG_TRUNCATED) && len > DMA_SIZE) {
			le_reasm.active = 1;
			le_reasm.first = *rx;
			le_reasm.need = len;
			le_reasm.have = DMA_SIZE;
			le_reasm.next_chunk = 1;
			memcpy(le_reasm.buf, rx->data, DMA_SIZE);
			break;
		}
		le_process(ut, rx, rx->data, len < DMA_SIZE ? len : DMA_SIZE, 0, opts);
		break;
	}

	default:
		/* stale data from another mode, keep-alives */
		break;
	}

	fflush(stdout);
}

/*
 * Sniff E-GO packets
 */
void cb_ego(ubertooth_t* ut, void* args)
{
	int i;
	uint8_t len = *(uint8_t *)args;
	static u32 prev_ts = 0;
	usb_pkt_rx usb = fifo_pop(ut->fifo);
	usb_pkt_rx* rx = &usb;
	u32 rx_time, ts_diff;

	if (rx->pkt_type != EGO_PACKET)
		return;
	if (len > DMA_SIZE)
		len = DMA_SIZE;

	/* CLK100NS wraps at 3276800000 */
	rx_time = le32toh(rx->clk100ns);
	if (rx_time >= prev_ts)
		ts_diff = rx_time - prev_ts;
	else
		ts_diff = (u32)(rx_time + CLK100NS_WRAP - prev_ts);
	prev_ts = rx_time;
	printf("time=%u delta_t=%.06f ms freq=%d \n",
	       rx_time, ts_diff / 10000.0,
	       rx->channel + 2402);

	for (i = 0; i < len; ++i)
		printf("%02x ", rx->data[i]);
	printf("\n\n");

	fflush(stdout);
}

#define CLOCK_TRIM_THRESHOLD 2

/*
 * Classic Bluetooth.
 *
 * A packet usually does not fit in the 400 symbols of one USB packet,
 * so packets are processed one behind: the access code is searched for
 * in the previous bank and, if the current bank directly follows it on
 * the same channel, the current symbols are appended.
 */
#define BR_BANK_TICKS (BANK_LEN * 10)   /* 400 us in 100 ns ticks */

static int br_contiguous(const usb_pkt_rx* a, const usb_pkt_rx* b)
{
	uint32_t ta = le32toh(a->clk100ns), tb = le32toh(b->clk100ns);
	uint32_t diff = tb >= ta ? tb - ta : (uint32_t)(tb + CLK100NS_WRAP - ta);

	return a->channel == b->channel && !(b->status & DISCARD) &&
	       diff >= BR_BANK_TICKS - 50 && diff <= BR_BANK_TICKS + 50;
}

static void br_process(ubertooth_t* ut, usb_pkt_rx* rx, const usb_pkt_rx* next,
                       btbb_piconet* pn);

/*
 * A receiver that covers several channels at once (the HackRF backend) can
 * find one packet on neighboring channels too. Detections of the same LAP
 * within BR_SAME_TICKS of each other are one packet: the one with the
 * fewest access code errors is kept. A detection is therefore delivered
 * when the next one (or a bank BR_FLUSH_TICKS later) shows it has no rival.
 */
#define BR_SAME_TICKS  20                  /* 2 us */
#define BR_FLUSH_TICKS (BR_BANK_TICKS * 2)

typedef struct {
	int have;
	btbb_packet* pkt;
	btbb_piconet* pn;
	usb_pkt_rx rx;
	uint32_t ac_ticks;      /* untrimmed time of the access code */
	uint32_t clkn;
	uint16_t clk_offset;
	int8_t signal_level, noise_level, snr;
	uint64_t nowns;
	uint32_t lap;
	uint8_t uap;
} br_detection;

static br_detection br_pending;

static void br_deliver(ubertooth_t* ut, br_detection* d);

static uint32_t br_ticks_apart(uint32_t a, uint32_t b)
{
	uint32_t d = a >= b ? a - b : b - a;
	return d > CLK100NS_WRAP / 2 ? CLK100NS_WRAP - d : d;
}

static void br_flush(ubertooth_t* ut)
{
	if (br_pending.have) {
		br_pending.have = 0;
		br_deliver(ut, &br_pending);
	}
}

/*
 * Each buffer is processed together with the next buffer of the same
 * channel when the two are contiguous, so access codes that straddle a
 * buffer boundary are found. A buffer is kept per channel: a receiver that
 * delivers several channels at once interleaves them. A pending buffer is
 * processed alone when its channel's next buffer is not contiguous, or once
 * it is more than 1.5 buffers older than the newest packet, which on a
 * single hopping receiver is the next packet on another channel.
 */
void cb_rx(ubertooth_t* ut, void* args)
{
	static usb_pkt_rx prev[NUM_BREDR_CHANNELS];
	static uint8_t have_prev[NUM_BREDR_CHANNELS];
	static unsigned n_pending = 0;
	btbb_piconet* pn = (btbb_piconet *)args;
	usb_pkt_rx usb = fifo_pop(ut->fifo);
	unsigned ch, c;

	if (usb.pkt_type != BR_PACKET || usb.channel > (NUM_BREDR_CHANNELS-1))
		return;
	ch = usb.channel;

	if (br_pending.have &&
	    br_ticks_apart(le32toh(usb.clk100ns), br_pending.ac_ticks) > BR_FLUSH_TICKS)
		br_flush(ut);

	/* flush buffers of other channels that can no longer be continued */
	if (n_pending > have_prev[ch]) {
		uint32_t tb = le32toh(usb.clk100ns);
		for (c = 0; c < NUM_BREDR_CHANNELS; c++) {
			uint32_t ta, age;
			if (!have_prev[c] || c == ch)
				continue;
			ta = le32toh(prev[c].clk100ns);
			age = tb >= ta ? tb - ta : (uint32_t)(tb + CLK100NS_WRAP - ta);
			if (age > BR_BANK_TICKS * 3 / 2 && age < CLK100NS_WRAP / 2) {
				br_process(ut, &prev[c], NULL, pn);
				have_prev[c] = 0;
				n_pending--;
			}
		}
	}

	if (have_prev[ch]) {
		br_process(ut, &prev[ch], br_contiguous(&prev[ch], &usb) ? &usb : NULL, pn);
		have_prev[ch] = 0;
		n_pending--;
	}

	if (usb.status & DISCARD)
		return;
	prev[ch] = usb;
	have_prev[ch] = 1;
	n_pending++;
}

static void br_process(ubertooth_t* ut, usb_pkt_rx* rx, const usb_pkt_rx* next,
                       btbb_piconet* pn)
{
	btbb_packet* pkt = NULL;
	char syms[BANK_LEN * 2 + 64];
	int nsyms = BANK_LEN;
	int offset;
	uint16_t clk_offset;
	uint32_t clkn, clk100ns;
	int r;
	uint32_t lap = LAP_ANY;
	uint8_t uap = UAP_ANY;

	memset(syms, 0, sizeof(syms));
	ubertooth_unpack_symbols((uint8_t*)rx->data, syms);
	if (next) {
		ubertooth_unpack_symbols((uint8_t*)next->data, syms + BANK_LEN);
		nsyms += BANK_LEN;
	}
	clk100ns = le32toh(rx->clk100ns);

	uint64_t nowns = ubertooth_packet_time_ns(ut, rx);

	int8_t signal_level = rx->rssi_max;
	int8_t noise_level = rx->rssi_min;
	determine_signal_and_noise( rx, &signal_level, &noise_level );
	int8_t snr = signal_level - noise_level;

	/* Look for packets with specified LAP, if given. Otherwise
	 * search for any packet. */
	if (pn) {
		lap = btbb_piconet_get_flag(pn, BTBB_LAP_VALID) ? btbb_piconet_get_lap(pn) : LAP_ANY;
		uap = btbb_piconet_get_flag(pn, BTBB_UAP_VALID) ? btbb_piconet_get_uap(pn) : UAP_ANY;
	}

	/* Pass packet-pointer-pointer so that
	 * packet can be created in libbtbb. The search reads 64 symbols
	 * past each candidate offset; syms has zero padding for that. */
#ifdef HAVE_BTBB_FIND_AC_CHECKED
	/* promiscuous matches must look like real packets (random matches
	 * within reach of a codeword are rejected) */
	offset = btbb_find_ac_checked(syms, BANK_LEN, nsyms, lap, max_ac_errors, &pkt);
#else
	offset = btbb_find_ac(syms, BANK_LEN, lap, max_ac_errors, &pkt);
#endif
	if (offset < 0)
		goto out;

	/* Time of the first bit of the AC, in 100 ns ticks of the receive
	 * clock (the bank's timestamp marks its end, 400 us later). A device
	 * that cannot trim its own clock (a capture file, the HackRF backend)
	 * is trimmed here instead, so that the AC of the piconet's packets
	 * arrives CLK_TUNE_TIME after the rising edge of CLKN: packets then sit
	 * well inside their slot and CLK1 is consistent from packet to packet. */
	{
		/* 64 bits: the trim follows the piconet's drift for as long as
		 * the program runs, at up to 781 ticks per packet */
		static int64_t soft_trim = 0;
		static int soft_calibrated = 0;
		int soft_clock = pn != NULL && (infile != NULL || ut->devh == NULL);
		int64_t t = (int64_t)clk100ns + offset * 10 - 4000;

		if (soft_clock) {
			int32_t err = (int32_t)((((t + soft_trim) % 6250) + 6250) % 6250) - CLK_TUNE_TIME;
			if (err > 3125)
				err -= 6250;
			else if (err <= -3125)
				err += 6250;
			/* calibrate on the first packet, then follow drift */
			soft_trim -= soft_calibrated ? err / 4 : err;
			soft_calibrated = 1;
			t += soft_trim;
		}
		/* calculate the offset between the first bit of the AC and the rising edge of CLKN */
		clk_offset = (uint16_t)(((t % 6250) + 6250) % 6250);
		/* CLKN here is the 312.5us CLK27-0 (floor, also before a wrap) */
		clkn = (((uint32_t)rx->clkn_high << 20) +
		        (uint32_t)(t >= 0 ? t / 3125 : -((-t + 3124) / 3125))) & 0x0fffffff;
	}

	btbb_packet_set_modulation(pkt, BTBB_MOD_GFSK);
	btbb_packet_set_transport(pkt, BTBB_TRANSPORT_ANY);

	/* Once offset is known for a valid packet, copy in symbols
	 * and other rx data. The btbb library can shift CLKN to CLK1 if
	 * needed. Only symbols that were actually received are passed on. */
	btbb_packet_set_data(pkt, syms + offset, nsyms - offset,
	                     rx->channel, clkn);

	{
		uint32_t ac_ticks = (clk100ns + offset * 10) % CLK100NS_WRAP;
		br_detection* d = &br_pending;
		if (d->have && (btbb_packet_get_lap(d->pkt) != btbb_packet_get_lap(pkt) ||
		                br_ticks_apart(ac_ticks, d->ac_ticks) > BR_SAME_TICKS))
			br_flush(ut);
		if (d->have) {
			/* the same packet on another channel: keep the better copy */
			if (btbb_packet_get_ac_errors(pkt) >= btbb_packet_get_ac_errors(d->pkt))
				goto out;
			btbb_packet_unref(d->pkt);
		}
		d->have = 1;
		d->pkt = pkt;
		pkt = NULL;
		d->pn = pn;
		d->rx = *rx;
		d->ac_ticks = ac_ticks;
		d->clkn = clkn;
		d->clk_offset = clk_offset;
		d->signal_level = signal_level;
		d->noise_level = noise_level;
		d->snr = snr;
		d->nowns = nowns;
		d->lap = lap;
		d->uap = uap;
	}

out:
	if (pkt)
		btbb_packet_unref(pkt);
}

static void br_deliver(ubertooth_t* ut, br_detection* d)
{
	btbb_packet* pkt = d->pkt;
	btbb_piconet* pn = d->pn;
	usb_pkt_rx* rx = &d->rx;
	uint32_t clkn = d->clkn;
	uint16_t clk_offset = d->clk_offset;
	int8_t signal_level = d->signal_level, noise_level = d->noise_level, snr = d->snr;
	uint32_t lap = d->lap;
	uint8_t uap = d->uap;
	int r;

	static int trim_counter = 0;
	static int calibrated = 0;
	static uint32_t clkn_trim = 0;

	/* When reading from file, caller will read
	 * systime before calling this routine, so do
	 * not overwrite. Otherwise, get current time. */
	if (infile == NULL)
		systime = time(NULL);

	printf("systime=%u ch=%2d LAP=%06x err=%u clkn=%u clk_offset=%u s=%d n=%d snr=%d\n",
	       (uint32_t)time(NULL),
	       btbb_packet_get_channel(pkt),
	       btbb_packet_get_lap(pkt),
	       btbb_packet_get_ac_errors(pkt),
	       clkn,
	       clk_offset,
	       signal_level,
	       noise_level,
	       snr
	);

	/* calibrate Ubertooth clock such that the first bit of the AC
	 * arrives CLK_TUNE_TIME after the rising edge of CLKN */
	if (pn != NULL && infile == NULL && ut->devh != NULL) {
		int32_t elapsed = (int32_t)(clkn - clkn_trim);
		if (trim_counter < -CLOCK_TRIM_THRESHOLD
		    || ((clk_offset < CLK_TUNE_TIME) && !calibrated)) {
			printf("offset < CLK_TUNE_TIME\n");
			printf("CLK100ns Trim: %d\n", 6250 + clk_offset - CLK_TUNE_TIME);
			cmd_trim_clock(ut->devh, 6250 + clk_offset - CLK_TUNE_TIME);
			trim_counter = 0;
			if (calibrated && elapsed > 0) {
				int ppm = ((int)clk_offset - CLK_TUNE_TIME) * 320 / elapsed;
				printf("Clock drifted %d in %f s. %d PPM too slow.\n",
				       ((int)clk_offset - CLK_TUNE_TIME),
				       (double)elapsed / 3200, ppm);
				cmd_fix_clock_drift(ut->devh, ppm);
			}
			clkn_trim = clkn;
			calibrated = 1;
			goto out;
		} else if (trim_counter > CLOCK_TRIM_THRESHOLD
		           || ((clk_offset > CLK_TUNE_TIME) && !calibrated)) {
			printf("offset > CLK_TUNE_TIME\n");
			printf("CLK100ns Trim: %d\n", clk_offset - CLK_TUNE_TIME);
			cmd_trim_clock(ut->devh, clk_offset - CLK_TUNE_TIME);
			trim_counter = 0;
			if (calibrated && elapsed > 0) {
				int ppm = ((int)clk_offset - CLK_TUNE_TIME) * 320 / elapsed;
				printf("Clock drifted %d in %f s. %d PPM too fast.\n",
				       ((int)clk_offset - CLK_TUNE_TIME),
				       (double)elapsed / 3200, ppm);
				cmd_fix_clock_drift(ut->devh, ppm);
			}
			clkn_trim = clkn;
			calibrated = 1;
			goto out;
		}

		if (clk_offset < CLK_TUNE_TIME - CLK_TUNE_OFFSET) {
			trim_counter--;
			goto out;
		} else if (clk_offset > CLK_TUNE_TIME + CLK_TUNE_OFFSET) {
			trim_counter++;
			goto out;
		} else {
			trim_counter = 0;
		}
	}

	/* If dumpfile is specified, write out the bank(s) the packet
	 * was found in. */
	if (dumpfile) {
		uint32_t systime_be = htobe32(systime);
		if (fwrite(&systime_be, sizeof(systime_be), 1, dumpfile) != 1 ||
		    fwrite(rx, sizeof(usb_pkt_rx), 1, dumpfile) != 1) {
			perror("dump file");
			ut->stop_ubertooth = 1;
		}
	}

#ifdef HAVE_BTBB_SET_BEHIND
	/* expensive work in libbtbb waits while packets pile up here */
	btbb_set_behind(fifo_count(ut->fifo) > FIFO_BEHIND);
#endif
	r = btbb_process_packet(pkt, pn);

	/* Dump to PCAP/PCAPNG if specified */
	if (ut->h_pcap_bredr) {
		btbb_pcap_append_packet(ut->h_pcap_bredr, d->nowns,
		                        signal_level, noise_level,
		                        lap, uap, pkt);
	}
	if (ut->h_pcapng_bredr) {
		btbb_pcapng_append_packet(ut->h_pcapng_bredr, d->nowns,
		                          signal_level, noise_level,
		                          lap, uap, pkt);
	}

	if(pn != NULL && infile == NULL && r < 0) {
		cmd_start_hopping(ut->devh, btbb_piconet_get_clk_offset(pn), 0);
		calibrated = 0;
	}

out:
	if (pkt)
		btbb_packet_unref(pkt);
}
