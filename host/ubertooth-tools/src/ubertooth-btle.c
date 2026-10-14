/*
 * Copyright 2012-2016 Michael Ryan
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

#include "ubertooth.h"
#include "ubertooth_callback.h"
#include "ubertooth_le_recover.h"
#include <ctype.h>
#include <err.h>
#include <getopt.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <signal.h>

static volatile sig_atomic_t cancel_follow = 0;

// send SIGUSR1 to cancel following an active connection
static void cancel_follow_handler(int sig __attribute__((unused))) {
	cancel_follow = 1;
}

void print_mac(uint8_t *mac_address, int mask) {
	unsigned i;
	for (i = 0; i < 5; ++i)
		printf("%02x:", mac_address[i]);
	printf("%02x", mac_address[5]);
	if (mask >= 0)
		printf("/%u", mask);
}

int convert_mac_address(char *s, uint8_t *o, uint8_t *mask_out) {
	int i;
	char *mask_p;
	unsigned mask = 48; // default mask is entire BD ADDR

	if (strcmp(s, "none") == 0) {
		memset(o, 0, 6);
		*mask_out = 0;
		return 1;
	}

	// if there is a mask included, parse it
	mask_p = strchr(s, '/');
	if (mask_p != NULL) {
		char *end;
		*mask_p = '\0';
		++mask_p;
		mask = strtoul(mask_p, &end, 10);
		if (end == mask_p || *end != '\0' || mask > 48) {
			printf("Error: MAC mask must be between 0 and 48\n");
			return 0;
		}
	}
	*mask_out = mask; // see above, default 48, otherwise user-supplied

	// validate length
	if (strlen(s) != 6 * 2 + 5) {
		printf("Error: MAC address is wrong length\n");
		return 0;
	}

	// validate hex chars and : separators
	for (i = 0; i < 6*3; i += 3) {
		if (!isxdigit((unsigned char)s[i]) ||
			!isxdigit((unsigned char)s[i+1])) {
			printf("Error: MAC address contains invalid character(s)\n");
			return 0;
		}
		if (i < 5*3 && s[i+2] != ':') {
			printf("Error: MAC address contains invalid character(s)\n");
			return 0;
		}
	}

	// sanity: checked; convert
	for (i = 0; i < 6; ++i) {
		unsigned byte;
		sscanf(&s[i*3], "%02x",&byte);
		o[i] = byte;
	}

	return 1;
}

/*
 * ---------------------------------------------------------------------
 * -P: promiscuous mode with host-side parameter recovery (CSA#1 + CSA#2)
 *
 * 1. the firmware finds an access address (strongest with RSSI >= -m)
 * 2. LE follow mode, parked on data channels with that AA
 *    (UBERTOOTH_LE_MONITOR); packets feed ubertooth_le_recover
 * 3. the solved parameters are handed to the firmware
 *    (UBERTOOTH_LE_FOLLOW_PARAMS), which then follows the connection;
 *    if it is lost, recovery starts again
 * ---------------------------------------------------------------------
 */

static uint8_t mhz_to_chan_idx(unsigned mhz)
{
	if (mhz == 2402) return 37;
	if (mhz == 2426) return 38;
	if (mhz == 2480) return 39;
	if (mhz < 2426) return (mhz - 2404) / 2;
	return (mhz - 2428) / 2 + 11;
}

static unsigned chan_idx_to_mhz(unsigned idx)
{
	if (idx < 11) return 2404 + 2 * idx;
	if (idx < 37) return 2428 + 2 * (idx - 11);
	return idx == 37 ? 2402 : idx == 38 ? 2426 : 2480;
}

typedef enum { REC_AA, REC_MONITOR, REC_SURVEY, REC_FOLLOW } rec_phase_t;

typedef struct {
	rec_phase_t phase;
	uint32_t aa;
	int aa_found;
	unsigned promisc_mhz;
	le_recover_t rec;
	uint8_t monitor_chan;
	unsigned chan_switches;
	unsigned events_on_chan;
	time_t phase_start, chan_start;
	int lost;              // firmware reported CONN_LOST
	int started;           // firmware reported CONN_START
	unsigned chan_events[37];  // events seen per channel
	int survey_done;
	// channel survey
	uint8_t survey_chan;
	uint8_t survey_seen[37];
	uint8_t survey_chm[5];
	uint16_t interval;
	btle_options* opts;
	// the solver is the expensive step: it runs again only when the events
	// have grown by an eighth (or were halved to make room), and not while
	// packets pile up in the receive queue, so that a busy link (or someone
	// sending packets on it) cannot keep the receiver behind
	unsigned solve_events;
} recover_state_t;

static int recover_solve_due(ubertooth_t* ut, recover_state_t* st)
{
	unsigned n = st->rec.num_events, last = st->solve_events;
	if (n < 8 || n == last || (n > last && n < last + last / 8) ||
			fifo_count(ut->fifo) > FIFO_BEHIND)
		return 0;
	st->solve_events = n;
	return 1;
}

static void cb_recover(ubertooth_t* ut, void* args)
{
	recover_state_t* st = (recover_state_t*)args;
	usb_pkt_rx rx = fifo_pop(ut->fifo);

	if (rx.pkt_type == LE_PROMISC) {
		if (rx.data[0] == 0 && !st->aa_found) {
			memcpy(&st->aa, &rx.data[1], 4);
			st->aa = le32toh(st->aa);
			st->aa_found = 1;
		}
		return;
	}
	if (rx.pkt_type == LE_STATE) {
		if (rx.data[0] == LE_STATE_CONN_LOST)
			st->lost = 1;
		if (rx.data[0] == LE_STATE_CONN_START)
			st->started = 1;
		return;
	}
	if (rx.pkt_type != LE_PACKET)
		return;

	uint32_t aa;
	memcpy(&aa, rx.data, 4);
	aa = le32toh(aa);
	if (aa != st->aa || (rx.clkn_high & LE_FLAG_DATA))
		return;

	unsigned mhz = rx.channel + 2402;
	uint8_t idx = mhz_to_chan_idx(mhz);
	if (idx > 36)
		return;

	// PDU bytes available in this USB packet (long PDUs are truncated;
	// they still count as events, without a CRCInit vote)
	unsigned len = 2 + rx.data[5] + 3;
	if (len > DMA_SIZE - 4)
		len = DMA_SIZE - 4;

	if (st->phase == REC_SURVEY) {
		// only CRC-consistent packets count: a strong signal on an
		// adjacent channel or a false sync must not mark a channel used
		uint32_t best, ci;
		if (len == 2 + rx.data[5] + 3u && le_recover_get_crc(&st->rec, &best) >= 3) {
			ci = le_recover_crc_init(&rx.data[4], len - 3, &rx.data[4 + len - 3]);
			if (ci == best)
				st->survey_seen[idx] = 1;
		}
		return;
	}
	if (st->phase != REC_MONITOR)
		return;

	unsigned before = st->rec.num_events;
	le_recover_packet(&st->rec, le32toh(rx.clk100ns), idx, &rx.data[4], len);
	if (st->rec.num_events != before) {
		++st->events_on_chan;
		++st->chan_events[idx];
		printf("  [recover] AA %08x: event on ch %u (%u MHz) at %u, %u events, %u channel(s)\n",
		       st->aa, idx, mhz, le32toh(rx.clk100ns), st->rec.num_events,
		       le_recover_channels_seen(&st->rec));
	}
}

static int recover_monitor(ubertooth_t* ut, recover_state_t* st, uint8_t chan)
{
	st->monitor_chan = chan;
	st->events_on_chan = 0;
	st->chan_start = time(NULL);
	printf("  [recover] monitoring channel %u (%u MHz)\n", chan, chan_idx_to_mhz(chan));
	return cmd_le_monitor(ut->devh, chan_idx_to_mhz(chan), st->aa);
}

static int recover_try_solve(ubertooth_t* ut, recover_state_t* st, const uint8_t* chm)
{
	le_follow_params_t p;
	le_recover_quality_t q;
	uint32_t crc;
	unsigned votes = le_recover_get_crc(&st->rec, &crc);
	int ok;

	if (votes < 2)
		return 0;
	if (!st->interval)
		st->interval = le_recover_interval(&st->rec, 6);
	if (!st->interval)
		return 0;

	ok = chm ? le_recover_solve_map(&st->rec, st->interval, chm, &p, &q)
	         : le_recover_solve(&st->rec, st->interval, &p, &q);
	if (!ok) {
		if (chm)
			printf("  [recover] no solution with the surveyed map: interval %.2f ms, "
			       "%u events on %u channel(s), CRCInit %06x (%u votes)\n",
			       st->interval * 1.25, st->rec.num_events,
			       le_recover_channels_seen(&st->rec), crc, votes);
		return 0;
	}

	printf("  [recover] solved: AA %08x CRCInit %06x interval %.2f ms CSA#%d",
	       p.access_address, p.crc_init, p.interval * 1.25, p.csa);
	if (p.csa == 1)
		printf(" hop %u", p.hop);
	else
		printf(" counter %u", p.counter);
	printf(" (%u/%u events explained%s)\n", q.matched, q.considered,
	       chm ? ", surveyed map" : "");
	st->started = 0;
	st->lost = 0;
	if (cmd_le_follow_params(ut->devh, &p) < 0) {
		printf("  [recover] firmware rejected the parameters\n");
		return 0;
	}
	st->phase = REC_FOLLOW;
	return 1;
}

static void recover_restart(ubertooth_t* ut, recover_state_t* st, uint8_t chan)
{
	le_recover_init(&st->rec, st->aa);
	st->solve_events = 0;
	memset(st->chan_events, 0, sizeof(st->chan_events));
	st->survey_done = 0;
	st->interval = 0;
	st->chan_switches = 0;
	st->phase = REC_MONITOR;
	st->phase_start = time(NULL);
	recover_monitor(ut, st, chan);
}

static int run_promisc_recover(ubertooth_t* ut, btle_options* opts, int aa_known,
                               uint32_t aa, int8_t min_rssi, unsigned budget_s,
                               unsigned start_mhz)
{
	recover_state_t st;
	int r;

	memset(&st, 0, sizeof(st));
	st.opts = opts;
	st.promisc_mhz = start_mhz;

	if (aa_known) {
		st.aa = aa;
		st.aa_found = 1;
	} else {
		printf("Searching for connections (min RSSI %d dBm)...\n", min_rssi);
		if (cmd_le_promisc_config(ut->devh, min_rssi,
				LE_PROMISC_STRONGEST | LE_PROMISC_AA_ONLY) < 0) {
			printf("Error: firmware does not support promiscuous recovery\n");
			return 1;
		}
		cmd_set_channel(ut->devh, st.promisc_mhz);
		ubertooth_set_le_timebase(ut, UBERTOOTH_TIMEBASE_CLK100NS);
		if (ubertooth_bulk_init(ut) < 0 || ubertooth_bulk_thread_start() < 0)
			return 1;
		r = cmd_btle_promisc(ut->devh);
		while (r >= 0 && !ut->stop_ubertooth && !st.aa_found)
			ubertooth_bulk_receive(ut, cb_recover, &st);
		ubertooth_bulk_thread_stop();
		cmd_stop(ut->devh);
		if (!st.aa_found)
			return ut->stop_ubertooth ? 0 : 1;
		ut->stop_ubertooth = 0;
	}
	printf("Access address %08x: recovering connection parameters\n", st.aa);

	// LE follow mode, parked on the channel the AA was seen on
	cmd_set_channel(ut->devh, 2402);
	ubertooth_set_le_timebase(ut, UBERTOOTH_TIMEBASE_T1);
	if (ubertooth_bulk_init(ut) < 0 || ubertooth_bulk_thread_start() < 0)
		return 1;
	if (cmd_btle_sniffing(ut->devh, 1) < 0)
		goto out;
	usleep(100000);
	recover_restart(ut, &st, mhz_to_chan_idx(st.promisc_mhz) < 37 ?
	                mhz_to_chan_idx(st.promisc_mhz) : 17);

	while (!ut->stop_ubertooth) {
		if (st.phase == REC_FOLLOW) {
			ubertooth_bulk_receive(ut, cb_btle, opts);
			// peek is not possible without consuming; watch for the
			// loss through a side flag set by cb_btle's state printing
			if (ubertooth_le_conn_lost(ut)) {
				printf("  [recover] connection lost, recovering again\n");
				recover_restart(ut, &st, st.monitor_chan);
			}
			continue;
		}

		ubertooth_bulk_receive(ut, cb_recover, &st);
		time_t now = time(NULL);

		if (st.phase == REC_MONITOR) {
			if (recover_solve_due(ut, &st) &&
					recover_try_solve(ut, &st, st.survey_done ? st.survey_chm : NULL))
				continue;
			if (!st.interval && st.rec.num_events >= 6)
				st.interval = le_recover_interval(&st.rec, 6);

			// the busiest channel so far
			unsigned c, busiest = st.monitor_chan;
			for (c = 0; c < 37; ++c)
				if (st.chan_events[c] > st.chan_events[busiest])
					busiest = c;

			// after enough events on one channel (CSA#2 usually solves
			// from one), visit another: it separates CSA#1 from CSA#2
			// and gives CSA#1 its hop increment
			if (st.interval && st.events_on_chan >= 24 && st.chan_switches < 8 &&
					now - st.chan_start >= 1) {
				uint8_t next = (st.monitor_chan + 7) % 37;
				if (st.survey_done) {
					// next used channel of the surveyed map
					for (c = 1; c <= 37; ++c) {
						next = (st.monitor_chan + c) % 37;
						if (st.survey_chm[next / 8] & (1 << (next % 8)))
							break;
					}
				}
				++st.chan_switches;
				recover_monitor(ut, &st, next);
			} else if (st.events_on_chan == 0 && now - st.chan_start >= 2) {
				// quiet channel: probably unused. Go back to a channel
				// with traffic if there is one, otherwise keep seeking.
				if (st.chan_events[busiest] > 0 && busiest != st.monitor_chan)
					recover_monitor(ut, &st, busiest);
				else
					recover_monitor(ut, &st, (st.monitor_chan + 1) % 37);
			}
			if (!st.survey_done && now - st.phase_start > (time_t)budget_s && st.interval) {
				// channel survey: dwell on each channel long enough for
				// a used channel to be hit (~37 events) and solve with the
				// surveyed map
				unsigned dwell_ms = st.interval * 125 * 37 * 3 / 200;
				printf("  [recover] not solved in %u s: surveying the channel map (%u ms per channel)\n",
				       budget_s, dwell_ms);
				st.phase = REC_SURVEY;
				memset(st.survey_seen, 0, sizeof(st.survey_seen));
				for (st.survey_chan = 0; st.survey_chan < 37 && !ut->stop_ubertooth; ++st.survey_chan) {
					cmd_le_monitor(ut->devh, chan_idx_to_mhz(st.survey_chan), st.aa);
					time_t t0 = time(NULL);
					while (!ut->stop_ubertooth && !st.survey_seen[st.survey_chan] &&
					       (time(NULL) - t0) * 1000 < (time_t)dwell_ms + 200)
						ubertooth_bulk_receive(ut, cb_recover, &st);
				}
				memset(st.survey_chm, 0, 5);
				for (unsigned c = 0; c < 37; ++c)
					if (st.survey_seen[c])
						st.survey_chm[c / 8] |= 1 << (c % 8);
				printf("  [recover] surveyed map %02x%02x%02x%02x%02x\n",
				       st.survey_chm[0], st.survey_chm[1], st.survey_chm[2],
				       st.survey_chm[3], st.survey_chm[4]);
				st.survey_done = 1;
				st.phase = REC_MONITOR;
				st.phase_start = time(NULL);
				if (recover_try_solve(ut, &st, st.survey_chm))
					continue;
				// keep collecting on a used channel
				for (unsigned c = 0; c < 37; ++c) {
					if (st.survey_seen[c]) {
						recover_monitor(ut, &st, c);
						break;
					}
				}
			}
		}
	}

out:
	ubertooth_bulk_thread_stop();
	cmd_stop(ut->devh);
	return 0;
}

static void usage(void)
{
	printf("ubertooth-btle - passive Bluetooth Low Energy monitoring\n");
	printf("Usage:\n");
	printf("\t-h this help\n");
	printf("\n");
	printf("    Major modes:\n");
	printf("\t-f follow connections\n");
	printf("\t-n don't follow, only print advertisements\n");
	printf("\t-p promiscuous: sniff active connections (firmware, CSA#1 only)\n");
	printf("\t-P promiscuous with host recovery: find and follow active\n");
	printf("\t   connections (CSA#1 and CSA#2); -a<aa> skips the AA search\n");
	printf("\t-m<dBm> -P: minimum RSSI of the connection to pick (default -90)\n");
	printf("\t-C<MHz> -P: data channel to search first (default 2440)\n");
	printf("\n");
	printf("\t-a[address] get/set access address (example: -a8e89bed6)\n");
	printf("\t-s<address> faux slave mode, using MAC addr (example: -s22:44:66:88:aa:cc)\n");
	printf("\t-t<address> set connection following target (example: -t22:44:66:88:aa:cc/48)\n");
	printf("\t-tnone unset connection following target\n");
	printf("\n");
	printf("    Advertising channels (use with -f or -n):\n");
	printf("\t-H[dwell_ms] hop between advertising channels 37/38/39\n");
	printf("\t-X follow AuxPtr to secondary advertising channels\n");
	printf("\n");
	printf("    Interference (use with -f or -p):\n");
	printf("\t-i interfere with one connection and return to idle\n");
	printf("\t-I interfere continuously\n");
	printf("\n");
	printf("    Data source:\n");
	printf("\t-U<0-7> set ubertooth device to use\n");
	printf("\n");
	printf("    Misc:\n");
	printf("\t-r<filename> capture packets to PCAPNG file\n");
	printf("\t-q<filename> capture packets to PCAP file (DLT_BLUETOOTH_LE_LL_WITH_PHDR)\n");
	printf("\t-c<filename> capture packets to PCAP file (DLT_PPI + DLT_BLUETOOTH_LE_LL)\n");
	printf("\t-d<filename> dump raw USB packets to file\n");
	printf("\t-A<index> advertising channel index (default 37)\n");
	printf("\t-v[01] verify CRC mode, get status or enable/disable\n");
	printf("\t-x<n> allow n access address offenses (default 32)\n");

	printf("\nIf an input file is not specified, an Ubertooth device is used for live capture.\n");
	printf("In get/set mode no capture occurs.\n");
	printf("Send SIGUSR1 to stop following the current connection.\n");
}

int main(int argc, char *argv[])
{
	int opt;
	int do_follow, do_no_follow, do_promisc, do_recover = 0;
	int min_rssi = -90;
	unsigned start_mhz = 2440;
	int do_get_aa, do_set_aa;
	int do_crc;
	int do_adv_index;
	int do_slave_mode;
	int do_target;
	uint16_t adv_flags = 0, adv_dwell = 0;
	enum jam_modes jam_mode = JAM_NONE;
	int ubertooth_device = -1;
	ubertooth_t* ut = ubertooth_init();
	struct sigaction sa;
	int ret = 0;

	btle_options cb_opts = { .allowed_access_address_errors = 32 };

	int r;
	u32 access_address = 0;
	uint8_t slave_mac[6] = { 0, };
	uint8_t target_mac[6] = { 0, };
	uint8_t slave_mask = 0, target_mask = 0;

	if (ut == NULL)
		return 1;

	do_follow = do_no_follow = do_promisc = 0;
	do_get_aa = do_set_aa = 0;
	do_crc = -1; // 0 and 1 mean set, 2 means get
	do_adv_index = 37;
	do_slave_mode = do_target = 0;

	while ((opt=getopt(argc,argv,"a::r:hfnpPm:C:U:v::A:s:t:x:c:q:d:jJiIH::X")) != EOF) {
		switch(opt) {
		case 'a':
			if (optarg == NULL) {
				do_get_aa = 1;
			} else {
				char *end;
				do_set_aa = 1;
				access_address = strtoul(optarg, &end, 16);
				if (end == optarg || *end != '\0') {
					printf("Error: invalid access address '%s'\n", optarg);
					return 1;
				}
			}
			break;
		case 'f':
			do_follow = 1;
			break;
		case 'n':
			do_no_follow = 1;
			break;
		case 'p':
			do_promisc = 1;
			break;
		case 'P':
			do_recover = 1;
			break;
		case 'C':
			start_mhz = atoi(optarg);
			if (start_mhz < 2404 || start_mhz > 2478 || (start_mhz & 1) ||
					start_mhz == 2426) {
				printf("Error: -C must be a BLE data channel frequency (MHz)\n");
				return 1;
			}
			break;
		case 'm':
			min_rssi = atoi(optarg);
			if (min_rssi < -128 || min_rssi > 0) {
				printf("Error: minimum RSSI must be -128..0 dBm\n");
				return 1;
			}
			break;
		case 'U':
			ubertooth_device = atoi(optarg);
			break;
		case 'r':
			if (!ut->h_pcapng_le) {
				if (lell_pcapng_create_file(optarg, "Ubertooth", &ut->h_pcapng_le)) {
					err(1, "lell_pcapng_create_file: ");
				}
			}
			else {
				printf("Ignoring extra capture file: %s\n", optarg);
			}
			break;
		case 'q':
			if (!ut->h_pcap_le) {
				if (lell_pcap_create_file(optarg, &ut->h_pcap_le)) {
					err(1, "lell_pcap_create_file: ");
				}
			}
			else {
				printf("Ignoring extra capture file: %s\n", optarg);
			}
			break;
		case 'c':
			if (!ut->h_pcap_le) {
				if (lell_pcap_ppi_create_file(optarg, 0, &ut->h_pcap_le)) {
					err(1, "lell_pcap_ppi_create_file: ");
				}
			}
			else {
				printf("Ignoring extra capture file: %s\n", optarg);
			}
			break;
		case 'd':
			dumpfile = fopen(optarg, "w");
			if (dumpfile == NULL) {
				perror(optarg);
				return 1;
			}
			break;
		case 'v':
			if (optarg)
				do_crc = atoi(optarg) ? 1 : 0;
			else
				do_crc = 2; // get
			break;
		case 'A':
			do_adv_index = atoi(optarg);
			if (do_adv_index < 37 || do_adv_index > 39) {
				printf("Error: advertising index must be 37, 38, or 39\n");
				usage();
				return 1;
			}
			break;
		case 's':
			do_slave_mode = 1;
			r = convert_mac_address(optarg, slave_mac, &slave_mask);
			if (!r) {
				usage();
				return 1;
			}
			break;
		case 't':
			do_target = 1;
			r = convert_mac_address(optarg, target_mac, &target_mask);
			if (!r) {
				usage();
				return 1;
			}
			break;
		case 'x':
			cb_opts.allowed_access_address_errors = (unsigned) atoi(optarg);
			if (cb_opts.allowed_access_address_errors > 32) {
				printf("Error: can tolerate 0-32 access address bit errors\n");
				usage();
				return 1;
			}
			break;
		case 'i':
		case 'j':
			jam_mode = JAM_ONCE;
			break;
		case 'I':
		case 'J':
			jam_mode = JAM_CONTINUOUS;
			break;
		case 'H':
			adv_flags |= LE_ADV_HOP;
			if (optarg) {
				int dwell = atoi(optarg);
				if (dwell < 1 || dwell > 65535) {
					printf("Error: dwell time must be 1-65535 ms\n");
					return 1;
				}
				adv_dwell = (uint16_t)dwell;
			}
			break;
		case 'X':
			adv_flags |= LE_ADV_FOLLOW_AUX;
			break;
		case 'h':
			usage();
			return 0;
		default:
			usage();
			return 1;
		}
	}

	if (do_follow + do_no_follow + do_promisc + do_recover > 1) {
		printf("Error: must choose one -f, -n, or -p, pick one pal\n");
		return 1;
	}
	if (adv_flags && !(do_follow || do_no_follow)) {
		printf("Error: -H and -X require -f or -n\n");
		return 1;
	}
	if (!(do_follow || do_no_follow || do_promisc || do_recover || do_get_aa || do_set_aa ||
	      do_crc >= 0 || do_slave_mode || do_target)) {
		usage();
		return 1;
	}

	r = ubertooth_connect(ut, ubertooth_device);
	if (r < 0) {
		usage();
		ubertooth_free(ut);
		return 1;
	}

	r = ubertooth_check_api(ut);
	if (r < 0) {
		ubertooth_free(ut);
		return 1;
	}

	// quit on ctrl-C
	register_cleanup_handler(ut, 0);

	// cancel following on USR1
	memset(&sa, 0, sizeof(sa));
	sa.sa_handler = cancel_follow_handler;
	sigemptyset(&sa.sa_mask);
	sigaction(SIGUSR1, &sa, NULL);

	// allow user to set target before following
	if (do_target) {
		r = cmd_btle_set_target(ut->devh, target_mac, target_mask);
		if (r == 0) {
			if (target_mask == 0) {
				printf("target cleared\n");
			} else {
				printf("target set to: ");
				print_mac(target_mac, target_mask);
				printf("\n");
			}
		} else {
			printf("Unable to set target\n");
			ret = 1;
			goto out;
		}
	}

	if (do_recover) {
		cmd_set_modulation(ut->devh, MOD_BT_LOW_ENERGY);
		ret = run_promisc_recover(ut, &cb_opts, do_set_aa, access_address,
		                          (int8_t)min_rssi, 30, start_mhz);
		do_set_aa = 0;
		goto out;
	}

	if (do_follow || do_no_follow || do_promisc) {
		r = cmd_set_jam_mode(ut->devh, jam_mode);
		if (jam_mode != JAM_NONE && r != 0) {
			printf("Jamming not supported\n");
			ret = 1;
			goto out;
		}
		cmd_set_modulation(ut->devh, MOD_BT_LOW_ENERGY);

		if (do_follow || do_no_follow) {
			u16 channel;
			if (do_adv_index == 37)
				channel = 2402;
			else if (do_adv_index == 38)
				channel = 2426;
			else
				channel = 2480;
			cmd_set_channel(ut->devh, channel);

			// always send the advertising configuration: the firmware
			// keeps it across sessions
			r = cmd_le_adv_config(ut->devh, adv_flags, adv_dwell);
			if (r < 0 && adv_flags) {
				printf("Error: firmware does not support advertising channel options (-H/-X)\n");
				ret = 1;
				goto out;
			}

			/* LE follow mode timestamps are raw 32-bit timer ticks */
			ubertooth_set_le_timebase(ut, UBERTOOTH_TIMEBASE_T1);
		} else {
			/* promiscuous mode timestamps are CLK100NS */
			ubertooth_set_le_timebase(ut, UBERTOOTH_TIMEBASE_CLK100NS);
		}

		if (do_promisc) {
			/* The firmware's legacy promiscuous loop answers control
			 * requests (from the USB interrupt) but does not service the
			 * bulk endpoint: poll for packets, as upstream does. */
			usb_pkt_rx rx;
			if (cmd_btle_promisc(ut->devh) < 0) {
				ret = 1;
				goto out;
			}
			while (!ut->stop_ubertooth) {
				r = cmd_poll(ut->devh, &rx);
				if (r < 0) {
					printf("USB error\n");
					ret = 1;
					break;
				}
				if (r == sizeof(usb_pkt_rx)) {
					fifo_push(ut->fifo, &rx);
					cb_btle(ut, &cb_opts);
				} else {
					usleep(500);
				}
			}
			cmd_stop(ut->devh);
			ut->stop_ubertooth = 0;
			if (ret)
				goto out;
			goto promisc_done;
		}

		r = ubertooth_bulk_init(ut);
		if (r < 0) {
			ret = 1;
			goto out;
		}
		r = ubertooth_bulk_thread_start();
		if (r < 0) {
			ret = 1;
			goto stream_out;
		}

		r = cmd_btle_sniffing(ut->devh, do_follow);
		if (r < 0) {
			ret = 1;
			goto stream_out;
		}

		// stop_ubertooth is set by the signal handlers or on USB errors
		while (!ut->stop_ubertooth) {
			if (cancel_follow) {
				cancel_follow = 0;
				cmd_cancel_follow(ut->devh);
			}
			ubertooth_bulk_receive(ut, cb_btle, &cb_opts);
		}

stream_out:
		ubertooth_bulk_thread_stop();
		cmd_stop(ut->devh);
		ut->stop_ubertooth = 0;
		if (ret)
			goto out;
	}
promisc_done:

	if (do_get_aa) {
		access_address = cmd_get_access_address(ut->devh);
		printf("Access address: %08x\n", access_address);
	}

	if (do_set_aa) {
		if (cmd_set_access_address(ut->devh, access_address) == 0)
			printf("access address set to: %08x\n", access_address);
		else
			ret = 1;
	}

	if (do_crc >= 0) {
		int crc;
		if (do_crc == 2) {
			crc = cmd_get_crc_verify(ut->devh);
		} else {
			crc = cmd_set_crc_verify(ut->devh, do_crc) < 0 ? -1 : do_crc;
		}
		if (crc < 0)
			ret = 1;
		else
			printf("CRC: %sverify\n", crc ? "" : "DO NOT ");
	}

	if (do_slave_mode) {
		u16 channel;
		if (do_adv_index == 37)
			channel = 2402;
		else if (do_adv_index == 38)
			channel = 2426;
		else
			channel = 2480;
		cmd_set_channel(ut->devh, channel);

		// flags: LE Limited Discovery
		uint8_t adv_data[] = { 0x02, 0x01, 0x05 };
		cmd_le_set_adv_data(ut->devh, adv_data, sizeof(adv_data));

		if (cmd_btle_slave(ut->devh, slave_mac) < 0)
			ret = 1;
	}

out:
	/* Close without sending STOP: streaming was already stopped above,
	 * and slave mode / get/set operations must keep the device state. */
	if (ut->devh) {
		libusb_release_interface(ut->devh, 0);
		libusb_close(ut->devh);
		ut->devh = NULL;
	}
	ubertooth_free(ut);
	if (dumpfile)
		fclose(dumpfile);
	return ret;
}
