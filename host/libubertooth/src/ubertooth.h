/*
 * Copyright 2010 - 2013 Michael Ossmann, Dominic Spill, Will Code, Mike Ryan
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

#ifndef __UBERTOOTH_H__
#define __UBERTOOTH_H__

#include <stdio.h>
#include "ubertooth_control.h"
#include "ubertooth_fifo.h"
#include <btbb.h>
#include <pthread.h>
#include <signal.h>

/* specan output types
 * see https://github.com/dkogan/feedgnuplot for plotter */
enum specan_modes {
	SPECAN_STDOUT         = 0,
	SPECAN_GNUPLOT_NORMAL = 1,
	SPECAN_GNUPLOT_3D     = 2,
	SPECAN_FILE           = 3
};

enum board_ids {
	BOARD_ID_UBERTOOTH_ZERO = 0,
	BOARD_ID_UBERTOOTH_ONE  = 1,
	BOARD_ID_TC13BADGE      = 2
};

/* Number of bulk IN transfers kept in flight while streaming. Each one
 * carries a single 64 byte usb_pkt_rx, so the device can deliver this
 * many packets before the host has to resubmit anything. */
#define UBERTOOTH_NUM_XFERS 16

/* Time base of the clk100ns field of LE packets (see
 * ubertooth_set_le_timebase()). Both count 100 ns ticks. */
enum ubertooth_timebase {
	/* raw 32-bit timer ticks, wrap at 2^32 (LE follow mode, le_phy) */
	UBERTOOTH_TIMEBASE_T1       = 0,
	/* CLK100NS = 3125 * CLKN[19:0] + T0TC, wraps at 3276800000
	 * (BR modes, LE promiscuous mode) */
	UBERTOOTH_TIMEBASE_CLK100NS = 1,
};

#define CLK100NS_WRAP 3276800000ULL

/* converts device timestamps into monotonically increasing host time */
typedef struct {
	int      started;
	uint32_t start;
	uint32_t last;
	uint64_t wraps;
	uint64_t abs_start_ns;
} ubertooth_ts_tracker;

typedef struct {
	/* Ringbuffer of received USB packets */
	fifo_t* fifo;

	struct libusb_device_handle* devh;

	/* Set to stop streaming. Written from signal handlers and the
	 * libusb event thread, read by the main loop. */
	volatile sig_atomic_t stop_ubertooth;
	/* set (together with stop_ubertooth) when the -t timeout expired */
	volatile sig_atomic_t timed_out;

	/* time base of LE_PACKET/LE_PACKET_CONT/LE_STATE timestamps */
	int le_timebase;
	ubertooth_ts_tracker ts_br;
	ubertooth_ts_tracker ts_le;

	btbb_pcap_handle* h_pcap_bredr;
	lell_pcap_handle* h_pcap_le;
	btbb_pcapng_handle* h_pcapng_bredr;
	lell_pcapng_handle* h_pcapng_le;

	/* --- private: bulk streaming state --- */
	int usb_initialized;
	pthread_mutex_t xfer_lock;
	struct libusb_transfer* rx_xfers[UBERTOOTH_NUM_XFERS];
	int rx_xfers_active;
	int rx_canceling;
	pthread_t poll_thread;
	int poll_thread_running;
	volatile int poll_exit;

	/* statistics, reported when streaming stops */
	unsigned long stat_packets;
	unsigned long stat_short;          /* keep-alives / short transfers */
	unsigned long stat_fw_queue_full;  /* status FIFO_OVERFLOW */
	unsigned long stat_fw_dma_overflow;
	unsigned long stat_fw_dma_error;
} ubertooth_t;

typedef void (*rx_callback)(ubertooth_t* ut, void* args);

typedef struct {
	unsigned allowed_access_address_errors;
} btle_options;

extern uint32_t systime;
extern FILE* infile;
extern FILE* dumpfile;
extern int max_ac_errors;

void print_version(void);
void register_cleanup_handler(ubertooth_t* ut, int do_exit);
ubertooth_t* ubertooth_init(void);
/* Number of attached Uberteeth, or -1 on error */
int ubertooth_count(void);
/* print every selectable device with its -U index; the number listed, or -1 */
int ubertooth_list_devices(FILE* out);
int ubertooth_connect(ubertooth_t* ut, int ubertooth_device);
ubertooth_t* ubertooth_start(int ubertooth_device);
/* Stop the device, close it and close capture files. Idempotent. */
void ubertooth_stop(ubertooth_t* ut);
/* ubertooth_stop() plus freeing ut */
void ubertooth_free(ubertooth_t* ut);
int ubertooth_get_api(ubertooth_t *ut, uint16_t *version);
int ubertooth_check_api(ubertooth_t *ut);
void ubertooth_set_timeout(ubertooth_t* ut, int seconds);
void ubertooth_set_le_timebase(ubertooth_t* ut, int timebase);

/* Convert a packet timestamp to host time (ns since the epoch). */
uint64_t ubertooth_packet_time_ns(ubertooth_t* ut, const usb_pkt_rx* rx);

/*
 * Bulk streaming:
 *   ubertooth_bulk_init(ut);          drain stale data, submit transfers
 *   ubertooth_bulk_thread_start();    start the libusb event thread
 *   <send the command that starts streaming>
 *   while (!ut->stop_ubertooth)
 *       ubertooth_bulk_receive(ut, cb, args);
 *   ubertooth_bulk_thread_stop();     cancel transfers, join the thread
 *
 * The thread functions operate on the device passed to the most recent
 * ubertooth_bulk_init().
 */
int ubertooth_bulk_init(ubertooth_t* ut);
void ubertooth_bulk_wait(ubertooth_t* ut);
/* Calls cb for one packet if available (returns 0), otherwise waits up
 * to 100 ms for one (returns -1). Returns 1 once stop_ubertooth is set. */
int ubertooth_bulk_receive(ubertooth_t* ut, rx_callback cb, void* cb_args);
int ubertooth_bulk_thread_start(void);
void ubertooth_bulk_thread_stop(void);

int stream_rx_file(ubertooth_t* ut,FILE* fp, rx_callback cb, void* cb_args);

void rx_dump(ubertooth_t* ut, int full);
void rx_btle_file(FILE* fp);
void rx_afh(ubertooth_t* ut, btbb_piconet* pn, int timeout);
void rx_afh_r(ubertooth_t* ut, btbb_piconet* pn, int timeout);

void ubertooth_unpack_symbols(const uint8_t* buf, char* unpacked);

#endif /* __UBERTOOTH_H__ */
