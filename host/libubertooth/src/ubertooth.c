/*
 * Copyright 2010, 2011 Michael Ossmann
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

#include <errno.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "ubertooth.h"
#ifdef HAVE_LIBHACKRF
#include "ubertooth_hackrf.h"
#endif
#include "ubertooth_callback.h"
#include "ubertooth_control.h"
#include "ubertooth_interface.h"

#ifndef RELEASE
#define RELEASE "unknown"
#endif
#ifndef VERSION
#define VERSION "unknown"
#endif

_Static_assert(sizeof(usb_pkt_rx) == PKT_LEN, "usb_pkt_rx must be one 64 byte USB packet");

uint32_t systime;
FILE* infile = NULL;
FILE* dumpfile = NULL;
/* Access code bit errors tolerated. With libbtbb's checked search,
 * promiscuous matches must also look like real packets, so 4 errors
 * find more packets without the random LAPs 2 already let through. */
#ifdef HAVE_BTBB_FIND_AC_CHECKED
int max_ac_errors = 4;
#else
int max_ac_errors = 2;
#endif

unsigned int packet_counter_max;

/* device used by the argument-less ubertooth_bulk_thread_*() calls */
static ubertooth_t* bulk_ut = NULL;

void print_version(void) {
	printf("libubertooth %s (%s), libbtbb %s (%s)\n", VERSION, RELEASE,
	       btbb_get_version(), btbb_get_release());
}

/*
 * Signal handlers only touch volatile sig_atomic_t flags (everything
 * else is not async-signal-safe). The main loop notices the flag within
 * one fifo_wait() timeout and shuts down normally.
 */
static ubertooth_t* volatile cleanup_devh = NULL;
static volatile sig_atomic_t cleanup_signals = 0;

static void cleanup(int sig __attribute__((unused)))
{
	if (cleanup_devh)
		cleanup_devh->stop_ubertooth = 1;
}

static void cleanup_exit(int sig __attribute__((unused)))
{
	/* first signal: ask the main loop to stop, second: give up */
	if (cleanup_signals++ || !cleanup_devh) {
#ifdef HAVE_LIBHACKRF
		ubertooth_hackrf_emergency_restore();
#endif
		_exit(1);
	}
	cleanup_devh->stop_ubertooth = 1;
}

static void set_handler(int sig, void (*handler)(int))
{
	struct sigaction sa;

	memset(&sa, 0, sizeof(sa));
	sa.sa_handler = handler;
	sigemptyset(&sa.sa_mask);
	sigaction(sig, &sa, NULL);
}

void register_cleanup_handler(ubertooth_t* ut, int do_exit) {
	void (*handler)(int) = do_exit ? cleanup_exit : cleanup;

	cleanup_devh = ut;
	cleanup_signals = 0;

	set_handler(SIGINT, handler);
	set_handler(SIGQUIT, handler);
	set_handler(SIGTERM, handler);
}

static ubertooth_t* volatile timeout_dev = NULL;
static void stop_transfers(int sig __attribute__((unused))) {
	if (timeout_dev) {
		timeout_dev->timed_out = 1;
		timeout_dev->stop_ubertooth = 1;
	}
}

void ubertooth_set_timeout(ubertooth_t* ut, int seconds) {
	timeout_dev = ut;
	set_handler(SIGALRM, stop_transfers);
	alarm(seconds);
}

void ubertooth_set_le_timebase(ubertooth_t* ut, int timebase)
{
	ut->le_timebase = timebase;
	memset(&ut->ts_le, 0, sizeof(ut->ts_le));
}

static int is_ubertooth(const struct libusb_device_descriptor* desc)
{
	return (desc->idVendor == TC13_VENDORID && desc->idProduct == TC13_PRODUCTID)
	    || (desc->idVendor == U0_VENDORID && desc->idProduct == U0_PRODUCTID)
	    || (desc->idVendor == U1_VENDORID && desc->idProduct == U1_PRODUCTID);
}

/* Ubertooths only (libusb initialized by the caller) */
static int __attribute__((unused)) count_uberteeth(void)
{
	struct libusb_device **usb_list = NULL;
	struct libusb_device_descriptor desc;
	ssize_t usb_devs, i;
	int n = 0;

	usb_devs = libusb_get_device_list(NULL, &usb_list);
	if (usb_devs < 0)
		return 0;
	for (i = 0; i < usb_devs; ++i)
		if (libusb_get_device_descriptor(usb_list[i], &desc) == 0 && is_ubertooth(&desc))
			n++;
	libusb_free_device_list(usb_list, 1);
	return n;
}

int ubertooth_count(void) {
	struct libusb_device **usb_list = NULL;
	struct libusb_device_descriptor desc;
	ssize_t usb_devs, i;
	int r, uberteeth = 0;

	r = libusb_init(NULL);
	if (r < 0) {
		fprintf(stderr, "libusb_init failed (got 1.0?)\n");
		return -1;
	}

	usb_devs = libusb_get_device_list(NULL, &usb_list);
	if (usb_devs < 0) {
		show_libusb_error((int)usb_devs);
		libusb_exit(NULL);
		return -1;
	}

	for (i = 0; i < usb_devs; ++i) {
		if (libusb_get_device_descriptor(usb_list[i], &desc) < 0) {
			fprintf(stderr, "couldn't get usb descriptor for dev #%zd!\n", i);
			continue;
		}
		if (is_ubertooth(&desc))
			uberteeth++;
	}

	libusb_free_device_list(usb_list, 1);
	libusb_exit(NULL);
#ifdef HAVE_LIBHACKRF
	uberteeth += ubertooth_hackrf_count();
#endif
	return uberteeth;
}

/*
 * Every device the tools can select, with its -U index: the Ubertooths in
 * USB order, then qualifying HackRF Pros (listing loads no gateware).
 * Returns the number listed, or -1.
 */
int ubertooth_list_devices(FILE* out)
{
	struct libusb_device **usb_list = NULL;
	struct libusb_device_descriptor desc;
	ssize_t usb_devs, i;
	int n = 0;

	if (libusb_init(NULL) < 0)
		return -1;
	usb_devs = libusb_get_device_list(NULL, &usb_list);
	if (usb_devs < 0) {
		libusb_exit(NULL);
		return -1;
	}
	for (i = 0; i < usb_devs; ++i) {
		struct libusb_device_handle* devh;
		uint8_t serial[17];
		char rev[64] = "unknown", sn[40] = "unknown";
		int j;

		if (libusb_get_device_descriptor(usb_list[i], &desc) < 0 || !is_ubertooth(&desc))
			continue;
		if (libusb_open(usb_list[i], &devh) == 0) {
			if (cmd_get_serial(devh, serial) == 0)
				for (j = 0; j < 16; j++)
					sprintf(sn + 2 * j, "%02x", serial[1 + j]);
			cmd_get_rev_num(devh, rev, sizeof(rev));
			libusb_close(devh);
			fprintf(out, "%d: %-14s serial %s  firmware %s\n", n,
			        desc.idProduct == U1_PRODUCTID ? "Ubertooth One" : "Ubertooth Zero",
			        sn, rev);
		} else {
			fprintf(out, "%d: %-14s (cannot open: in use, or no permission)\n", n,
			        desc.idProduct == U1_PRODUCTID ? "Ubertooth One" : "Ubertooth Zero");
		}
		n++;
	}
	libusb_free_device_list(usb_list, 1);
	libusb_exit(NULL);
#ifdef HAVE_LIBHACKRF
	{
		int pros = ubertooth_hackrf_count(), k;
		for (k = 0; k < pros; k++) {
			char line[160];
			if (ubertooth_hackrf_describe(k, line, sizeof(line)) == 0)
				fprintf(out, "%d: %s\n", n + k, line);
		}
		n += pros;
	}
#endif
	if (n == 0)
		fprintf(out, "no Ubertooth or HackRF Pro found\n");
	return n;
}

static struct libusb_device_handle* find_ubertooth_device(int ubertooth_device, int pros)
{
	struct libusb_device **usb_list = NULL;
	struct libusb_device_handle *devh = NULL;
	struct libusb_device_descriptor desc;
	ssize_t usb_devs, i;
	int ret, uberteeth = 0;
	libusb_device **ubertooth_devs = NULL;

	usb_devs = libusb_get_device_list(NULL, &usb_list);
	if (usb_devs < 0) {
		show_libusb_error((int)usb_devs);
		return NULL;
	}
	ubertooth_devs = calloc(usb_devs > 0 ? (size_t)usb_devs : 1, sizeof(*ubertooth_devs));
	if (ubertooth_devs == NULL) {
		libusb_free_device_list(usb_list, 1);
		return NULL;
	}

	for (i = 0; i < usb_devs; ++i) {
		if (libusb_get_device_descriptor(usb_list[i], &desc) < 0) {
			fprintf(stderr, "couldn't get usb descriptor for dev #%zd!\n", i);
			continue;
		}
		if (is_ubertooth(&desc))
			ubertooth_devs[uberteeth++] = usb_list[i];
	}

	if (uberteeth == 0) {
		if (pros > 0 && ubertooth_device < 0)
			fprintf(stderr, "no Ubertooth found; a HackRF Pro is available as device 0 (-U 0)\n");
	} else if (ubertooth_device >= uberteeth) {
		fprintf(stderr, "Ubertooth device %d not found (%d device%s present; "
		        "ubertooth-util -L lists them)\n",
		        ubertooth_device, uberteeth + pros, uberteeth + pros == 1 ? "" : "s");
	} else if (uberteeth == 1 || ubertooth_device >= 0) {
		int idx = ubertooth_device >= 0 ? ubertooth_device : 0;
		ret = libusb_open(ubertooth_devs[idx], &devh);
		if (ret) {
			show_libusb_error(ret);
			devh = NULL;
		}
	} else {
		uint8_t serial[17];
		fprintf(stderr, "multiple devices found: select one with -U "
		        "(ubertooth-util -L lists them)\n");
		(void)pros;
		for (i = 0; i < uberteeth; ++i) {
			ret = libusb_open(ubertooth_devs[i], &devh);
			if (ret) {
				fprintf(stderr, "  Device %zd: ", i);
				show_libusb_error(ret);
			} else {
				if (cmd_get_serial(devh, serial) == 0) {
					fprintf(stderr, "  Device %zd: ", i);
					print_serial(serial, stderr);
				}
				libusb_close(devh);
			}
		}
#ifdef HAVE_LIBHACKRF
		for (i = 0; i < pros; ++i) {
			char desc[128];
			if (ubertooth_hackrf_describe((int)i, desc, sizeof(desc)) == 0)
				fprintf(stderr, "  Device %zd: %s\n", uberteeth + i, desc);
		}
#endif
		devh = NULL;
	}

	free(ubertooth_devs);
	libusb_free_device_list(usb_list, 1);
	return devh;
}

/*
 * based on http://libusb.sourceforge.net/api-1.0/group__asyncio.html#ga9fcb2aa23d342060ebda1d0cf7478856
 */
static void rx_xfer_status(int status)
{
	fprintf(stderr, "rx_xfer status: %s (%d)\n",
	        ubertooth_transfer_status_str(status), status);
}

/* Forget a transfer that is no longer in flight and free it. */
static void release_xfer(ubertooth_t* ut, struct libusb_transfer* xfer)
{
	int i;

	pthread_mutex_lock(&ut->xfer_lock);
	for (i = 0; i < UBERTOOTH_NUM_XFERS; i++) {
		if (ut->rx_xfers[i] == xfer) {
			ut->rx_xfers[i] = NULL;
			ut->rx_xfers_active--;
			break;
		}
	}
	pthread_mutex_unlock(&ut->xfer_lock);

	/* LIBUSB_TRANSFER_FREE_BUFFER is set, this frees the buffer too */
	libusb_free_transfer(xfer);
}

static void handle_rx_packet(ubertooth_t* ut, const uint8_t* buf)
{
	const usb_pkt_rx* pkt = (const usb_pkt_rx*)buf;

	ut->stat_packets++;
	if (pkt->status & FIFO_OVERFLOW)
		ut->stat_fw_queue_full++;
	if (pkt->status & DMA_OVERFLOW)
		ut->stat_fw_dma_overflow++;
	if (pkt->status & DMA_ERROR)
		ut->stat_fw_dma_error++;

	memcpy(fifo_get_write_element(ut->fifo), buf, PKT_LEN);
	fifo_inc_write_ptr(ut->fifo);
}

/* runs in the libusb event thread */
static void cb_xfer(struct libusb_transfer *xfer)
{
	ubertooth_t* ut = (ubertooth_t*)xfer->user_data;
	int resubmit = 0;
	int r;

	switch (xfer->status) {
	case LIBUSB_TRANSFER_COMPLETED:
		/* The firmware sends 1 byte KEEP_ALIVE packets when idle;
		 * only complete packets are queued. */
		if (xfer->actual_length == PKT_LEN)
			handle_rx_packet(ut, xfer->buffer);
		else
			ut->stat_short++;
		resubmit = 1;
		break;
	case LIBUSB_TRANSFER_TIMED_OUT:
		resubmit = 1;
		break;
	case LIBUSB_TRANSFER_CANCELLED:
		break;
	default:
		rx_xfer_status(xfer->status);
		ut->stop_ubertooth = 1;
		break;
	}

	if (resubmit && !ut->stop_ubertooth && !ut->rx_canceling) {
		r = libusb_submit_transfer(xfer);
		if (r == 0)
			return;
		fprintf(stderr, "Failed to submit USB transfer (%d)\n", r);
		ut->stop_ubertooth = 1;
	}

	release_xfer(ut, xfer);
	/* let the consumer notice stop_ubertooth right away */
	fifo_wake(ut->fifo);
}

static void* poll_thread_main(void* arg)
{
	ubertooth_t* ut = (ubertooth_t*)arg;
	int r;

	while (!ut->poll_exit) {
		struct timeval tv = { 0, 100000 };
		r = libusb_handle_events_timeout_completed(NULL, &tv, NULL);
		if (r < 0 && r != LIBUSB_ERROR_INTERRUPTED) {
			show_libusb_error(r);
			ut->stop_ubertooth = 1;
			fifo_wake(ut->fifo);
			break;
		}
	}

	return NULL;
}

int ubertooth_bulk_thread_start(void)
{
	ubertooth_t* ut = bulk_ut;
	sigset_t all, old;
	int r;

	if (ut == NULL) {
		fprintf(stderr, "ubertooth_bulk_thread_start: call ubertooth_bulk_init first\n");
		return -1;
	}
#ifdef HAVE_LIBHACKRF
	/* libhackrf runs its own thread; streaming starts with cmd_rx_syms() */
	if (ut->devh == NULL && ubertooth_hackrf_active())
		return 0;
#endif
	if (ut->poll_thread_running)
		return 0;

	ut->poll_exit = 0;

	/* deliver signals to the main thread only */
	sigfillset(&all);
	pthread_sigmask(SIG_BLOCK, &all, &old);
	r = pthread_create(&ut->poll_thread, NULL, poll_thread_main, ut);
	pthread_sigmask(SIG_SETMASK, &old, NULL);
	if (r != 0) {
		fprintf(stderr, "pthread_create: %s\n", strerror(r));
		return -1;
	}
	ut->poll_thread_running = 1;
	return 0;
}

static uint64_t monotonic_ms(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (uint64_t)ts.tv_sec * 1000 + (uint64_t)ts.tv_nsec / 1000000;
}

static void report_stats(ubertooth_t* ut)
{
	if (ut->stat_fw_queue_full)
		fprintf(stderr, "Ubertooth reported %lu firmware queue overflow(s): packets were lost on the device\n",
		        ut->stat_fw_queue_full);
	if (ut->stat_fw_dma_overflow || ut->stat_fw_dma_error)
		fprintf(stderr, "Ubertooth reported %lu DMA overflow(s), %lu DMA error(s)\n",
		        ut->stat_fw_dma_overflow, ut->stat_fw_dma_error);
	if (ut->fifo && ut->fifo->overflows)
		fprintf(stderr, "%lu packets dropped: host processing fell behind\n",
		        ut->fifo->overflows);

	ut->stat_fw_queue_full = 0;
	ut->stat_fw_dma_overflow = 0;
	ut->stat_fw_dma_error = 0;
	if (ut->fifo)
		ut->fifo->overflows = 0;
}

/* Cancel all transfers and wait (bounded) until libusb returned them. */
static void cancel_transfers(ubertooth_t* ut)
{
	uint64_t deadline;
	int i, active;

	pthread_mutex_lock(&ut->xfer_lock);
	ut->rx_canceling = 1;
	for (i = 0; i < UBERTOOTH_NUM_XFERS; i++)
		if (ut->rx_xfers[i])
			libusb_cancel_transfer(ut->rx_xfers[i]);
	active = ut->rx_xfers_active;
	pthread_mutex_unlock(&ut->xfer_lock);

	deadline = monotonic_ms() + 2000;
	while (active > 0 && monotonic_ms() < deadline) {
		if (ut->poll_thread_running) {
			struct timespec ts = { 0, 1000000 };
			nanosleep(&ts, NULL);
		} else {
			struct timeval tv = { 0, 10000 };
			libusb_handle_events_timeout_completed(NULL, &tv, NULL);
		}
		pthread_mutex_lock(&ut->xfer_lock);
		active = ut->rx_xfers_active;
		pthread_mutex_unlock(&ut->xfer_lock);
	}
	if (active > 0)
		fprintf(stderr, "%d USB transfer(s) did not complete cancellation\n", active);

	ut->rx_canceling = 0;
}

void ubertooth_bulk_thread_stop(void)
{
	ubertooth_t* ut = bulk_ut;

	if (ut == NULL)
		return;
#ifdef HAVE_LIBHACKRF
	if (ut->devh == NULL && ubertooth_hackrf_active()) {
		ubertooth_hackrf_stream_stop();
		report_stats(ut);
		return;
	}
#endif

	cancel_transfers(ut);

	if (ut->poll_thread_running) {
		ut->poll_exit = 1;
		pthread_join(ut->poll_thread, NULL);
		ut->poll_thread_running = 0;
	}

	report_stats(ut);
}

/*
 * Read and discard whatever the device still has buffered from an
 * earlier session (the bulk IN endpoint is double buffered and is not
 * flushed by the firmware on mode changes).
 */
static void bulk_flush(ubertooth_t* ut)
{
	uint8_t buf[PKT_LEN];
	int i, r, transferred;

	for (i = 0; i < 64; i++) {
		r = libusb_bulk_transfer(ut->devh, DATA_IN, buf, sizeof(buf),
		                         &transferred, 10);
		if (r != 0)
			break;
	}
}

int ubertooth_bulk_init(ubertooth_t* ut)
{
	int i, r, submitted = 0;

#ifdef HAVE_LIBHACKRF
	if (ut->devh == NULL && ubertooth_hackrf_active() && ut->fifo != NULL) {
		bulk_ut = ut;
		fifo_reset(ut->fifo);
		return 0;
	}
#endif

	if (ut->devh == NULL || ut->fifo == NULL)
		return -1;
	if (ut->rx_xfers_active) {
		fprintf(stderr, "ubertooth_bulk_init: transfers already active\n");
		return -1;
	}

	bulk_ut = ut;
	bulk_flush(ut);
	fifo_reset(ut->fifo);
	ut->rx_canceling = 0;

	for (i = 0; i < UBERTOOTH_NUM_XFERS; i++) {
		struct libusb_transfer* xfer = libusb_alloc_transfer(0);
		uint8_t* buf = malloc(PKT_LEN);

		if (xfer == NULL || buf == NULL) {
			libusb_free_transfer(xfer);
			free(buf);
			break;
		}
		libusb_fill_bulk_transfer(xfer, ut->devh, DATA_IN, buf, PKT_LEN,
		                          cb_xfer, ut, TIMEOUT);
		xfer->flags = LIBUSB_TRANSFER_FREE_BUFFER;

		pthread_mutex_lock(&ut->xfer_lock);
		r = libusb_submit_transfer(xfer);
		if (r == 0) {
			ut->rx_xfers[i] = xfer;
			ut->rx_xfers_active++;
			submitted++;
		}
		pthread_mutex_unlock(&ut->xfer_lock);

		if (r < 0) {
			fprintf(stderr, "rx_xfer submission: %s\n", libusb_strerror(r));
			libusb_free_transfer(xfer);
			break;
		}
	}

	if (submitted == 0)
		return -1;
	return 0;
}

void ubertooth_bulk_wait(ubertooth_t* ut)
{
	while (!ut->stop_ubertooth && !fifo_wait(ut->fifo, 100))
		;
}

int ubertooth_bulk_receive(ubertooth_t* ut, rx_callback cb, void* cb_args)
{
	if (!fifo_empty(ut->fifo)) {
		(*cb)(ut, cb_args);
		return ut->stop_ubertooth ? 1 : 0;
	}
	if (ut->stop_ubertooth)
		return 1;
	fifo_wait(ut->fifo, 100);
	return -1;
}

static int stream_rx_usb(ubertooth_t* ut, rx_callback cb, void* cb_args)
{
	// init USB transfer
	int r = ubertooth_bulk_init(ut);
	if (r < 0)
		return r;

	r = ubertooth_bulk_thread_start();
	if (r < 0) {
		ubertooth_bulk_thread_stop();
		return r;
	}

	// tell ubertooth to send packets
	r = cmd_rx_syms(ut->devh);
	if (r < 0) {
		ubertooth_bulk_thread_stop();
		return r;
	}

	// receive and process each packet
	while (!ut->stop_ubertooth)
		ubertooth_bulk_receive(ut, cb, cb_args);

	ubertooth_bulk_thread_stop();

	return 1;
}

/* file should be in full USB packet format (ubertooth-dump -f) */
int stream_rx_file(ubertooth_t* ut, FILE* fp, rx_callback cb, void* cb_args)
{
	uint8_t buf[PKT_LEN];
	size_t nitems;

	while (!ut->stop_ubertooth) {
		uint32_t systime_be;
		nitems = fread(&systime_be, sizeof(systime_be), 1, fp);
		if (nitems != 1)
			break;
		systime = be32toh(systime_be);

		nitems = fread(buf, sizeof(buf[0]), PKT_LEN, fp);
		if (nitems != PKT_LEN)
			break;
		fifo_push(ut->fifo, (usb_pkt_rx*)buf);
		(*cb)(ut, cb_args);
	}
	if (ferror(fp)) {
		perror("read");
		return -1;
	}
	return 0;
}

void rx_afh(ubertooth_t* ut, btbb_piconet* pn, int timeout)
{
	int r = btbb_init(max_ac_errors);
	if (r < 0)
		return;

	cmd_set_channel(ut->devh, 9999);

	if (timeout) {
		ubertooth_set_timeout(ut, timeout);

		cmd_afh(ut->devh);
		stream_rx_usb(ut, cb_afh_initial, pn);

		cmd_stop(ut->devh);

		/* stopped by the user (or an error) rather than the timeout */
		if (!ut->timed_out)
			return;
		ut->timed_out = 0;
		ut->stop_ubertooth = 0;

		btbb_print_afh_map(pn);
	}

	/*
	 * Monitor changes in AFH channel map
	 */
	cmd_clear_afh_map(ut->devh);
	cmd_afh(ut->devh);
	stream_rx_usb(ut, cb_afh_monitor, pn);
}

void rx_afh_r(ubertooth_t* ut, btbb_piconet* pn, int timeout __attribute__((unused)))
{
	static uint32_t lasttime;

	int r = btbb_init(max_ac_errors);
	int i, j;
	if (r < 0)
		return;

	cmd_set_channel(ut->devh, 9999);

	cmd_afh(ut->devh);

	// init USB transfer
	r = ubertooth_bulk_init(ut);
	if (r < 0)
		return;

	r = ubertooth_bulk_thread_start();
	if (r < 0) {
		ubertooth_bulk_thread_stop();
		return;
	}

	// tell ubertooth to send packets
	r = cmd_rx_syms(ut->devh);
	if (r < 0) {
		ubertooth_bulk_thread_stop();
		return;
	}

	// receive and process each packet
	while (!ut->stop_ubertooth) {
		ubertooth_bulk_receive(ut, cb_afh_r, pn);
		if (lasttime < time(NULL)) {
			lasttime = time(NULL);
			printf("%u ", (uint32_t)time(NULL));
			// btbb_print_afh_map(pn);

			uint8_t* afh_map = btbb_piconet_get_afh_map(pn);
			for (i = 0; i < 10; i++)
				for (j = 0; j < 8; j++)
					if (afh_map[i] & (1 << j))
						printf("1");
					else
						printf("0");
			printf("\n");
		}
	}

	ubertooth_bulk_thread_stop();
}

void rx_btle_file(FILE* fp)
{
	ubertooth_t* ut = ubertooth_init();
	if (ut == NULL)
		return;

	/* keep the file's timestamps instead of the current time */
	infile = fp;
	stream_rx_file(ut, fp, cb_btle, NULL);
	ubertooth_free(ut);
}

/* one byte per symbol (0x00 or 0x01), most significant bit first */
void ubertooth_unpack_symbols(const uint8_t* buf, char* unpacked)
{
	static uint8_t table[256][8];
	static int ready = 0;
	int i, j;

	if (!ready) {
		for (i = 0; i < 256; i++)
			for (j = 0; j < 8; j++)
				table[i][j] = (i >> (7 - j)) & 1;
		ready = 1;
	}
	for (i = 0; i < SYM_LEN; i++)
		memcpy(&unpacked[i * 8], table[buf[i]], 8);
}

static void cb_dump_bitstream(ubertooth_t* ut, void* args __attribute__((unused)))
{
	int i;
	char nl = '\n';
	FILE* out = dumpfile ? dumpfile : stdout;

	usb_pkt_rx usb = fifo_pop(ut->fifo);
	usb_pkt_rx* rx = &usb;
	char bitstream[BANK_LEN];

	if (rx->pkt_type != BR_PACKET && rx->pkt_type != LE_PACKET)
		return;

	ubertooth_unpack_symbols((uint8_t*)rx->data, bitstream);

	// convert to ascii
	for (i = 0; i < BANK_LEN; ++i)
		bitstream[i] += 0x30;

	if (fwrite(bitstream, sizeof(uint8_t), BANK_LEN, out) != BANK_LEN ||
	    fwrite(&nl, sizeof(uint8_t), 1, out) != 1) {
		perror("write");
		ut->stop_ubertooth = 1;
	}
}

static void cb_dump_full(ubertooth_t* ut, void* args __attribute__((unused)))
{
	FILE* out = dumpfile ? dumpfile : stdout;
	usb_pkt_rx usb = fifo_pop(ut->fifo);
	usb_pkt_rx* rx = &usb;
	uint32_t time_be = htobe32((uint32_t)time(NULL));

	if (fwrite(&time_be, 1, sizeof(time_be), out) != sizeof(time_be) ||
	    fwrite((uint8_t*)rx, sizeof(uint8_t), PKT_LEN, out) != PKT_LEN) {
		perror("write");
		ut->stop_ubertooth = 1;
	}
}

/* dump received symbols to stdout */
void rx_dump(ubertooth_t* ut, int bitstream)
{
	if (bitstream)
		stream_rx_usb(ut, cb_dump_bitstream, NULL);
	else
		stream_rx_usb(ut, cb_dump_full, NULL);
}

static void close_captures(ubertooth_t* ut)
{
	if (ut->h_pcap_bredr) {
		btbb_pcap_close(ut->h_pcap_bredr);
		ut->h_pcap_bredr = NULL;
	}
	if (ut->h_pcap_le) {
		lell_pcap_close(ut->h_pcap_le);
		ut->h_pcap_le = NULL;
	}

	if (ut->h_pcapng_bredr) {
		btbb_pcapng_close(ut->h_pcapng_bredr);
		ut->h_pcapng_bredr = NULL;
	}
	if (ut->h_pcapng_le) {
		lell_pcapng_close(ut->h_pcapng_le);
		ut->h_pcapng_le = NULL;
	}
}

void ubertooth_stop(ubertooth_t* ut)
{
	if (ut == NULL)
		return;

	/* make sure no transfers are active */
	if (bulk_ut == ut && (ut->rx_xfers_active || ut->poll_thread_running))
		ubertooth_bulk_thread_stop();

#ifdef HAVE_LIBHACKRF
	if (ut->devh == NULL && ubertooth_hackrf_active())
		ubertooth_hackrf_close(ut);
#endif
	if (ut->devh != NULL) {
		cmd_stop(ut->devh);
		libusb_release_interface(ut->devh, 0);
		libusb_close(ut->devh);
		ut->devh = NULL;
	}
	if (ut->usb_initialized) {
		libusb_exit(NULL);
		ut->usb_initialized = 0;
	}

	close_captures(ut);
}

void ubertooth_free(ubertooth_t* ut)
{
	if (ut == NULL)
		return;
	ubertooth_stop(ut);
	if (bulk_ut == ut)
		bulk_ut = NULL;
	if (cleanup_devh == ut)
		cleanup_devh = NULL;
	if (timeout_dev == ut)
		timeout_dev = NULL;
	pthread_mutex_destroy(&ut->xfer_lock);
	fifo_free(ut->fifo);
	free(ut);
}

ubertooth_t* ubertooth_init(void)
{
	ubertooth_t* ut = (ubertooth_t*)calloc(1, sizeof(ubertooth_t));
	if (ut == NULL) {
		fprintf(stderr, "Unable to allocate memory\n");
		return NULL;
	}

	ut->fifo = fifo_init();
	if (ut->fifo == NULL) {
		fprintf(stderr, "Unable to initialize ringbuffer\n");
		free(ut);
		return NULL;
	}

	pthread_mutex_init(&ut->xfer_lock, NULL);
	ut->le_timebase = UBERTOOTH_TIMEBASE_T1;

	return ut;
}

int ubertooth_connect(ubertooth_t* ut, int ubertooth_device)
{
	int r;

	if (ut == NULL)
		return -1;

	r = libusb_init(NULL);
	if (r < 0) {
		fprintf(stderr, "libusb_init failed (got 1.0?)\n");
		return -1;
	}
	ut->usb_initialized = 1;

	int pros = 0;
#ifdef HAVE_LIBHACKRF
	/* HackRF Pros are listed after the Ubertooths, used only when selected */
	pros = ubertooth_hackrf_count();
	if (pros > 0 && ubertooth_device >= 0) {
		int n = ubertooth_device - count_uberteeth();
		if (n >= 0 && n < pros) {
			libusb_exit(NULL);
			ut->usb_initialized = 0;
			return ubertooth_hackrf_open(ut, n) == 0 ? 1 : -1;
		}
	}
#endif

	ut->devh = find_ubertooth_device(ubertooth_device, pros);
	if (ut->devh == NULL) {
		fprintf(stderr, "could not open Ubertooth device\n");
		ubertooth_stop(ut);
		return -1;
	}

	r = libusb_claim_interface(ut->devh, 0);
	if (r < 0) {
		fprintf(stderr, "usb_claim_interface error %d\n", r);
		/* don't send STOP to a device we could not claim */
		libusb_close(ut->devh);
		ut->devh = NULL;
		ubertooth_stop(ut);
		return -1;
	}

	return 1;
}

ubertooth_t* ubertooth_start(int ubertooth_device)
{
	ubertooth_t* ut = ubertooth_init();
	if (ut == NULL)
		return NULL;

	int r = ubertooth_connect(ut, ubertooth_device);
	if (r < 0) {
		ubertooth_free(ut);
		return NULL;
	}

	return ut;
}

int ubertooth_get_api(ubertooth_t *ut, uint16_t *version) {
	int result;
	libusb_device* dev;
	struct libusb_device_descriptor desc;

#ifdef HAVE_LIBHACKRF
	if (ut != NULL && ut->devh == NULL && ubertooth_hackrf_active()) {
		*version = UBERTOOTH_API_VERSION;
		return 0;
	}
#endif
	if (ut == NULL || ut->devh == NULL)
		return LIBUSB_ERROR_NO_DEVICE;

	dev = libusb_get_device(ut->devh);
	result = libusb_get_device_descriptor(dev, &desc);
	if (result < 0) {
		if (result == LIBUSB_ERROR_PIPE) {
			fprintf(stderr, "control message unsupported\n");
		} else {
			show_libusb_error(result);
		}
		return result;
	}
	*version = desc.bcdDevice;
	return 0;
}

int ubertooth_check_api(ubertooth_t *ut) {
	uint16_t version;
	int result;
	result = ubertooth_get_api(ut, &version);
	if (result < 0) {
		return result;
	}

	if (version < UBERTOOTH_API_VERSION) {
		fprintf(stderr, "Ubertooth API version %x.%02x found, libubertooth %s requires %x.%02x.\n",
				(version>>8)&0xFF, version&0xFF, VERSION,
				(UBERTOOTH_API_VERSION>>8)&0xFF, UBERTOOTH_API_VERSION&0xFF);
		fprintf(stderr, "Please upgrade to latest released firmware.\n");
		fprintf(stderr, "See: https://github.com/greatscottgadgets/ubertooth/wiki/Firmware\n");
		ubertooth_stop(ut);
		return -1;
	}
	else if (version > UBERTOOTH_API_VERSION) {
		fprintf(stderr, "Ubertooth API version %x.%02x found, newer than that supported by libubertooth (%x.%02x).\n",
				(version>>8)&0xFF, version&0xFF,
				(UBERTOOTH_API_VERSION>>8)&0xFF, UBERTOOTH_API_VERSION&0xFF);
		fprintf(stderr, "Things will still work, but you might want to update your host tools.\n");
	}
	return 0;
}
