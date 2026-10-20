/*
 * HackRF Pro backend: a virtual Ubertooth for Bluetooth BR (see ubertooth_hackrf.h).
 *
 * The BR channelizer gateware streams one 16-bit word per symbol at 16 Msps
 * (1 M words/s): bit k is the decision for the channel at center + k MHz
 * (k >= 9 meaning k - 16), bit 8 marks the first word of each 400-symbol
 * bank. Each bank becomes one usb_pkt_rx per channel inside 2402-2480 MHz.
 *
 * Retunes (a new channel, or scanning) are timed against the word stream:
 * the capture time of word i is estimated as off + i us, with off the least
 * (arrival time - word count) seen so far, and a retune requested at host
 * time t lands about 0.7 ms later. Banks within 2 ms of a landing are
 * dropped; later banks carry the new center.
 *
 * LE (ubertooth-btle -n or -f): the same decisions on the window's even
 * channels are LE 1M bits. ubertooth_hackrf_le.c searches them for
 * advertising packets and for the packets of connections whose CONNECT_IND
 * it saw, and each becomes an LE_PACKET (plus LE_PACKET_CONT) as the
 * firmware sends in follow mode. The window does not hop: connections are
 * captured on its data channels, not followed.
 *
 * Device handling: a HackRF Pro qualifies when its firmware has the RAM
 * gateware loader (vendor requests 70-79 of the praline-ram-fpga and
 * praline-apps branches) and the gateware pack is installed. Selecting it
 * loads the pack into the FPGA from RAM (nothing is written to flash);
 * closing it, exiting, or a fatal signal switches back to the stock image
 * in flash.
 *
 * This file is part of Project Ubertooth (GPL-2.0-or-later).
 */
#include <endian.h>
#include <errno.h>
#include <libhackrf/hackrf.h>
#include <libusb-1.0/libusb.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "ubertooth.h"
#include "ubertooth_hackrf.h"
#include "ubertooth_hackrf_le.h"

#define BR_KIND            0x4C     /* gateware identity kind of the BR image */
#define REG_MODE           6        /* 0x80: BR words; bits 4-6 leak margin, 0-3 threshold */
#define REG_LE_THR         4        /* bit 0: LE's slower decision threshold (images that
                                       have it; others leave the register unused) */
#define SAMPLE_RATE        16000000
#define BANK               400
#define RETUNE_LAND_NS     700000LL /* measured on a HackRF Pro: 0.65-0.81 ms after the request */
#define RETUNE_GUARD       2000     /* words dropped either side of a landing */
#define MAX_EVENTS         64
#define LE_CENTER_MIN      2409     /* LE windows stay inside the band: 2402-2416 */
#define LE_CENTER_MAX      2473     /* ... and 2466-2480 */
#define LE_AA_ERRORS       2        /* access address bits that may differ */

/* USB: HackRF IDs and the vendor requests used here */
#define HACKRF_VID         0x1d50
#define HACKRF_PID         0x6089
#define BOARD_PRALINE      5
#define REQ_TRANSCEIVER    1        /* set transceiver mode; 0 = off */
#define REQ_BOARD_ID       14
#define REQ_FPGA_BITSTREAM 54       /* switch to image n of the flash pack */
#define REQ_STAGE_BEGIN    70
#define REQ_STAGE_WRITE    71
#define REQ_STAGE_LOAD     72
#define REQ_STAGE_ABORT    73
#define REQ_STAGE_STATUS   74
#define REQ_APP_STOP       78
#define REQ_APP_STATUS     79
#define STAGE_STATUS_LEN   44
#define PACK_MAX           (128 * 1024)
#ifndef UBERTOOTH_HACKRF_PACK_DEFAULT
#define UBERTOOTH_HACKRF_PACK_DEFAULT "/usr/local/share/ubertooth/br_rx.pack"
#endif

static const int scan_centers[] = { 2409, 2424, 2439, 2454, 2469, 2477 };
#define N_SCAN ((int)(sizeof(scan_centers) / sizeof(scan_centers[0])))

typedef struct {
	uint64_t word;          /* estimated word index of the landing */
	int center;
} retune_event;

static struct {
	int open;
	hackrf_device* dev;
	ubertooth_t* ut;
	int center;             /* current (requested) center, MHz */
	int lna, vga;
	int scan_ms, afh;
	int want_scan;          /* sweep requested (channel 9999, AFH mode, or SCAN_MS) */
	uint8_t mode;           /* REG_MODE value while streaming */
	int le;                 /* LE mode: the even channels' bits searched for LE packets */
	int le_aa_errors;
	int le_correct;         /* correct one bit error from the CRC */
	int le_center;          /* LE window center (MHz) instead of the tool's channel, or 0 */
	int le_thr;             /* LE mode: the gateware's slower decision threshold */

	volatile int streaming;
	int scanning;
	volatile int scan_run;
	pthread_t scan_thread;
	pthread_mutex_t lock;

	/* stream state, callback thread */
	uint64_t words;
	uint16_t bank[BANK];
	int nbank;
	int synced;
	uint64_t bank_start;
	int have_carry;
	uint8_t carry;

	/* time map and retunes, under lock */
	int64_t off_ns;
	int have_off;
	int base_center;        /* center before the oldest kept event */
	retune_event ev[MAX_EVENTS];
	int nev;

	unsigned long banks, banks_dropped;

	/* the board's EP0 through libusb (its own context), for the loader */
	libusb_context* uctx;
	libusb_device_handle* usb;
	volatile sig_atomic_t loaded;   /* our image is in the FPGA: restore at exit */
	int ram_firmware;               /* a RAM build ("-ramfpga"): no flash images */
} H = { .lock = PTHREAD_MUTEX_INITIALIZER };

static int64_t now_ns(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (int64_t)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

static const char* pack_path(void)
{
	const char* p = getenv("UBERTOOTH_HACKRF_PACK");
	return p && *p ? p : UBERTOOTH_HACKRF_PACK_DEFAULT;
}

static int ctrl(libusb_device_handle* h, int in, uint8_t req, uint32_t arg,
                uint8_t* data, uint16_t len, unsigned ms)
{
	uint8_t type = (in ? LIBUSB_ENDPOINT_IN : LIBUSB_ENDPOINT_OUT) |
	               LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE;
	return libusb_control_transfer(h, type, req, (uint16_t)(arg >> 16),
	                               (uint16_t)(arg & 0xffff), data, len, ms);
}

/* A HackRF Pro whose firmware has the RAM loader: its board ID and stage
 * status answer over EP0, which a board streaming for another program
 * still does. */
static int qualifies(libusb_device* dev, libusb_device_handle** out, char* serial, size_t slen)
{
	struct libusb_device_descriptor d;
	libusb_device_handle* h;
	uint8_t id = 0, st[STAGE_STATUS_LEN];
	int ok;

	if (libusb_get_device_descriptor(dev, &d) != 0 ||
	    d.idVendor != HACKRF_VID || d.idProduct != HACKRF_PID ||
	    libusb_open(dev, &h) != 0)
		return 0;
	ok = ctrl(h, 1, REQ_BOARD_ID, 0, &id, 1, 500) == 1 && id == BOARD_PRALINE &&
	     ctrl(h, 1, REQ_STAGE_STATUS, 0, st, sizeof(st), 500) == (int)sizeof(st);
	if (ok && serial) {
		serial[0] = '\0';
		if (d.iSerialNumber)
			libusb_get_string_descriptor_ascii(h, d.iSerialNumber,
			                                   (unsigned char*)serial, (int)slen);
	}
	if (ok && out)
		*out = h;
	else
		libusb_close(h);
	return ok;
}

/* the n-th qualifying board (n < 0: count them); its handle, if asked */
static int find_pro(libusb_context* ctx, int n, libusb_device_handle** out,
                    uint8_t* bus, uint8_t* addr, char* serial, size_t slen)
{
	libusb_device** list;
	ssize_t cnt = libusb_get_device_list(ctx, &list), i;
	int found = 0, r = -1;

	if (cnt < 0)
		return n < 0 ? 0 : -1;
	for (i = 0; i < cnt; i++) {
		libusb_device_handle* h = NULL;
		if (!qualifies(list[i], found == n ? &h : NULL,
		               found == n ? serial : NULL, slen))
			continue;
		if (found == n) {
			if (bus)
				*bus = libusb_get_bus_number(list[i]);
			if (addr)
				*addr = libusb_get_device_address(list[i]);
			if (out)
				*out = h;
			else
				libusb_close(h);
			r = 0;
		}
		found++;
	}
	libusb_free_device_list(list, 1);
	return n < 0 ? found : r;
}

int ubertooth_hackrf_count(void)
{
	libusb_context* ctx;
	FILE* f = fopen(pack_path(), "rb");
	int n;

	if (f == NULL)
		return 0;       /* no gateware to load: not usable */
	fclose(f);
	if (libusb_init(&ctx) != 0)
		return 0;
	n = find_pro(ctx, -1, NULL, NULL, NULL, NULL, 0);
	libusb_exit(ctx);
	return n;
}

int ubertooth_hackrf_describe(int n, char* buf, size_t len)
{
	libusb_context* ctx;
	char serial[64] = "";
	int r;

	if (libusb_init(&ctx) != 0)
		return -1;
	r = find_pro(ctx, n, NULL, NULL, NULL, serial, sizeof(serial));
	libusb_exit(ctx);
	if (r == 0)
		snprintf(buf, len, "HackRF Pro     serial %s  BR channelizer, 15 channels (BR, LE 1M)",
		         serial[0] ? serial : "unknown");
	return r;
}

static uint32_t crc32_le(const uint8_t* data, size_t len)
{
	uint32_t r = 0xffffffff;
	size_t i;
	int b;
	for (i = 0; i < len; i++) {
		r ^= data[i];
		for (b = 0; b < 8; b++)
			r = (r >> 1) ^ ((r & 1) ? 0xedb88320 : 0);
	}
	return r ^ 0xffffffff;
}

/* Any running app stopped, then the pack staged and loaded into the FPGA. */
static int load_pack(libusb_device_handle* h)
{
	static uint8_t buf[PACK_MAX];
	const char* path = pack_path();
	FILE* f = fopen(path, "rb");
	size_t len = f ? fread(buf, 1, sizeof(buf), f) : 0, off;
	uint8_t st[STAGE_STATUS_LEN], result = 0;
	int i;

	if (f)
		fclose(f);
	if (len == 0 || len == sizeof(buf)) {
		fprintf(stderr, "HackRF: cannot read the gateware pack %s\n", path);
		return -1;
	}
	if (ctrl(h, 1, REQ_APP_STATUS, 0, st, sizeof(st), 500) == (int)sizeof(st) && st[0] == 1) {
		ctrl(h, 0, REQ_APP_STOP, 0, NULL, 0, 1000);
		for (i = 0; i < 100; i++) {
			if (ctrl(h, 1, REQ_APP_STATUS, 0, st, sizeof(st), 500) == (int)sizeof(st) &&
			    st[0] != 1)
				break;
			usleep(10000);
		}
	}
	/* A receiver left running (a program killed outright) blocks the
	 * loader. If no other program holds the board, turn it off. */
	if (ctrl(h, 1, REQ_STAGE_STATUS, 0, st, sizeof(st), 500) == (int)sizeof(st) && !st[3]) {
		if (libusb_claim_interface(h, 0) != 0) {
			fprintf(stderr, "HackRF: the board is in use by another program\n");
			return -1;
		}
		ctrl(h, 0, REQ_TRANSCEIVER, 0, NULL, 0, 500);
		libusb_release_interface(h, 0);
		for (i = 0; i < 50; i++) {
			if (ctrl(h, 1, REQ_STAGE_STATUS, 0, st, sizeof(st), 500) == (int)sizeof(st) && st[3])
				break;
			usleep(10000);
		}
	}
	if (ctrl(h, 0, REQ_STAGE_BEGIN, (uint32_t)len, NULL, 0, 1000) != 0) {
		fprintf(stderr, "HackRF: the gateware loader refused the image (board busy)\n");
		return -1;
	}
	for (off = 0; off < len; off += 4096) {
		uint16_t n = (uint16_t)(len - off < 4096 ? len - off : 4096);
		if (ctrl(h, 0, REQ_STAGE_WRITE, (uint32_t)off, buf + off, n, 1000) != n) {
			fprintf(stderr, "HackRF: gateware upload failed at byte %zu\n", off);
			ctrl(h, 0, REQ_STAGE_ABORT, 0, NULL, 0, 1000);
			return -1;
		}
	}
	if (ctrl(h, 1, REQ_STAGE_LOAD, crc32_le(buf, len), &result, 1, 3000) != 1 || result != 1) {
		fprintf(stderr, "HackRF: gateware load failed (result %u)\n", result);
		ctrl(h, 0, REQ_STAGE_ABORT, 0, NULL, 0, 1000);
		return -1;
	}
	return 0;
}

/*
 * Back to the stock image in flash. Safe to call from a signal handler in
 * the sense that matters here: it only sends two control requests on the
 * loader's own libusb context, and runs once.
 */
static void restore_stock(void)
{
	if (!H.loaded || H.usb == NULL)
		return;
	H.loaded = 0;
	ctrl(H.usb, 0, REQ_TRANSCEIVER, 0, NULL, 0, 500);
	if (H.ram_firmware) {
		fprintf(stderr, "HackRF: this firmware runs from RAM and has no stock gateware in "
		        "flash; reset the board to restore it\n");
		return;
	}
	if (ctrl(H.usb, 0, REQ_FPGA_BITSTREAM, 0, NULL, 0, 2000) != 0)
		fprintf(stderr, "HackRF: could not switch back to the stock gateware "
		        "(run: hackrf_debug -P 0)\n");
}

void ubertooth_hackrf_emergency_restore(void)
{
	restore_stock();
}

static void restore_at_exit(void)
{
	restore_stock();
}

static void restore_on_signal(int sig)
{
	restore_stock();
	signal(sig, SIG_DFL);
	raise(sig);
}

/* Restore on any exit: normal exit, a crash, or a terminating signal the
 * program does not handle itself. */
static void install_restore_handlers(void)
{
	static int installed = 0;
	static const int fatal[] = { SIGSEGV, SIGBUS, SIGFPE, SIGILL, SIGABRT };
	static const int term[] = { SIGINT, SIGTERM, SIGHUP, SIGQUIT };
	struct sigaction sa, old;
	size_t i;

	if (installed)
		return;
	installed = 1;
	atexit(restore_at_exit);
	memset(&sa, 0, sizeof(sa));
	sa.sa_handler = restore_on_signal;
	sigemptyset(&sa.sa_mask);
	for (i = 0; i < sizeof(fatal) / sizeof(fatal[0]); i++)
		sigaction(fatal[i], &sa, NULL);
	for (i = 0; i < sizeof(term) / sizeof(term[0]); i++)
		if (sigaction(term[i], NULL, &old) == 0 && old.sa_handler == SIG_DFL)
			sigaction(term[i], &sa, NULL);
}

int ubertooth_hackrf_active(void)
{
	return H.open;
}

/* An LE packet as the firmware sends it in follow mode: T1 time base (100 ns
 * ticks, wrapping at 2^32), flags in clkn_high, AA and PDU split into
 * LE_PACKET + LE_PACKET_CONT records. No RSSI: the words carry decisions
 * only. */
static void emit_le(void* user, const hle_packet* p)
{
	uint8_t flags = LE_FLAG_CRC_CHECKED | LE_FLAG_CRC_OK;
	/* the firmware stamps the preamble's start, 8 symbols before the access
	 * address */
	uint32_t ts = (uint32_t)((p->word >= 8 ? p->word - 8 : 0) * 10);
	unsigned off, chunk, i;
	(void)user;

	if (p->is_data)
		flags |= LE_FLAG_DATA | (p->slave ? LE_FLAG_SLAVE : 0);
	if (p->aux)
		flags |= LE_FLAG_AUX;
	if (p->len > DMA_SIZE)
		flags |= LE_FLAG_TRUNCATED;
	for (off = 0, chunk = 0; off < p->len; off += DMA_SIZE, chunk++) {
		usb_pkt_rx f;
		memset(&f, 0, sizeof(f));
		f.pkt_type = chunk == 0 ? LE_PACKET : LE_PACKET_CONT;
		f.clkn_high = chunk == 0 ? flags : (uint8_t)chunk;
		f.channel = (uint8_t)(p->freq - 2402);
		f.clk100ns = htole32(ts);
		for (i = 0; i < DMA_SIZE && off + i < p->len; i++)
			f.data[i] = p->data[off + i];
		fifo_push(H.ut->fifo, &f);
	}
}

static void emit_bank(void)
{
	uint64_t start = H.bank_start, end = start + BANK;
	int center, drop = 0, i, k, s;

	pthread_mutex_lock(&H.lock);
	center = H.base_center;
	for (i = 0; i < H.nev; i++) {
		retune_event* e = &H.ev[i];
		if (end + RETUNE_GUARD > e->word && start < e->word + RETUNE_GUARD)
			drop = 1;
		if (e->word <= start)
			center = e->center;
	}
	/* forget events far behind the stream */
	while (H.nev > 0 && H.ev[0].word + 10 * RETUNE_GUARD < start) {
		H.base_center = H.ev[0].center;
		memmove(&H.ev[0], &H.ev[1], (H.nev - 1) * sizeof(H.ev[0]));
		H.nev--;
	}
	pthread_mutex_unlock(&H.lock);

	H.banks++;
	if (drop) {
		H.banks_dropped++;
		return;
	}
	if (H.le) {
		hle_bank(H.bank, BANK, start, center);
		return;
	}

	/* the Ubertooth's clocks: 100 ns ticks wrapping with CLKN (312.5 us) at 2^20 */
	uint64_t t100 = start * 10;
	uint32_t clk100ns = (uint32_t)(t100 % CLK100NS_WRAP);
	uint64_t clkn = t100 / 3125;

	usb_pkt_rx pkt[16];
	memset(pkt, 0, sizeof(pkt));
	for (s = 0; s < BANK; s++) {
		uint16_t w = H.bank[s];
		for (k = 0; k < 16; k++)
			if (w & (1u << k))
				pkt[k].data[s >> 3] |= 0x80 >> (s & 7);
	}
	for (k = 0; k < 16; k++) {
		int off = k < 8 ? k : k - 16;
		int ch = center + off - 2402;
		if (k == 8 || ch < 0 || ch >= NUM_BREDR_CHANNELS)
			continue;
		pkt[k].pkt_type = BR_PACKET;
		pkt[k].status = 0;
		pkt[k].channel = (uint8_t)ch;
		pkt[k].clkn_high = (uint8_t)((clkn >> 20) & 0xff);
		pkt[k].clk100ns = htole32(clk100ns);
		fifo_push(H.ut->fifo, &pkt[k]);
	}
}

static void handle_word(uint16_t w)
{
	if (w & 0x100) {
		H.synced = 1;
		H.nbank = 0;
		H.bank_start = H.words;
	}
	H.words++;
	if (!H.synced)
		return;
	H.bank[H.nbank++] = w;
	if (H.nbank == BANK) {
		emit_bank();
		/* the next word must carry the marker again */
		H.synced = 0;
	}
}

static int hackrf_rx_callback(hackrf_transfer* t)
{
	int64_t now = now_ns();
	int i = 0, n = t->valid_length;

	if (H.have_carry && n > 0) {
		handle_word((uint16_t)(H.carry | (t->buffer[0] << 8)));
		H.have_carry = 0;
		i = 1;
	}
	for (; i + 1 < n; i += 2)
		handle_word((uint16_t)(t->buffer[i] | (t->buffer[i + 1] << 8)));
	if (i < n) {
		H.carry = t->buffer[i];
		H.have_carry = 1;
	}

	/* capture time of word i ~ off + i us: keep the least latency seen */
	int64_t off = now - (int64_t)H.words * 1000;
	pthread_mutex_lock(&H.lock);
	if (!H.have_off || off < H.off_ns) {
		H.off_ns = off;
		H.have_off = 1;
	}
	pthread_mutex_unlock(&H.lock);

	return H.streaming ? 0 : -1;
}

static void retune(int center)
{
	int64_t t = now_ns();
	int r = hackrf_set_freq(H.dev, (uint64_t)center * 1000000ULL);

	if (r != HACKRF_SUCCESS) {
		fprintf(stderr, "hackrf_set_freq(%d MHz): %s\n", center, hackrf_error_name(r));
		return;
	}
	pthread_mutex_lock(&H.lock);
	H.center = center;
	if (!H.streaming) {
		H.base_center = center;
		H.nev = 0;
	} else {
		uint64_t w = H.have_off ? (uint64_t)((t + RETUNE_LAND_NS - H.off_ns) / 1000) : H.words;
		if (H.nev == MAX_EVENTS) {
			H.base_center = H.ev[0].center;
			memmove(&H.ev[0], &H.ev[1], (MAX_EVENTS - 1) * sizeof(H.ev[0]));
			H.nev--;
		}
		H.ev[H.nev].word = w;
		H.ev[H.nev].center = center;
		H.nev++;
	}
	pthread_mutex_unlock(&H.lock);
}

static void* scan_main(void* arg);

static void scan_start(void)
{
	if (!H.streaming || H.scanning)
		return;
	if (H.scan_ms <= 0)
		H.scan_ms = 100;
	H.scan_run = 1;
	if (pthread_create(&H.scan_thread, NULL, scan_main, NULL) == 0)
		H.scanning = 1;
}

static void scan_stop(void)
{
	if (!H.scanning)
		return;
	H.scan_run = 0;
	pthread_join(H.scan_thread, NULL);
	H.scanning = 0;
}

static void* scan_main(void* arg)
{
	int i = 0;
	(void)arg;
	while (H.scan_run) {
		struct timespec ts = { H.scan_ms / 1000, (long)(H.scan_ms % 1000) * 1000000L };
		retune(scan_centers[i]);
		i = (i + 1) % N_SCAN;
		while (nanosleep(&ts, &ts) == -1 && errno == EINTR && H.scan_run)
			;
	}
	return NULL;
}

int ubertooth_hackrf_open(void* arg, int n)
{
	ubertooth_t* ut = (ubertooth_t*)arg;
	hackrf_device_list_t* list;
	uint8_t bus = 0, addr = 0;
	const char* gain = getenv("UBERTOOTH_HACKRF_GAIN");
	const char* scan = getenv("UBERTOOTH_HACKRF_SCAN_MS");
	const char* leak = getenv("UBERTOOTH_HACKRF_LEAK");
	uint8_t id[4] = { 0 };
	int r, i;

	if (H.open) {
		fprintf(stderr, "HackRF backend already open\n");
		return -1;
	}
	if (libusb_init(&H.uctx) != 0)
		return -1;
	if (find_pro(H.uctx, n, &H.usb, &bus, &addr, NULL, 0) != 0) {
		fprintf(stderr, "HackRF Pro %d not found\n", n);
		goto fail_usb;
	}
	install_restore_handlers();
	if (load_pack(H.usb) != 0)
		goto fail_usb;
	H.loaded = 1;

	/* the same board through libhackrf, by its bus and address */
	r = hackrf_init();
	list = r == HACKRF_SUCCESS ? hackrf_device_list() : NULL;
	r = HACKRF_ERROR_NOT_FOUND;
	for (i = 0; list && i < list->devicecount; i++) {
		libusb_device* u = (libusb_device*)list->usb_devices[list->usb_device_index[i]];
		if (libusb_get_bus_number(u) == bus && libusb_get_device_address(u) == addr)
			r = hackrf_device_list_open(list, i, &H.dev);
	}
	if (list)
		hackrf_device_list_free(list);
	if (r != HACKRF_SUCCESS) {
		fprintf(stderr, "HackRF: could not open the board: %s\n", hackrf_error_name(r));
		hackrf_exit();
		goto fail_usb;
	}
	{
		char version[256] = "";
		H.ram_firmware = hackrf_version_string_read(H.dev, version, sizeof(version) - 1) ==
		                 HACKRF_SUCCESS && strstr(version, "-ramfpga") != NULL;
	}
	for (i = 0; i < 4; i++)
		hackrf_fpga_read_register(H.dev, (uint8_t)(0x78 + i), &id[i]);
	if (id[0] != 'G' || id[1] != 'W' || id[3] != BR_KIND) {
		fprintf(stderr,
		        "HackRF: the loaded gateware is not the BR channelizer (kind 0x%02x)\n",
		        id[0] == 'G' && id[1] == 'W' ? id[3] : 0);
		goto fail_dev;
	}

	H.lna = 16;
	H.vga = 16;
	/* leak suppression margin in 6 dB steps, 1-6; 0 turns it off (the
	 * gateware's code 7); unset leaves the gateware default (3) */
	H.mode = 0x80;
	if (leak) {
		int de = atoi(leak);
		H.mode |= (de <= 0 || de > 6 ? 7 : de) << 4;
	}
	if (gain && sscanf(gain, "%d,%d", &H.lna, &H.vga) != 2) {
		fprintf(stderr, "UBERTOOTH_HACKRF_GAIN must be \"lna,vga\"\n");
		goto fail_dev;
	}
	H.scan_ms = scan ? atoi(scan) : 0;
	H.want_scan = H.scan_ms > 0;
	H.afh = 0;
	H.le = 0;
	H.le_aa_errors = getenv("UBERTOOTH_HACKRF_LE_AA_ERRORS") ?
	                 atoi(getenv("UBERTOOTH_HACKRF_LE_AA_ERRORS")) : LE_AA_ERRORS;
	H.le_correct = getenv("UBERTOOTH_HACKRF_LE_CORRECT") ?
	               atoi(getenv("UBERTOOTH_HACKRF_LE_CORRECT")) : 1;
	H.le_thr = getenv("UBERTOOTH_HACKRF_LE_THRESHOLD") ?
	           atoi(getenv("UBERTOOTH_HACKRF_LE_THRESHOLD")) : 1;
	H.le_center = getenv("UBERTOOTH_HACKRF_LE_CENTER") ?
	              atoi(getenv("UBERTOOTH_HACKRF_LE_CENTER")) : 0;
	if (H.le_center && (H.le_center < 2402 || H.le_center > 2480)) {
		fprintf(stderr, "UBERTOOTH_HACKRF_LE_CENTER must be 2402-2480 (MHz)\n");
		goto fail_dev;
	}
	H.ut = ut;
	H.center = H.base_center = 2441;
	if (getenv("UBERTOOTH_HACKRF_CENTER")) {
		int c = atoi(getenv("UBERTOOTH_HACKRF_CENTER"));
		if (c >= 2402 && c <= 2480)
			H.center = H.base_center = c;
	}
	H.nev = 0;

	r = hackrf_set_sample_rate(H.dev, SAMPLE_RATE);
	if (r == HACKRF_SUCCESS)
		r = hackrf_set_baseband_filter_bandwidth(H.dev,
			hackrf_compute_baseband_filter_bw((uint32_t)(SAMPLE_RATE * 0.75)));
	if (r == HACKRF_SUCCESS)
		r = hackrf_set_freq(H.dev, (uint64_t)H.center * 1000000ULL);
	if (r == HACKRF_SUCCESS)
		r = hackrf_set_amp_enable(H.dev, 0);
	if (r == HACKRF_SUCCESS)
		r = hackrf_set_lna_gain(H.dev, H.lna);
	if (r == HACKRF_SUCCESS)
		r = hackrf_set_vga_gain(H.dev, H.vga);
	if (r != HACKRF_SUCCESS) {
		fprintf(stderr, "HackRF configuration failed: %s\n", hackrf_error_name(r));
		goto fail_dev;
	}
	H.open = 1;
	fprintf(stderr, "HackRF: BR channelizer loaded, 15 channels at %d MHz, LNA %d dB, VGA %d dB\n",
	        H.center, H.lna, H.vga);
	return 0;

fail_dev:
	hackrf_close(H.dev);
	H.dev = NULL;
	hackrf_exit();
fail_usb:
	restore_stock();
	if (H.usb)
		libusb_close(H.usb);
	H.usb = NULL;
	libusb_exit(H.uctx);
	H.uctx = NULL;
	return -1;
}

int ubertooth_hackrf_stream_start(void)
{
	int r;

	if (!H.open)
		return -1;
	if (H.streaming)
		return 0;

	H.words = 0;
	H.nbank = 0;
	H.synced = 0;
	H.have_carry = 0;
	H.have_off = 0;
	H.banks = H.banks_dropped = 0;
	pthread_mutex_lock(&H.lock);
	H.base_center = H.center;
	H.nev = 0;
	pthread_mutex_unlock(&H.lock);
	if (H.le)
		hle_init(emit_le, NULL, H.le_aa_errors, H.le_correct);

	/* the threshold follows the data within a packet. A legacy
	 * advertisement's CRC covers up to 312 bits, and one wrong bit fails the
	 * packet: LE takes the slower threshold */
	r = hackrf_fpga_write_register(H.dev, REG_LE_THR, H.le && H.le_thr ? 1 : 0);
	if (r == HACKRF_SUCCESS)
		r = hackrf_fpga_write_register(H.dev, REG_MODE, H.mode);
	if (r != HACKRF_SUCCESS) {
		fprintf(stderr, "could not select BR mode: %s\n", hackrf_error_name(r));
		return -1;
	}
	H.streaming = 1;
	r = hackrf_start_rx(H.dev, hackrf_rx_callback, NULL);
	if (r != HACKRF_SUCCESS) {
		fprintf(stderr, "hackrf_start_rx: %s\n", hackrf_error_name(r));
		H.streaming = 0;
		return -1;
	}

	if (H.want_scan)
		scan_start();
	return 0;
}

void ubertooth_hackrf_stream_stop(void)
{
	if (!H.open || !H.streaming)
		return;
	scan_stop();
	H.streaming = 0;
	hackrf_stop_rx(H.dev);
	if (H.banks_dropped)
		fprintf(stderr, "HackRF: %lu of %lu banks (%.1f%%) discarded at retunes\n",
		        H.banks_dropped, H.banks, 100.0 * H.banks_dropped / (H.banks ? H.banks : 1));
	if (H.le) {
		hle_stats s;
		hle_get_stats(&s);
		fprintf(stderr, "HackRF LE: %lu access address matches, %lu packets with a good CRC "
		        "(%lu of them corrected; %lu/%lu/%lu/%lu with 0/1/2/3 access address errors), "
		        "%lu with a bad CRC, %lu connections taken up\n",
		        s.matches, s.crc_ok, s.corrected, s.aa_err_hist[0], s.aa_err_hist[1],
		        s.aa_err_hist[2], s.aa_err_hist[3], s.crc_bad, s.connections);
	}
}

void ubertooth_hackrf_close(void* arg)
{
	(void)arg;
	if (!H.open)
		return;
	ubertooth_hackrf_stream_stop();
	hackrf_fpga_write_register(H.dev, REG_MODE, 0);
	hackrf_fpga_write_register(H.dev, REG_LE_THR, 0);
	hackrf_close(H.dev);
	H.dev = NULL;
	hackrf_exit();
	H.open = 0;
	/* back to the stock gateware */
	restore_stock();
	if (H.usb)
		libusb_close(H.usb);
	H.usb = NULL;
	libusb_exit(H.uctx);
	H.uctx = NULL;
}

/* BR or LE. A change while streaming restarts the stream, which sets up the
 * search and the threshold for the new mode. */
static int set_le(int le)
{
	if (H.le == le)
		return 0;
	H.le = le;
	if (!H.streaming)
		return 0;
	ubertooth_hackrf_stream_stop();
	return ubertooth_hackrf_stream_start() == 0 ? 0 : LIBUSB_ERROR_IO;
}

/* A mode the HackRF Pro cannot do: say so once, and stop the tool's
 * receive loop (not every tool checks the result) */
static int refuse_mode(void)
{
	if (!H.ut->stop_ubertooth)
		fprintf(stderr, "HackRF backend: the HackRF Pro receives Bluetooth BR, and LE 1M "
		        "advertising and connections in its window (ubertooth-btle -n or -f); "
		        "this mode needs an Ubertooth\n");
	H.ut->stop_ubertooth = 1;
	return LIBUSB_ERROR_NOT_SUPPORTED;
}

int ubertooth_hackrf_control(uint8_t request_type, uint8_t request,
                             uint16_t value, uint16_t index,
                             unsigned char* data, uint16_t length)
{
	static const char rev[] = "hackrf-br";
	(void)request_type;
	(void)index;

	switch (request) {
	case UBERTOOTH_PING:
		return 0;
	case UBERTOOTH_JAM_MODE:
		return value == JAM_NONE ? 0 : LIBUSB_ERROR_NOT_SUPPORTED;
	case UBERTOOTH_RX_SYMBOLS:
		return ubertooth_hackrf_stream_start() == 0 ? 0 : LIBUSB_ERROR_IO;
	case UBERTOOTH_STOP:
		ubertooth_hackrf_stream_stop();
		return 0;
	case UBERTOOTH_SET_CHANNEL:
		/* a channel fixes the window's center there; an out-of-band value
		 * (9999: "sweep all channels") scans the band like the Ubertooth's
		 * sweep, the 15-channel window stepping through it */
		if (value >= 2402 && value <= 2480) {
			H.want_scan = 0;
			scan_stop();
			/* LE: -A 37 and -A 39 put the window at the band's edge with the
			 * advertising channel in its outer bin; -A 38 centers it on
			 * 2425 MHz. An odd center keeps the tuner's +-4 MHz bins, which
			 * leak suppression mutes, off the LE channels. */
			if (H.le && value < LE_CENTER_MIN)
				value = LE_CENTER_MIN;
			if (H.le && value > LE_CENTER_MAX)
				value = LE_CENTER_MAX;
			if (H.le && !(value & 1))
				value--;
			if (H.le && H.le_center)
				value = H.le_center;
			if (value != H.center) {
				retune(value);
				fprintf(stderr, "HackRF: window at %d MHz (channels %d-%d)\n",
				        value, value - 2409 < 0 ? 0 : value - 2409,
				        value - 2395 > 78 ? 78 : value - 2395);
			}
		} else if (!H.want_scan) {
			H.want_scan = 1;
			scan_start();
			fprintf(stderr, "HackRF: scanning the band, %d ms per window\n",
			        H.scan_ms > 0 ? H.scan_ms : 100);
		}
		return 0;
	case UBERTOOTH_GET_CHANNEL:
		if (length < 2)
			return LIBUSB_ERROR_OVERFLOW;
		data[0] = H.center & 0xff;
		data[1] = (H.center >> 8) & 0xff;
		return 2;
	case UBERTOOTH_SET_MOD:
		if (value != MOD_BT_BASIC_RATE && value != MOD_BT_LOW_ENERGY)
			return refuse_mode();
		return set_le(value == MOD_BT_LOW_ENERGY);
	case UBERTOOTH_BTLE_SNIFFING: {
		/* follow (-f) or not (-n): the window cannot hop. Both capture
		 * advertising and the connections seen starting, on the window's
		 * channels. */
		int lo = H.center - 7 < 2402 ? 2402 : H.center - 7;
		int hi = H.center + 7 > 2480 ? 2480 : H.center + 7, f;
		if (set_le(1) != 0)
			return LIBUSB_ERROR_IO;
		fprintf(stderr, "HackRF: LE 1M on %d-%d MHz (LE channels", lo, hi);
		for (f = lo; f <= hi; f++)
			if (hle_chan_idx(f) >= 0)
				fprintf(stderr, " %d", hle_chan_idx(f));
		fprintf(stderr, ")%s\n", value ? "; connections are captured in the window, "
		        "not followed" : "");
		return ubertooth_hackrf_stream_start() == 0 ? 0 : LIBUSB_ERROR_IO;
	}
	case UBERTOOTH_LE_ADV_CONFIG:
		/* the firmware's advertising channel options: none apply */
		return value == 0 ? 0 : LIBUSB_ERROR_NOT_SUPPORTED;
	case UBERTOOTH_CANCEL_FOLLOW:
		return 0;
	/* modes the gateware cannot do */
	case UBERTOOTH_TX_SYMBOLS:
	case UBERTOOTH_TX_TEST:
	case UBERTOOTH_SPECAN:
	case UBERTOOTH_LED_SPECAN:
	case UBERTOOTH_REPEATER:
	case UBERTOOTH_RANGE_TEST:
	case UBERTOOTH_BTLE_PROMISC:
	case UBERTOOTH_BTLE_SLAVE:
	case UBERTOOTH_EGO:
	case UBERTOOTH_RX_GENERIC:
		return refuse_mode();
	case UBERTOOTH_AFH:
		H.afh = 1;
		H.want_scan = 1;
		scan_start();
		return 0;
	case UBERTOOTH_GET_REV_NUM:
		if (length < 3 + sizeof(rev) - 1)
			return LIBUSB_ERROR_OVERFLOW;
		data[0] = data[1] = 0;
		data[2] = sizeof(rev) - 1;
		memcpy(&data[3], rev, sizeof(rev) - 1);
		return 3 + sizeof(rev) - 1;
	case UBERTOOTH_GET_SERIAL: {
		/* the HackRF's 128-bit serial in the Ubertooth's format */
		read_partid_serialno_t ps;
		int i;
		if (length < 17)
			return LIBUSB_ERROR_OVERFLOW;
		if (hackrf_board_partid_serialno_read(H.dev, &ps) != HACKRF_SUCCESS)
			return LIBUSB_ERROR_IO;
		data[0] = 0;
		for (i = 0; i < 16; i++)
			data[1 + i] = (uint8_t)(ps.serial_no[i / 4] >> (8 * (3 - i % 4)));
		return 17;
	}
	/* piconet following, clock trims, squelch, maps, LEDs: nothing to do */
	case UBERTOOTH_SET_BDADDR:
	case UBERTOOTH_START_HOPPING:
	case UBERTOOTH_TRIM_CLOCK:
	case UBERTOOTH_FIX_CLOCK_DRIFT:
	case UBERTOOTH_SET_SQUELCH:
	case UBERTOOTH_SET_AFHMAP:
	case UBERTOOTH_CLEAR_AFHMAP:
	case UBERTOOTH_SET_USRLED:
	case UBERTOOTH_SET_RXLED:
	case UBERTOOTH_SET_TXLED:
		return length;
	default:
		/* after a mode was refused, the tool is on its way out: no more
		 * errors for the requests it still sends */
		return H.ut->stop_ubertooth ? 0 : LIBUSB_ERROR_NOT_SUPPORTED;
	}
}
