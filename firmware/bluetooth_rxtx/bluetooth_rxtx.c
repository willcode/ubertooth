/*
 * Copyright 2010-2013 Michael Ossmann
 * Copyright 2011-2013 Dominic Spill
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

#include <string.h>

#include "ubertooth.h"
#include "ubertooth_usb.h"
#include "ubertooth_interface.h"
#include "ubertooth_rssi.h"
#include "ubertooth_cs.h"
#include "ubertooth_dma.h"
#include "ubertooth_clock.h"
#include "bluetooth.h"
#include "bluetooth_le.h"
#include "le_phy.h"
#include "cc2400_rangetest.h"
#include "ego.h"
#include "debug_uart.h"
#include "xmas.h"

#define MIN(x,y)	((x)<(y)?(x):(y))
#define MAX(x,y)	((x)>(y)?(x):(y))

#define CC2400_FIFO_SIZE 32   /* bytes in the CC2400 TX/RX FIFO */
#define LE_ADV_DATA_MAX  31   /* max BLE AdvData length (Core Spec Vol 6 B 2.3.1) */

/* build info */
const char compile_info[] =
	"ubertooth " GIT_REVISION " (" COMPILE_BY "@" COMPILE_HOST ") " TIMESTAMP;

/* hopping stuff */
volatile uint8_t  hop_mode = HOP_NONE;
volatile uint8_t  do_hop = 0;                  // set by timer interrupt
volatile uint16_t channel = 2441;
volatile uint16_t hop_direct_channel = 0;      // for hopping directly to a channel
volatile uint16_t hop_timeout = 158;
volatile uint16_t requested_channel = 0;
volatile uint16_t le_adv_channel = 2402;
volatile int      cancel_follow = 0;

/* bulk USB stuff */
volatile uint8_t  idle_buf_clkn_high = 0;
volatile uint32_t idle_buf_clk100ns = 0;
volatile uint16_t idle_buf_channel = 0;
volatile uint8_t  dma_discard = 0;
volatile uint8_t  status = 0;

/* operation mode */
volatile uint8_t mode = MODE_IDLE;
volatile uint8_t requested_mode = MODE_IDLE;
volatile uint8_t jam_mode = JAM_NONE;
volatile uint8_t ego_mode = EGO_FOLLOW;

volatile uint8_t modulation = MOD_BT_BASIC_RATE;

/* specan stuff */
volatile uint16_t low_freq = 2400;
volatile uint16_t high_freq = 2483;
volatile int8_t rssi_threshold = -30;  // -54dBm - 30 = -84dBm

/* Generic TX stuff */
generic_tx_packet tx_pkt;

/* le slave stuff */
uint8_t slave_mac_address[6] = { 0, };
uint8_t le_adv_data[LE_ADV_MAX_LEN] = { 0x02, 0x01, 0x05 };
unsigned le_adv_len = 3;

le_state_t le = {
	.access_address = 0x8e89bed6,           // advertising channel access address
	.synch = 0x6b7d,                        // bit-reversed adv channel AA
	.syncl = 0x9171,
	.crc_init  = 0x555555,                  // advertising channel CRCInit
	.crc_init_reversed = 0xAAAAAA,
	.crc_verify = 0,

	.link_state = LINK_INACTIVE,
	.conn_epoch = 0,
	.do_follow = 1,
	.target_set = 0,
	.last_packet = 0,
};

typedef struct _le_promisc_active_aa_t {
	u32 aa;
	int count;
	int8_t rssi_max;  // strongest RSSI seen for this AA, dBm
} le_promisc_active_aa_t;

typedef struct _le_promisc_state_t {
	// LFU cache of recently seen AA's
	le_promisc_active_aa_t active_aa[32];

	// recovering hop interval
	u32 smallest_hop_interval;
	int consec_intervals;
	u32 prev_clk;
	int aa_found;
} le_promisc_state_t;
le_promisc_state_t le_promisc;

// UBERTOOTH_LE_PROMISC_CONFIG
static int8_t le_promisc_min_rssi = -128;
static u16 le_promisc_flags = 0;
#define AA_LIST_SIZE (int)(sizeof(le_promisc.active_aa) / sizeof(le_promisc_active_aa_t))

/* LE jamming */
#define JAM_COUNT_DEFAULT 40
int le_jam_count = 0;

/* set LE access address */
static void le_set_access_address(u32 aa);

typedef int (*data_cb_t)(char *);
data_cb_t data_cb = NULL;

typedef void (*packet_cb_t)(u8 *);
packet_cb_t packet_cb = NULL;

/* mode that was active before the current one returned (for tx_generic) */
static uint8_t last_mode = MODE_IDLE;

/* Unpacked symbol buffers (two rxbufs) */
char unpacked[DMA_SIZE*8*2] AHB_BSS;

/* Queue one USB packet.  buf must hold at least DMA_SIZE bytes. */
static int enqueue_meta(uint8_t type, const uint8_t* buf, uint8_t clkn_high,
                        uint32_t clk100ns, uint16_t chan)
{
	usb_pkt_rx* f = usb_enqueue_reserve();

	/* fail if queue is full */
	if (f == NULL) {
		status |= FIFO_OVERFLOW;
		return 0;
	}

	f->pkt_type = type;
	f->clkn_high = clkn_high;
	f->clk100ns = clk100ns;
	f->channel = (uint8_t)((chan - 2402) & 0xff);
	f->rssi_min = rssi_min;
	f->rssi_max = rssi_max;
	f->rssi_avg = rssi_get_avg(chan);
	f->rssi_count = rssi_count;

	memcpy(f->data, buf, DMA_SIZE);

	f->status = status;
	status = 0;

	usb_enqueue_commit();
	return 1;
}

/* Queue a packet with the metadata of the last completed DMA buffer
 * (SPECAN packets get the current time). */
static int enqueue(uint8_t type, uint8_t* buf)
{
	if (type == SPECAN)
		return enqueue_meta(type, buf, (clkn >> 20) & 0xff, CLK100NS, channel);
	/* since API 0x0108 the clkn_high byte carries LE flags in LE packets */
	if (type == LE_PACKET)
		return enqueue_meta(type, buf, 0, idle_buf_clk100ns, idle_buf_channel);
	return enqueue_meta(type, buf, idle_buf_clkn_high, idle_buf_clk100ns,
	                    idle_buf_channel);
}

int enqueue_with_ts(uint8_t type, uint8_t* buf, uint32_t ts)
{
	usb_pkt_rx* f = usb_enqueue_reserve();

	/* fail if queue is full */
	if (f == NULL) {
		status |= FIFO_OVERFLOW;
		return 0;
	}

	f->pkt_type = type;

	f->clkn_high = 0;
	f->clk100ns = ts;

	f->channel = (uint8_t)((channel - 2402) & 0xff);
	f->rssi_avg = 0;
	f->rssi_count = 0;

	memcpy(f->data, buf, DMA_SIZE);

	f->status = status;
	status = 0;

	usb_enqueue_commit();
	return 1;
}

static int vendor_request_handler(uint8_t request, uint16_t* request_params, uint8_t* data, int* data_len)
{
	uint32_t clock;
	size_t length; // string length
	usb_pkt_rx* p = NULL;
	uint16_t reg_val;
	uint8_t i;
	unsigned n;
	/* number of bytes the host sent in data[] (0 for device-to-host) */
	unsigned data_in_len = request_params[2];

/* fail the request if the host did not send at least len data bytes */
#define REQUIRE_DATA(len) do { if (data_in_len < (len)) return 0; } while (0)

	switch (request) {

	case UBERTOOTH_PING:
		*data_len = 0;
		break;

	case UBERTOOTH_RX_SYMBOLS:
		requested_mode = MODE_RX_SYMBOLS;
		*data_len = 0;
		break;

#ifdef TX_ENABLE
	case UBERTOOTH_TX_SYMBOLS:
		hop_mode = HOP_BLUETOOTH;
		requested_mode = MODE_TX_SYMBOLS;
		*data_len = 0;
		break;
#endif

	case UBERTOOTH_GET_USRLED:
		data[0] = (USRLED) ? 1 : 0;
		*data_len = 1;
		break;

	case UBERTOOTH_SET_USRLED:
		if (request_params[0])
			USRLED_SET;
		else
			USRLED_CLR;
		break;

	case UBERTOOTH_GET_RXLED:
		data[0] = (RXLED) ? 1 : 0;
		*data_len = 1;
		break;

	case UBERTOOTH_SET_RXLED:
		if (request_params[0])
			RXLED_SET;
		else
			RXLED_CLR;
		break;

	case UBERTOOTH_GET_TXLED:
		data[0] = (TXLED) ? 1 : 0;
		*data_len = 1;
		break;

	case UBERTOOTH_SET_TXLED:
		if (request_params[0])
			TXLED_SET;
		else
			TXLED_CLR;
		break;

	case UBERTOOTH_GET_1V8:
		data[0] = (CC1V8) ? 1 : 0;
		*data_len = 1;
		break;

	case UBERTOOTH_SET_1V8:
		if (request_params[0])
			CC1V8_SET;
		else
			CC1V8_CLR;
		break;

	case UBERTOOTH_GET_PARTNUM:
		get_part_num(data, data_len);
		break;

	case UBERTOOTH_RESET:
		requested_mode = MODE_RESET;
		wdt_reset_in(300); /* in case the main loop is stuck */
		break;

	case UBERTOOTH_GET_SERIAL:
		get_device_serial(data, data_len);
		break;

#ifdef UBERTOOTH_ONE
	case UBERTOOTH_GET_PAEN:
		data[0] = (PAEN) ? 1 : 0;
		*data_len = 1;
		break;

	case UBERTOOTH_SET_PAEN:
		if (request_params[0])
			PAEN_SET;
		else
			PAEN_CLR;
		break;

	case UBERTOOTH_GET_HGM:
		data[0] = (HGM) ? 1 : 0;
		*data_len = 1;
		break;

	case UBERTOOTH_SET_HGM:
		if (request_params[0])
			HGM_SET;
		else
			HGM_CLR;
		break;
#endif

#ifdef TX_ENABLE
	case UBERTOOTH_TX_TEST:
		requested_mode = MODE_TX_TEST;
		break;

	case UBERTOOTH_GET_PALEVEL:
		data[0] = cc2400_get(FREND) & 0x7;
		*data_len = 1;
		break;

	case UBERTOOTH_SET_PALEVEL:
		if( request_params[0] < 8 ) {
			cc2400_set(FREND, 8 | request_params[0]);
		} else {
			return 0;
		}
		break;

	case UBERTOOTH_RANGE_TEST:
		requested_mode = MODE_RANGE_TEST;
		break;

	case UBERTOOTH_REPEATER:
		requested_mode = MODE_REPEATER;
		break;
#endif

	case UBERTOOTH_RANGE_CHECK:
		data[0] = rr.valid;
		data[1] = rr.request_pa;
		data[2] = rr.request_num;
		data[3] = rr.reply_pa;
		data[4] = rr.reply_num;
		*data_len = 5;
		break;

	case UBERTOOTH_STOP:
		requested_mode = MODE_IDLE;
		le_phy_set_host_ext(0);
		break;

	case UBERTOOTH_GET_MOD:
		data[0] = modulation;
		*data_len = 1;
		break;

	case UBERTOOTH_SET_MOD:
		modulation = request_params[0];
		break;

	case UBERTOOTH_GET_CHANNEL:
		data[0] = channel & 0xFF;
		data[1] = (channel >> 8) & 0xFF;
		*data_len = 2;
		break;

	case UBERTOOTH_SET_CHANNEL:
		requested_channel = request_params[0];
		/* bluetooth band sweep mode, start at channel 2402 */
		if (requested_channel > MAX_FREQ) {
			hop_mode = HOP_SWEEP;
			requested_channel = 2402;
		}
		/* fixed channel mode, can be outside bluetooth band */
		else {
			hop_mode = HOP_NONE;
			requested_channel = MAX(requested_channel, MIN_FREQ);
			requested_channel = MIN(requested_channel, MAX_FREQ);
		}

		le_adv_channel = requested_channel;
		if (mode != MODE_BT_FOLLOW_LE) {
			channel = requested_channel;
			requested_channel = 0;

			/* CS threshold is mode-dependent. Update it after
			 * possible mode change. TODO - kludgy. */
			cs_threshold_calc_and_set(channel);
		}
		break;

	case UBERTOOTH_SET_ISP:
		set_isp();
		*data_len = 0; /* should never return */
		break;

	case UBERTOOTH_FLASH:
		bootloader_ctrl = DFU_MODE;
		requested_mode = MODE_RESET;
		wdt_reset_in(300); /* in case the main loop is stuck */
		break;

	case UBERTOOTH_SPECAN:
		/* stay inside the range the CC2400 synthesizer can tune */
		if (request_params[0] < MIN_FREQ || request_params[0] > MAX_FREQ ||
				request_params[1] < MIN_FREQ || request_params[1] > MAX_FREQ ||
				request_params[1] < request_params[0])
			return 0;
		low_freq = request_params[0];
		high_freq = request_params[1];
		requested_mode = MODE_SPECAN;
		*data_len = 0;
		break;

	case UBERTOOTH_RX_GENERIC:
		requested_mode = MODE_RX_GENERIC;
		*data_len = 0;
		break;

	case UBERTOOTH_LED_SPECAN:
		/* rssi_threshold is an int8_t: 54 - 182 = -128 is the minimum */
		if (request_params[0] > 182)
			return 0;
		rssi_threshold = 54 - request_params[0];
		requested_mode = MODE_LED_SPECAN;
		*data_len = 0;
		break;

	case UBERTOOTH_GET_REV_NUM:
		data[0] = 0x00;
		data[1] = 0x00;

		length = (u8)strlen(GIT_REVISION);
		data[2] = length;

		memcpy(&data[3], GIT_REVISION, length);

		*data_len = 2 + 1 + length;
		break;

	case UBERTOOTH_GET_COMPILE_INFO:
		length = (u8)strlen(compile_info);
		data[0] = length;
		memcpy(&data[1], compile_info, length);
		*data_len = 1 + length;
		break;

	case UBERTOOTH_GET_BOARD_ID:
		data[0] = BOARD_ID;
		*data_len = 1;
		break;

	case UBERTOOTH_SET_SQUELCH:
		cs_threshold_req = (int8_t)request_params[0];
		cs_threshold_calc_and_set(channel);
		break;

	case UBERTOOTH_GET_SQUELCH:
		data[0] = cs_threshold_req;
		*data_len = 1;
		break;

	case UBERTOOTH_SET_BDADDR:
		REQUIRE_DATA(16);
		target.address = 0;
		target.syncword = 0;
		for(int i=0; i < 8; i++) {
			target.address |= (uint64_t)data[i] << 8*i;
		}
		for(int i=0; i < 8; i++) {
			target.syncword |= (uint64_t)data[i+8] << 8*i;
		}
		precalc();
		break;

	case UBERTOOTH_START_HOPPING:
		REQUIRE_DATA(6);
		/* Assemble the offset locally and store it with a single write:
		 * TIMER0_IRQHandler consumes clkn_offset and must never see a
		 * partially assembled value. */
		clock = 0;
		for(int i=0; i < 4; i++) {
			clock <<= 8;
			clock |= data[i];
		}
		clkn_offset = clock;
		hop_mode = HOP_BLUETOOTH;
		dma_discard = 1;
		DIO_SSEL_SET;
		clk100ns_offset = (data[4] << 8) | (data[5] << 0);
		requested_mode = MODE_BT_FOLLOW;
		break;

	case UBERTOOTH_AFH:
		hop_mode = HOP_AFH;
		requested_mode = MODE_AFH;

		for(int i=0; i < 10; i++) {
			afh_map[i] = 0;
		}
		afh_enabled = 1;
		afh_update();
		break;

	case UBERTOOTH_HOP:
		do_hop = 1;
		break;

	case UBERTOOTH_SET_CLOCK:
		REQUIRE_DATA(4);
		clock = data[0] | data[1] << 8 | data[2] << 16 | data[3] << 24;
		clkn = clock;
		cs_threshold_calc_and_set(channel);
		break;

	case UBERTOOTH_SET_AFHMAP:
		REQUIRE_DATA(10);
		for(int i=0; i < 10; i++) {
			afh_map[i] = data[i];
		}
		afh_enabled = 1;
		afh_update();
		break;

	case UBERTOOTH_CLEAR_AFHMAP:
		for(int i=0; i < 10; i++) {
			afh_map[i] = 0;
		}
		afh_enabled = 0;
		afh_update();
		break;

	case UBERTOOTH_GET_CLOCK:
		clock = clkn;
		for(int i=0; i < 4; i++) {
			data[i] = (clock >> (8*i)) & 0xff;
		}
		*data_len = 4;
		break;

	case UBERTOOTH_TRIM_CLOCK:
		REQUIRE_DATA(2);
		clk100ns_offset = (data[0] << 8) | (data[1] << 0);
		break;

	case UBERTOOTH_FIX_CLOCK_DRIFT:
		REQUIRE_DATA(2);
		clk_drift_ppm += (int16_t)((data[0] << 8) | (data[1] << 0));

		// Too slow
		if (clk_drift_ppm < 0) {
			clk_drift_correction = 320 / (uint16_t)(-clk_drift_ppm);
			clkn_next_drift_fix = clkn_last_drift_fix + clk_drift_correction;
		}
		// Too fast
		else if (clk_drift_ppm > 0) {
			clk_drift_correction = 320 / clk_drift_ppm;
			clkn_next_drift_fix = clkn_last_drift_fix + clk_drift_correction;
		}
		// Don't trim
		else {
			clk_drift_correction = 0;
			clkn_next_drift_fix = 0;
		}

		break;

	case UBERTOOTH_BTLE_SNIFFING:
		le.do_follow = request_params[0];
		*data_len = 0;

		do_hop = 0;
		hop_mode = HOP_BTLE;
		requested_mode = MODE_BT_FOLLOW_LE;

		usb_queue_init();
		cs_threshold_calc_and_set(channel);
		break;

	case UBERTOOTH_GET_ACCESS_ADDRESS:
		for(int i=0; i < 4; i++) {
			data[i] = (le.access_address >> (8*i)) & 0xff;
		}
		*data_len = 4;
		break;

	case UBERTOOTH_SET_ACCESS_ADDRESS:
		REQUIRE_DATA(4);
		le_set_access_address(data[0] | data[1] << 8 | data[2] << 16 | data[3] << 24);
		le.aa_set = 1;
		break;

	case UBERTOOTH_DO_SOMETHING:
		// do something! just don't commit anything here
		break;

	case UBERTOOTH_DO_SOMETHING_REPLY:
		// after you do something, tell me what you did!
		// don't commit here please
		data[0] = 0x13;
		data[1] = 0x37;
		*data_len = 2;
		break;

	case UBERTOOTH_GET_CRC_VERIFY:
		data[0] = le.crc_verify ? 1 : 0;
		*data_len = 1;
		break;

	case UBERTOOTH_SET_CRC_VERIFY:
		le.crc_verify = request_params[0] ? 1 : 0;
		break;

	case UBERTOOTH_POLL:
		p = usb_dequeue_peek();
		if (p != NULL) {
			memcpy(data, (void *)p, sizeof(usb_pkt_rx));
			usb_dequeue_release();
			*data_len = sizeof(usb_pkt_rx);
		} else {
			data[0] = 0;
			*data_len = 1;
		}
		break;

	case UBERTOOTH_BTLE_PROMISC:
		*data_len = 0;

		hop_mode = HOP_NONE;
		requested_mode = MODE_BT_PROMISC_LE;

		usb_queue_init();
		cs_threshold_calc_and_set(channel);
		break;

	case UBERTOOTH_READ_REGISTER:
		reg_val = cc2400_get(request_params[0]);
		data[0] = (reg_val >> 8) & 0xff;
		data[1] = reg_val & 0xff;
		*data_len = 2;
		break;

	case UBERTOOTH_WRITE_REGISTER:
		cc2400_set(request_params[0] & 0xff, request_params[1]);
		break;

	case UBERTOOTH_WRITE_REGISTERS:
		/* wValue register writes of 3 bytes each, bounded by the data
		 * actually received */
		n = MIN((unsigned)request_params[0], data_in_len / 3);
		for (unsigned k = 0; k < n; k++) {
			reg_val = (data[(k*3)+1] << 8) | data[(k*3)+2];
			cc2400_set(data[k*3], reg_val);
		}
		break;

	case UBERTOOTH_READ_ALL_REGISTERS:
		#define MAX_READ_REG 0x2d
		for(i=0; i<=MAX_READ_REG; i++) {
			reg_val = cc2400_get(i);
			data[i*3] = i;
			data[(i*3)+1] = (reg_val >> 8) & 0xff;
			data[(i*3)+2] = reg_val & 0xff;
		}
		*data_len = (MAX_READ_REG+1)*3;
		break;

#ifdef TX_ENABLE
	case UBERTOOTH_TX_GENERIC_PACKET:
		/* header: synch(2) syncl(2) channel(2) length(1) pa_level(1),
		 * followed by length payload bytes.  The payload must fit in
		 * tx_pkt.data and in the CC2400's 32 byte TX FIFO. */
		REQUIRE_DATA(8);
		n = data[6];
		if (n > MIN(sizeof(tx_pkt.data), CC2400_FIFO_SIZE) ||
				data_in_len < 8 + n)
			return 0;
		memset(&tx_pkt, 0, sizeof(tx_pkt));
		memcpy(&tx_pkt, data, 8 + n);
		requested_mode = MODE_TX_GENERIC;
		*data_len = 0;
		break;

	case UBERTOOTH_BTLE_SLAVE:
		REQUIRE_DATA(6);
		memcpy(slave_mac_address, data, 6);
		requested_mode = MODE_BT_SLAVE_LE;
		break;
#endif

	case UBERTOOTH_LE_SET_ADV_DATA:
		// AdvData is at most 31 bytes (and must fit bt_slave_le's PDU)
		if (data_in_len > LE_ADV_DATA_MAX)
			return 0;
		le_adv_len = data_in_len;
		memcpy(le_adv_data, data, le_adv_len);
		break;

	case UBERTOOTH_BTLE_SET_TARGET:
		// Addresses appear in packets in reverse-octet order.
		// Store the target address in reverse order so that we can do a simple memcmp later
		REQUIRE_DATA(7);
		if (data[6] > 48) {
			return 0; // invalid mask
		}
		else if (data[6] == 0) {
			le.target_set = 0;
			memset(le.target, 0, 6);
			memset(le.target_mask, 0, 6);
		} else {
			unsigned last;

			for (i = 0; i < 6; ++i)
				le.target[i] = data[5-i];

			// compute mask
			memset(le.target_mask, 0, 6);
			for (i = 5; data[6] > 8; --i, data[6] -= 8) {
				le.target_mask[i] = 0xff;
			}
			last = i;

			if (data[6] > 0) {
				uint8_t final_byte = 0;
				for (i = 0; i < data[6]; ++i) {
					final_byte >>= 1;
					final_byte |= 0b10000000;
				}
				le.target_mask[last] = final_byte;
			}

			// in case the user specifies a bad mask
			for (i = 0; i < 6; ++i)
				le.target[i] &= le.target_mask[i];

			le.target_set = 1;
		}
		break;

	case UBERTOOTH_CANCEL_FOLLOW:
		// cancel following an active connection
		cancel_follow = 1;
		break;

#ifdef TX_ENABLE
	case UBERTOOTH_JAM_MODE:
		jam_mode = request_params[0];
		break;
#endif

	case UBERTOOTH_EGO:
		ego_mode = request_params[0];
#ifndef TX_ENABLE
		if (ego_mode == EGO_JAM)
			return 0;
#endif
		requested_mode = MODE_EGO;
		break;

	case UBERTOOTH_RFCAT_SUBCMD:
		return rfcat_subcommand(request_params[0], data, data_in_len);
		break;

	case UBERTOOTH_XMAS:
		requested_mode = MODE_XMAS;
		break;

	// the following are applied by the LE follow mode main loop
	case UBERTOOTH_LE_MONITOR:
		le_phy_set_host_ext(1);
		if (request_params[0] != 0) {
			REQUIRE_DATA(4);
			if (request_params[0] < 2402 || request_params[0] > 2480)
				return 0;
		}
		le_phy_request_monitor(request_params[0],
				data[0] | data[1] << 8 | data[2] << 16 | (u32)data[3] << 24);
		break;

	case UBERTOOTH_LE_FOLLOW_PARAMS: {
		le_phy_set_host_ext(1);
		le_follow_params_t params;
		REQUIRE_DATA(sizeof(params));
		memcpy(&params, data, sizeof(params));
		le_phy_request_follow(&params);
		break;
	}

	case UBERTOOTH_LE_ADV_CONFIG:
		le_phy_set_host_ext(1);
		le_phy_set_adv_config(request_params[0], request_params[1]);
		break;

	case UBERTOOTH_LE_PROMISC_CONFIG:
		le_phy_set_host_ext(1);
		le_promisc_min_rssi = (int8_t)(request_params[0] & 0xff);
		le_promisc_flags = request_params[1];
		break;

	default:
		return 0;
	}
	return 1;
}

/* Update CLKN. */
void TIMER0_IRQHandler()
{
	if (T0IR & TIR_MR0_Interrupt) {

		clkn += clkn_offset + 1;
		clkn_offset = 0;

		uint32_t le_clk = (clkn - le.conn_epoch) & 0x03;

		/* Trigger hop based on mode */

		/* NONE or SWEEP -> 25 Hz */
		if (hop_mode == HOP_NONE || hop_mode == HOP_SWEEP) {
			if ((clkn & 0x7f) == 0)
				do_hop = 1;
		}
		/* BLUETOOTH -> 1600 Hz */
		else if (hop_mode == HOP_BLUETOOTH) {
			if ((clkn & 0x1) == 0)
				do_hop = 1;
		}
		/* BLUETOOTH Low Energy -> 7.5ms - 4.0s in multiples of 1.25 ms */
		else if (hop_mode == HOP_BTLE) {
			// Only hop if connected
			if (le.link_state == LINK_CONNECTED && le_clk == 0) {
				--le.interval_timer;
				if (le.interval_timer == 0) {
					do_hop = 1;
					++le.conn_count;
					le.interval_timer = le.conn_interval;
				} else {
					TXLED_CLR; // hack!
				}
			}
		}
		else if (hop_mode == HOP_AFH) {
			/* clkn can jump (clkn_offset, SET_CLOCK), so test for
			 * "at least hop_timeout ticks", not exact equality */
			if ((int32_t)(clkn - (last_hop + hop_timeout)) >= 0) {
				do_hop = 1;
			}
		}

		// Fix linear clock drift deviation
		if(clkn_next_drift_fix != 0 && clk100ns_offset == 0) {
			if((int32_t)(clkn - clkn_next_drift_fix) >= 0) {

				// Too fast
				if(clk_drift_ppm >= 0) {
					clk100ns_offset = 1;
				}

				// Too slow
				else {
					clk100ns_offset = 6249;
				}
				clkn_last_drift_fix = clkn;
				clkn_next_drift_fix = clkn_last_drift_fix + clk_drift_correction;
			}
		}

		/*
		 * Length of the CLKN tick that has just started, in 100 ns units:
		 *   offset 0           -> 3125 (nominal)
		 *   offset 1..3124     -> 3125 + offset (delay the clock)
		 *   offset 3125..6249  -> 3125 - (6250 - offset) (advance it)
		 * Advancing is done by shortening this tick.  (It used to be
		 * done by stretching the tick to ~3 ticks and adding 2 to clkn
		 * up front, which mis-timed one hop per correction.)  T0MR0 must
		 * stay ahead of T0TC, which is already counting this tick.
		 */
		{
			uint32_t off = clk100ns_offset;
			uint32_t mr0 = 3124;
			clk100ns_offset = 0;

			if (off > 0 && off <= 3124) {
				mr0 = 3124 + off;
			} else if (off > 3124 && off < 6250) {
				uint32_t min_mr0 = T0TC + 16;
				mr0 = 3124 - (6250 - off);
				if (mr0 < min_mr0)
					mr0 = min_mr0;
			}
			T0MR0 = mr0;
		}

		/* Ack interrupt */
		T0IR = TIR_MR0_Interrupt;
	}
}

/* EINT3 handler is also defined in ubertooth.c for TC13BADGE. */
#ifndef TC13BADGE
void EINT3_IRQHandler()
{
	/* TODO - check specific source of shared interrupt */
	IO2IntClr   = PIN_GIO6; // clear interrupt
	DIO_SSEL_CLR;           // enable SPI
	cs_trigger  = 1;        // signal trigger
	if (hop_mode == HOP_BLUETOOTH)
		dma_discard = 0;

}
#endif // TC13BADGE

/*
 * Sleep (busy wait) for 'millis' milliseconds
 * Needs clkn. Be sure to call clkn_init() before using it.
 */
static void msleep(uint32_t millis)
{
	uint32_t start = clkn;
	uint32_t ticks = millis * 16 / 5; // millis -> clkn ticks (3.2 per ms)

	// unsigned difference is correct across clkn wraparound
	while ((uint32_t)(clkn - start) < ticks);
}

void legacy_DMA_IRQHandler();
void le_DMA_IRQHandler();
void DMA_IRQHandler(void) {
	if (mode == MODE_BT_FOLLOW_LE)
		le_DMA_IRQHandler();
	else
		legacy_DMA_IRQHandler();

	// DMA channel 7: debug UART
	if (DMACIntStat & (1 << 7)) {
		// TC -- DMA completed, unset flag so another printf can occur
		if (DMACIntTCStat & (1 << 7)) {
			DMACIntTCClear = (1 << 7);
			debug_dma_active = 0;
		}
		// error -- blow up
		if (DMACIntErrStat & (1 << 7)) {
			DMACIntErrClr = (1 << 7);
			// FIXME do something better here
			USRLED_SET;
			while (1) { }
		}
	}
}

void legacy_DMA_IRQHandler()
{
	if ( mode == MODE_RX_SYMBOLS
	   || mode == MODE_BT_FOLLOW
	   || mode == MODE_AFH
	   || mode == MODE_SPECAN
	   || mode == MODE_BT_FOLLOW_LE
	   || mode == MODE_BT_PROMISC_LE
	   || mode == MODE_BT_SLAVE_LE
	   || mode == MODE_RX_GENERIC)
	{
		/* interrupt on channel 0 */
		if (DMACIntStat & (1 << 0)) {
			if (DMACIntTCStat & (1 << 0)) {
				DMACIntTCClear = (1 << 0);

				if (hop_mode == HOP_BLUETOOTH)
					DIO_SSEL_SET;

				idle_buf_clk100ns  = CLK100NS;
				idle_buf_clkn_high = (clkn >> 20) & 0xff;
				idle_buf_channel   = channel;

				/* Keep buffer swapping in sync with DMA. */
				volatile uint8_t* tmp = active_rxbuf;
				active_rxbuf = idle_rxbuf;
				idle_rxbuf = tmp;

				++rx_tc;
			}
			if (DMACIntErrStat & (1 << 0)) {
				DMACIntErrClr = (1 << 0);
				++rx_err;
			}
		}
	} else if (DMACIntStat & (1 << 0)) {
		/* Channel 0 interrupt in a mode that does not use it: clear it
		 * anyway, a pending flag would retrigger this IRQ forever. */
		DMACIntTCClear = (1 << 0);
		DMACIntErrClr = (1 << 0);
	}
}

/*
 * Turn the radio, PA, LEDs and the DIO SSP/DMA off and clear transient
 * receive state.  Host-supplied configuration (channel, hop mode,
 * squelch, modulation, ...), the clock and requested_mode are left
 * alone, so a mode requested while another one was running is not lost.
 */
static void cc2400_radio_off(void)
{
	cc2400_strobe(SRFOFF);
	cc2400_wait_status(FS_LOCK, 0); // need to wait for unlock?

#ifdef UBERTOOTH_ONE
	PAEN_CLR;
	HGM_CLR;
#endif

	RXLED_CLR;
	TXLED_CLR;
	USRLED_CLR;

	dio_ssp_stop();
	cs_trigger_disable();
	rssi_reset();

	do_hop = 0;

	/* bulk USB stuff */
	idle_buf_clkn_high = 0;
	idle_buf_clk100ns = 0;
	idle_buf_channel = 0;
	dma_discard = 0;
	status = 0;
	rx_tc = 0;
	rx_err = 0;
}

/*
 * Enter IDLE: radio off, clock stopped and all host configuration reset
 * to defaults.  Does not touch requested_mode.
 */
static void cc2400_idle()
{
	cc2400_radio_off();

	clkn_stop();
	cs_reset();
	rssi_iir_reset();

	/* hopping stuff */
	hop_mode = HOP_NONE;
	channel = 2441;
	hop_direct_channel = 0;
	hop_timeout = 158;
	requested_channel = 0;
	le_adv_channel = 2402;

	/* operation mode */
	mode = MODE_IDLE;
	last_mode = MODE_IDLE;
	jam_mode = JAM_NONE;
	ego_mode = EGO_FOLLOW;

	modulation = MOD_BT_BASIC_RATE;

	/* specan stuff */
	low_freq = 2400;
	high_freq = 2483;
	rssi_threshold = -30;

	target.address = 0;
	target.syncword = 0;
}

/* start un-buffered rx */
static void cc2400_rx()
{
	u16 mdmctrl = 0;

	if((modulation == MOD_BT_BASIC_RATE) || (modulation == MOD_BT_LOW_ENERGY)) {
		if (modulation == MOD_BT_BASIC_RATE) {
			mdmctrl = 0x0029; // 160 kHz frequency deviation
		} else if (modulation == MOD_BT_LOW_ENERGY) {
			mdmctrl = 0x0040; // 250 kHz frequency deviation
		}
		cc2400_set(MANAND,  0x7fff);
		cc2400_set(LMTST,   0x2b22);
		cc2400_set(MDMTST0, 0x134b); // without PRNG
		cc2400_set(GRMDM,   0x0101); // un-buffered mode, GFSK
		// 0 00 00 0 010 00 0 00 0 1
		//      |  | |   |  +--------> CRC off
		//      |  | |   +-----------> sync word: 8 MSB bits of SYNC_WORD
		//      |  | +---------------> 2 preamble bytes of 01010101
		//      |  +-----------------> not packet mode
			//      +--------------------> un-buffered mode
		cc2400_set(FSDIV,   channel - 1); // 1 MHz IF
		cc2400_set(MDMCTRL, mdmctrl);
	}

	// Set up CS register
	cs_threshold_calc_and_set(channel);

	clkn_start();

	cc2400_wait_status(XOSC16M_STABLE, 1);
	cc2400_strobe(SFSON);
	cc2400_wait_status(FS_LOCK, 1);
	cc2400_strobe(SRX);
#ifdef UBERTOOTH_ONE
	PAEN_SET;
	HGM_SET;
#endif
}

/* start un-buffered rx */
static void cc2400_rx_sync(u32 sync)
{
	u16 grmdm, mdmctrl;

	if (modulation == MOD_BT_BASIC_RATE) {
		mdmctrl = 0x0029; // 160 kHz frequency deviation
		grmdm = 0x0461; // un-buffered mode, packet w/ sync word detection
		// 0 00 00 1 000 11 0 00 0 1
		//   |  |  | |   |  +--------> CRC off
		//   |  |  | |   +-----------> sync word: 32 MSB bits of SYNC_WORD
		//   |  |  | +---------------> 0 preamble bytes of 01010101
		//   |  |  +-----------------> packet mode
		//   |  +--------------------> un-buffered mode
		//   +-----------------------> sync error bits: 0

	} else if (modulation == MOD_BT_LOW_ENERGY) {
		mdmctrl = 0x0040; // 250 kHz frequency deviation
		grmdm = 0x0561; // un-buffered mode, packet w/ sync word detection
		// 0 00 00 1 010 11 0 00 0 1
		//   |  |  | |   |  +--------> CRC off
		//   |  |  | |   +-----------> sync word: 32 MSB bits of SYNC_WORD
		//   |  |  | +---------------> 2 preamble bytes of 01010101
		//   |  |  +-----------------> packet mode
		//   |  +--------------------> un-buffered mode
		//   +-----------------------> sync error bits: 0

	} else {
		/* oops */
		return;
	}

	cc2400_set(MANAND,  0x7fff);
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

	cc2400_set(FSDIV,   channel - 1); // 1 MHz IF
	cc2400_set(MDMCTRL, mdmctrl);

	// Set up CS register
	cs_threshold_calc_and_set(channel);

	clkn_start();

	cc2400_wait_status(XOSC16M_STABLE, 1);
	cc2400_strobe(SFSON);
	cc2400_wait_status(FS_LOCK, 1);
	cc2400_strobe(SRX);
#ifdef UBERTOOTH_ONE
	PAEN_SET;
	HGM_SET;
#endif
}

/* start buffered tx */
#ifdef TX_ENABLE
static void cc2400_tx_sync(uint32_t sync)
{
	// Bluetooth-like modulation
	cc2400_set(MANAND,  0x7fff);
	cc2400_set(LMTST,   0x2b22);    // LNA and receive mixers test register
	cc2400_set(MDMTST0, 0x134b);    // no PRNG

	cc2400_set(GRMDM,   0x0c01);
	// 0 00 01 1 000 00 0 00 0 1
	//      |  | |   |  +--------> CRC off
	//      |  | |   +-----------> sync word: 8 MSB bits of SYNC_WORD
	//      |  | +---------------> 0 preamble bytes of 01010101
	//      |  +-----------------> packet mode
	//      +--------------------> buffered mode

	cc2400_set(SYNCL,   sync & 0xffff);
	cc2400_set(SYNCH,   (sync >> 16) & 0xffff);

	cc2400_set(FSDIV,   channel);
	cc2400_set(FREND,   0b1011);    // amplifier level (-7 dBm, picked from hat)

	if (modulation == MOD_BT_BASIC_RATE) {
		cc2400_set(MDMCTRL, 0x0029);    // 160 kHz frequency deviation
	} else if (modulation == MOD_BT_LOW_ENERGY) {
		cc2400_set(MDMCTRL, 0x0040);    // 250 kHz frequency deviation
	} else {
		/* oops */
		return;
	}

	clkn_start();

	cc2400_wait_status(XOSC16M_STABLE, 1);
	cc2400_strobe(SFSON);
	cc2400_wait_status(FS_LOCK, 1);

#ifdef UBERTOOTH_ONE
	PAEN_SET;
#endif

	while ((cc2400_get(FSMSTATE) & 0x1f) != STATE_STROBE_FS_ON);
	cc2400_strobe(STX);

}
#endif

/*
 * Transmit a BTLE packet with the specified access address.
 *
 * All modulation parameters are set within this function. The data
 * should not be pre-whitened, but the CRC should be calculated and
 * included in the data length.
 *
 */
#ifdef TX_ENABLE
void le_transmit(u32 aa, u8 len, u8 *data)
{
	unsigned int fifo_space, i, j;
	int bit;
	u8 txbuf[LE_ADV_MAX_LEN];
	u8 tx_len;
	u8 byte;
	uint32_t sync = rbit(aa);
	u16 gio_save;

	// whiten the data and copy it into the txbuf
	int idx = whitening_index[btle_channel_index(channel)];
	for (i = 0; i < len; ++i) {
		byte = data[i];
		txbuf[i] = 0;
		for (j = 0; j < 8; ++j) {
			bit = (byte & 1) ^ whitening[idx];
			idx = (idx + 1) % sizeof(whitening);
			byte >>= 1;
			txbuf[i] |= bit << (7 - j);
		}
	}

	// Bluetooth-like modulation
	cc2400_set(MANAND,  0x7fff);
	cc2400_set(LMTST,   0x2b22);    // LNA and receive mixers test register
	cc2400_set(MDMTST0, 0x134b);    // no PRNG

	cc2400_set(GRMDM,   0x0ce1);
	// 0 00 01 1 001 11 0 00 0 1
	//      |  | |   |  +--------> CRC off
	//      |  | |   +-----------> sync word: all 32 bits of SYNC_WORD
	//      |  | +---------------> 1 preamble byte of 01010101
	//      |  +-----------------> packet mode
	//      +--------------------> buffered mode

	cc2400_set(FSDIV,   channel);
	cc2400_set(FREND,   0b1011);    // amplifier level (-7 dBm, picked from hat)
	cc2400_set(MDMCTRL, 0x0040);    // 250 kHz frequency deviation
	cc2400_set(INT,     0x0010);    // FIFO_THRESHOLD: 16 bytes

	// set sync word to bit-reversed AA
	cc2400_set(SYNCL,   sync & 0xffff);
	cc2400_set(SYNCH,   (sync >> 16) & 0xffff);

	// set GIO to FIFO_FULL
	gio_save = cc2400_get(IOCFG);
	cc2400_set(IOCFG, (GIO_FIFO_EMPTY << 9) | (gio_save & 0x1ff));

	// turn on the radio
	cc2400_wait_status(XOSC16M_STABLE, 1);
	cc2400_strobe(SFSON);
	cc2400_wait_status(FS_LOCK, 1);
	TXLED_SET;
#ifdef UBERTOOTH_ONE
	PAEN_SET;
#endif
	while ((cc2400_get(FSMSTATE) & 0x1f) != STATE_STROBE_FS_ON);
	/*
		This is the correct way of talking to the CC2400 FIFO, the algorithm is as follows:
	 	While i need to send more data:
	 		1. Wait for FIFO to drain according to FIFO_THRESHOLD, (or if first iteration immediately)
	 		2. Fill the FIFO with the correct amount of data to send (fifo_space is an indicator for how much data has "drained" from the fifo and is now available for us to fill up again)
	 		3. Signal FIFO to send data
	*/
	fifo_space = 32; // We start at 32, because the C2400 FIFO overall size is 32 bytes.
	i = 0;
	while (len != 0)
	{
		// Wait for the FIFO to drain (when FIFO_FULL is false)	
		while (!GIO6);
		// Decide how much we should send in this iteration
		if (len >= fifo_space) {
			len -= fifo_space;
			tx_len = fifo_space;
		} else {
			tx_len = len;
			len = 0;
		}
		// Signal FIFO to send data
		cc2400_fifo_write(tx_len, txbuf + i);
		cc2400_strobe(STX);
		i += tx_len;
		// From now on, we only get interrupt on GIO6, which means that we work according to FIFO_THRESHOLD now, the fifo is only half full.
		fifo_space = 16;
	}
	while ((cc2400_get(FSMSTATE) & 0x1f) != STATE_STROBE_FS_ON);
	TXLED_CLR;

	cc2400_strobe(SRFOFF);
	cc2400_wait_status(FS_LOCK, 0);

#ifdef UBERTOOTH_ONE
	PAEN_CLR;
#endif
	// Restore GIO
 	cc2400_set(IOCFG, gio_save);
}
#endif

#ifdef TX_ENABLE
void le_jam(void) {
	cc2400_set(MANAND,  0x7fff);
	cc2400_set(LMTST,   0x2b22);    // LNA and receive mixers test register
	cc2400_set(MDMTST0, 0x234b);    // PRNG, 1 MHz offset

	cc2400_set(GRMDM,   0x0c01);
	// 0 00 01 1 000 00 0 00 0 1
	//      |  | |   |  +--------> CRC off
	//      |  | |   +-----------> sync word: 8 MSB bits of SYNC_WORD
	//      |  | +---------------> 0 preamble bytes of 01010101
	//      |  +-----------------> packet mode
	//      +--------------------> buffered mode

	// cc2400_set(FSDIV,   channel);
	cc2400_set(FREND,   0b1011);    // amplifier level (-7 dBm, picked from hat)
	cc2400_set(MDMCTRL, 0x0040);    // 250 kHz frequency deviation

	cc2400_wait_status(XOSC16M_STABLE, 1);
	cc2400_strobe(SFSON);
	cc2400_wait_status(FS_LOCK, 1);
	TXLED_SET;
#ifdef UBERTOOTH_ONE
	PAEN_SET;
#endif
	while ((cc2400_get(FSMSTATE) & 0x1f) != STATE_STROBE_FS_ON);
	cc2400_strobe(STX);
}
#endif

/* TODO - return whether hop happened, or should caller have to keep
 * track of this? */
void hop(void)
{
	do_hop = 0;
	last_hop = clkn;

	// No hopping, if channel is set correctly, do nothing
	if (hop_mode == HOP_NONE) {
		if (cc2400_get(FSDIV) == (channel - 1))
			return;
	}
	/* Slow sweep (100 hops/sec)
	 * only hop to currently used channels if AFH is enabled
	 */
	else if (hop_mode == HOP_SWEEP) {
		/* at most one full cycle (gcd(32, 79) == 1 visits all 79) */
		for (int n = 0; n < NUM_BREDR_CHANNELS; n++) {
			channel += 32;
			if (channel > 2480)
				channel -= 79;
			if (!(used_channels != 0 && afh_enabled && !afh_channel_used(channel)))
				break;
		}
	}

	/* AFH detection
	 * only hop to currently unused channels
	 */
	else if (hop_mode == HOP_AFH) {
		for (int n = 0; n < NUM_BREDR_CHANNELS; n++) {
			channel += 32;
			if (channel > 2480)
				channel -= 79;
			if (!(used_channels < NUM_BREDR_CHANNELS && afh_channel_used(channel)))
				break;
		}
	}

	else if (hop_mode == HOP_BLUETOOTH) {
		uint16_t next = next_hop(clkn);
		/* with AFH both slots of a pair use the same channel: no need
		 * to retune (and lose the start of the slave slot) */
		if (next == channel && cc2400_get(FSDIV) == (channel - 1))
			return;
		channel = next;
	}

	else if (hop_mode == HOP_BTLE) {
		channel = btle_next_hop(&le);
	}

	else if (hop_mode == HOP_DIRECT) {
		channel = hop_direct_channel;
	}
	/* IDLE mode, but leave amp on, so don't call cc2400_idle(). */
	cc2400_strobe(SRFOFF);
	cc2400_wait_status(FS_LOCK, 0); // need to wait for unlock?

	/* Retune */
	if(mode == MODE_TX_SYMBOLS)
		cc2400_set(FSDIV, channel);
	else
		cc2400_set(FSDIV, channel - 1);

	/* Update CS register if hopping.  */
	if (hop_mode > 0) {
		cs_threshold_calc_and_set(channel);
	}

	/* Wait for lock */
	cc2400_strobe(SFSON);
	cc2400_wait_status(FS_LOCK, 1);

	dma_discard = 1;

	if(mode == MODE_TX_SYMBOLS) {
#ifdef TX_ENABLE
		cc2400_strobe(STX);
#endif
	} else {
		cc2400_strobe(SRX);
	}
}

/* modes served by bt_stream_rx() that can be switched between without
 * leaving it (e.g. RX_SYMBOLS -> BT_FOLLOW keeps the clock running) */
static int bt_stream_mode(uint8_t m)
{
	return m == MODE_RX_SYMBOLS || m == MODE_BT_FOLLOW || m == MODE_AFH;
}

/* should bt_stream_rx(), entered in entry_mode, keep running? */
static int bt_stream_continue(uint8_t entry_mode)
{
	uint8_t r = requested_mode;

	if (r == entry_mode)
		return 1;
	if (bt_stream_mode(entry_mode) && bt_stream_mode(r)) {
		mode = r;
		return 1;
	}
	return 0;
}

/* Bluetooth packet monitoring */
void bt_stream_rx()
{
	int8_t rssi;
	int8_t rssi_at_trigger;
	const uint8_t entry_mode = mode;
	uint8_t buf[DMA_SIZE];
	uint8_t buf_clkn_high;
	uint32_t buf_clk100ns;
	uint16_t buf_channel;
	uint32_t tc, err, primask;

	RXLED_CLR;

	usb_queue_init();
	dio_ssp_init();
	dma_init_rx_symbols();
	dio_ssp_start();

	cc2400_rx();

	cs_trigger_enable();

	while (bt_stream_continue(entry_mode))
	{

		RXLED_CLR;

		/* Wait for DMA transfer. TODO - need more work on
		 * RSSI. Should send RSSI indications to host even
		 * when not transferring data. That would also keep
		 * the USB stream going. This loop runs 50-80 times
		 * while waiting for DMA, but RSSI sampling does not
		 * cover all the symbols in a DMA transfer. Can not do
		 * RSSI sampling in CS interrupt, but could log time
		 * at multiple trigger points there. The MAX() below
		 * helps with statistics in the case that cs_trigger
		 * happened before the loop started. */
		rssi_reset();
		rssi_at_trigger = INT8_MIN;
		while (!rx_tc) {
			/* Without a carrier sense trigger (follow mode) no DMA
			 * may ever complete: honor mode changes while waiting. */
			if (!bt_stream_continue(entry_mode))
				goto out;

			rssi = (int8_t)(cc2400_get(RSSI) >> 8);
			if (cs_trigger && (rssi_at_trigger == INT8_MIN)) {
				rssi = MAX(rssi,(cs_threshold_cur+54));
				rssi_at_trigger = rssi;
			}
			rssi_add(rssi);

			handle_usb(clkn);

			/* If timer says time to hop, do it. */
			if (do_hop) {
				hop();
			} else {
				TXLED_CLR;
			}
			/* TODO - set per-channel carrier sense threshold.
			 * Set by firmware or host. */
		}

		RXLED_SET;

		/* Take the completed buffer and its metadata in one go, so a
		 * DMA completion arriving meanwhile can neither swap the buffer
		 * under us nor be lost by clearing rx_tc later. */
		primask = irq_save();
		tc = rx_tc;
		err = rx_err;
		rx_tc = 0;
		rx_err = 0;
		memcpy(buf, (const uint8_t *)idle_rxbuf, DMA_SIZE);
		buf_clkn_high = idle_buf_clkn_high;
		buf_clk100ns = idle_buf_clk100ns;
		buf_channel = idle_buf_channel;
		irq_restore(primask);

		if (err) {
			status |= DMA_ERROR;
		}

		/* Missed a DMA trasfer? */
		if (tc > 1)
			status |= DMA_OVERFLOW;

		if (dma_discard) {
			status |= DISCARD;
			dma_discard = 0;
		}

		rssi_iir_update(buf_channel);

		/* Set squelch hold if there was either a CS trigger, squelch
		 * is disabled, or if the current rssi_max is above the same
		 * threshold. Currently, this is redundant, but allows for
		 * per-channel or other rssi triggers in the future. */
		if (cs_trigger || cs_no_squelch) {
			status |= CS_TRIGGER;
			cs_trigger = 0;
		}

		if (rssi_max >= (cs_threshold_cur + 54)) {
			status |= RSSI_TRIGGER;
		}

		enqueue_meta(BR_PACKET, buf, buf_clkn_high, buf_clk100ns, buf_channel);

		handle_usb(clkn);
	}

out:
	dio_ssp_stop();
	cs_trigger_disable();
}

static uint8_t reverse8(uint8_t data)
{
	uint8_t reversed = 0;

	for(size_t i=0; i<8; i++)
	{
		reversed |= ((data >> i) & 0x01) << (7-i);
	}

	return reversed;
}

static uint16_t reverse16(uint16_t data)
{
	uint16_t reversed = 0;

	for(size_t i=0; i<16; i++)
	{
		reversed |= ((data >> i) & 0x01) << (15-i);
	}

	return reversed;
}

/*
 * Transmit a BTBR packet with the specified access code.
 *
 * All modulation parameters are set within this function.
 */
#ifdef TX_ENABLE
void br_transmit()
{
	uint16_t gio_save;

	uint32_t clkn_saved = 0;

	uint16_t preamble = (target.syncword & 1) == 1 ? 0x5555 : 0xaaaa;
	uint8_t trailer = ((target.syncword >> 63) & 1) == 1 ? 0xaa : 0x55;

	uint8_t data[16] = {
		reverse8((target.syncword >> 0) & 0xFF),
		reverse8((target.syncword >> 8) & 0xFF),
		reverse8((target.syncword >> 16) & 0xFF),
		reverse8((target.syncword >> 24) & 0xFF),
		reverse8((target.syncword >> 32) & 0xFF),
		reverse8((target.syncword >> 40) & 0xFF),
		reverse8((target.syncword >> 48) & 0xFF),
		reverse8((target.syncword >> 56) & 0xFF),
		reverse8(trailer),
		reverse8(0x77),
		reverse8(0x66),
		reverse8(0x55),
		reverse8(0x44),
		reverse8(0x33),
		reverse8(0x22),
		reverse8(0x11)
	};

	cc2400_tx_sync(reverse16(preamble));

	cc2400_set(INT,     0x0014);    // FIFO_THRESHOLD: 20 bytes

	// set GIO to FIFO_FULL
	gio_save = cc2400_get(IOCFG);
	cc2400_set(IOCFG, (GIO_FIFO_FULL << 9) | (gio_save & 0x1ff));

	while ( requested_mode == MODE_TX_SYMBOLS )
	{

		while ((clkn >> 1) == (clkn_saved >> 1) || T0TC < 2250) {

			// If timer says time to hop, do it.
			if (do_hop) {
				hop();
			}
		}

		clkn_saved = clkn;

		TXLED_SET;

		cc2400_fifo_write(16, data);

		while ((cc2400_get(FSMSTATE) & 0x1f) != STATE_STROBE_FS_ON);
		TXLED_CLR;

		cc2400_strobe(SRFOFF);
		cc2400_wait_status(FS_LOCK, 0);

		cc2400_wait_status(XOSC16M_STABLE, 1);
		cc2400_strobe(SFSON);
		cc2400_wait_status(FS_LOCK, 1);

		while ((cc2400_get(FSMSTATE) & 0x1f) != STATE_STROBE_FS_ON);
		cc2400_strobe(STX);

		handle_usb(clkn);
	}

#ifdef UBERTOOTH_ONE
	PAEN_CLR;
#endif

	// reset GIO
	cc2400_set(IOCFG, gio_save);
}
#endif

/* set LE access address */
static void le_set_access_address(u32 aa) {
	u32 aa_rev;

	le.access_address = aa;
	aa_rev = rbit(aa);
	le.syncl = aa_rev & 0xffff;
	le.synch = aa_rev >> 16;
}

/* reset le state, called by bt_generic_le and bt_follow_le() */
void reset_le() {
	le_set_access_address(0x8e89bed6);     // advertising channel access address
	le.crc_init  = 0x555555;               // advertising channel CRCInit
	le.crc_init_reversed = 0xAAAAAA;
	le.crc_verify = 0;
	le.last_packet = 0;

	le.link_state = LINK_INACTIVE;

	le.channel_idx = 0;
	le.channel_increment = 0;

	le.conn_epoch = 0;
	le.interval_timer = 0;
	le.conn_interval = 0;
	le.conn_interval = 0;
	le.conn_count = 0;

	le.win_size = 0;
	le.win_offset = 0;

	le.update_pending = 0;
	le.update_instant = 0;
	le.interval_update = 0;
	le.win_size_update = 0;
	le.win_offset_update = 0;

	do_hop = 0;
}

// reset LE Promisc state
void reset_le_promisc(void) {
	memset(&le_promisc, 0, sizeof(le_promisc));
	le_promisc.smallest_hop_interval = 0xffffffff;
}

/* generic le mode */
void bt_generic_le(u8 active_mode)
{
	u8 hold;
	int i, j;
	int8_t rssi, rssi_at_trigger;

	modulation = MOD_BT_LOW_ENERGY;
	mode = active_mode;

	reset_le();

	// enable USB interrupts
	ISER0 = ISER0_ISE_USB;

	RXLED_CLR;

	usb_queue_init();
	dio_ssp_init();
	dma_init_rx_symbols();
	dio_ssp_start();
	cc2400_rx();

	cs_trigger_enable();

	hold = 0;

	while (requested_mode == active_mode) {
		if (requested_channel != 0) {
			cc2400_strobe(SRFOFF);
			cc2400_wait_status(FS_LOCK, 0); // need to wait for unlock?

			/* Retune */
			cc2400_set(FSDIV, channel - 1);

			/* Wait for lock */
			cc2400_strobe(SFSON);
			cc2400_wait_status(FS_LOCK, 1);

			/* RX mode */
			cc2400_strobe(SRX);

			requested_channel = 0;
		}

		if (do_hop) {
			hop();
		} else {
			TXLED_CLR;
		}

		RXLED_CLR;

		/* Wait for DMA. Meanwhile keep track of RSSI. */
		rssi_reset();
		rssi_at_trigger = INT8_MIN;
		while ((rx_tc == 0) && (rx_err == 0) && requested_mode == active_mode)
		{
			rssi = (int8_t)(cc2400_get(RSSI) >> 8);
			if (cs_trigger && (rssi_at_trigger == INT8_MIN)) {
				rssi = MAX(rssi,(cs_threshold_cur+54));
				rssi_at_trigger = rssi;
			}
			rssi_add(rssi);
		}

		if (rx_err) {
			status |= DMA_ERROR;
		}

		/* No DMA transfer? */
		if (!rx_tc)
			goto rx_continue;

		/* Missed a DMA trasfer? */
		if (rx_tc > 1)
			status |= DMA_OVERFLOW;

		rssi_iir_update(channel);

		/* Set squelch hold if there was either a CS trigger, squelch
		 * is disabled, or if the current rssi_max is above the same
		 * threshold. Currently, this is redundant, but allows for
		 * per-channel or other rssi triggers in the future. */
		if (cs_trigger || cs_no_squelch) {
			status |= CS_TRIGGER;
			hold = CS_HOLD_TIME;
			cs_trigger = 0;
		}

		if (rssi_max >= (cs_threshold_cur + 54)) {
			status |= RSSI_TRIGGER;
			hold = CS_HOLD_TIME;
		}

		/* Hold expired? Ignore data. */
		if (hold == 0) {
			goto rx_continue;
		}
		hold--;

		// copy the previously unpacked symbols to the front of the buffer
		memcpy(unpacked, unpacked + DMA_SIZE*8, DMA_SIZE*8);

		// unpack the new packet to the end of the buffer
		for (i = 0; i < DMA_SIZE; ++i) {
			/* output one byte for each received symbol (0x00 or 0x01) */
			for (j = 0; j < 8; ++j) {
				unpacked[DMA_SIZE*8 + i * 8 + j] = (idle_rxbuf[i] & 0x80) >> 7;
				idle_rxbuf[i] <<= 1;
			}
		}

		int ret = data_cb(unpacked);
		if (!ret) break;

	rx_continue:
		rx_tc = 0;
		rx_err = 0;
	}

	// disable USB interrupts
	ICER0 = ICER0_ICE_USB;

	// turn the radio off (keeps requested_mode and host settings)
	cc2400_radio_off();
	dio_ssp_stop();
	cs_trigger_disable();
}


void bt_le_sync(u8 active_mode)
{
	int i;
	int8_t rssi;
	static int restart_jamming = 0;

	modulation = MOD_BT_LOW_ENERGY;
	mode = active_mode;

	le.link_state = LINK_LISTENING;

	// enable USB interrupts
	ISER0 = ISER0_ISE_USB;

	RXLED_CLR;

	usb_queue_init();
	dio_ssp_init();
	dma_init_le();
	dio_ssp_start();

	cc2400_rx_sync(rbit(le.access_address)); // bit-reversed access address

	while (requested_mode == active_mode) {
		if (requested_channel != 0) {
			cc2400_strobe(SRFOFF);
			cc2400_wait_status(FS_LOCK, 0); // need to wait for unlock?

			/* Retune */
			cc2400_set(FSDIV, channel - 1);

			/* Wait for lock */
			cc2400_strobe(SFSON);
			cc2400_wait_status(FS_LOCK, 1);

			/* RX mode */
			cc2400_strobe(SRX);

			requested_channel = 0;
		}

		RXLED_CLR;

		/* Wait for DMA. Meanwhile keep track of RSSI. */
		rssi_reset();
		while ((rx_tc == 0) && (rx_err == 0) && (do_hop == 0) && requested_mode == active_mode)
			;

		rssi = (int8_t)(cc2400_get(RSSI) >> 8);
		rssi_min = rssi_max = rssi;

		if (requested_mode != active_mode) {
			goto cleanup;
		}

		if (rx_err) {
			status |= DMA_ERROR;
		}

		if (do_hop)
			goto rx_flush;

		/* No DMA transfer? */
		if (!rx_tc)
			continue;

		/////////////////////
		// process the packet

		uint32_t packet[48/4+1] = { 0, };
		u8 *p = (u8 *)packet;
		packet[0] = le.access_address;

		const uint32_t *whit = whitening_word[btle_channel_index(channel)];
		for (i = 0; i < 4; i+= 4) {
			uint32_t v = rxbuf1[i+0] << 24
					   | rxbuf1[i+1] << 16
					   | rxbuf1[i+2] << 8
					   | rxbuf1[i+3] << 0;
			packet[i/4+1] = rbit(v) ^ whit[i/4];
		}

		unsigned len = p[5] + 2; // 8 bit length since BT 4.2
		if (len > 39)
			goto rx_flush;

		// transfer the minimum number of bytes from the CC2400
		// this allows us enough time to resume RX for subsequent packets on the same channel
		unsigned total_transfers = ((len + 3) + 4 - 1) / 4;
		if (total_transfers < 11) {
			while (DMACC0DestAddr < (uint32_t)rxbuf1 + 4 * total_transfers && rx_err == 0)
				;
		} else { // max transfers? just wait till DMA's done
			while (DMACC0Config & DMACCxConfig_E && rx_err == 0)
				;
		}
		DIO_SSP_DMACR &= ~SSPDMACR_RXDMAE;

		// strobe SFSON to allow the resync to occur while we process the packet
		cc2400_strobe(SFSON);

		// unwhiten the rest of the packet
		for (i = 4; i < 44; i += 4) {
			uint32_t v = rxbuf1[i+0] << 24
					   | rxbuf1[i+1] << 16
					   | rxbuf1[i+2] << 8
					   | rxbuf1[i+3] << 0;
			packet[i/4+1] = rbit(v) ^ whit[i/4];
		}

		if (le.crc_verify) {
			u32 calc_crc = btle_crcgen_lut(le.crc_init_reversed, p + 4, len);
			u32 wire_crc = (p[4+len+2] << 16)
						 | (p[4+len+1] << 8)
						 | (p[4+len+0] << 0);
			if (calc_crc != wire_crc) // skip packets with a bad CRC
				goto rx_flush;
		}


		RXLED_SET;
		packet_cb((uint8_t *)packet);

		// disable USB interrupts while we touch USB data structures
		ICER0 = ICER0_ICE_USB;
		enqueue(LE_PACKET, (uint8_t *)packet);
		ISER0 = ISER0_ISE_USB;

		le.last_packet = CLK100NS;

	rx_flush:
		// this might happen twice, but it's safe to do so
		cc2400_strobe(SFSON);

		// flush any excess bytes from the SSP's buffer
		DIO_SSP_DMACR &= ~SSPDMACR_RXDMAE;
		while (SSP1SR & SSPSR_RNE) {
			u8 tmp = (u8)DIO_SSP_DR;
		}

		// timeout - FIXME this is an ugly hack
		u32 now = CLK100NS;
		if (now < le.last_packet)
			now += 3276800000; // handle rollover
		if  ( // timeout
			((le.link_state == LINK_CONNECTED || le.link_state == LINK_CONN_PENDING)
			&& (now - le.last_packet > 50000000))
			// jam finished
			|| (le_jam_count == 1)
			)
		{
			reset_le();
			le_jam_count = 0;
			TXLED_CLR;

			if (jam_mode == JAM_ONCE) {
				jam_mode = JAM_NONE;
				requested_mode = MODE_IDLE;
				goto cleanup;
			}

			// go back to promisc if the connection dies
			if (active_mode == MODE_BT_PROMISC_LE)
				goto cleanup;

			le.link_state = LINK_LISTENING;

			cc2400_strobe(SRFOFF);
			cc2400_wait_status(FS_LOCK, 0);

			/* Retune */
			channel = le_adv_channel != 0 ? le_adv_channel : 2402;
			restart_jamming = 1;
		}

		cc2400_set(SYNCL, le.syncl);
		cc2400_set(SYNCH, le.synch);

		if (do_hop)
			hop();

		// ♪ you can jam but you keep turning off the light ♪
		if (le_jam_count > 0) {
#ifdef TX_ENABLE
			le_jam();
#endif
			--le_jam_count;
		} else {
			/* RX mode */
			dma_init_le();
			dio_ssp_start();

			if (restart_jamming) {
				cc2400_rx_sync(rbit(le.access_address));
				restart_jamming = 0;
			} else {
				// wait till we're in FSLOCK before strobing RX
				cc2400_wait_status(FS_LOCK, 1);
				cc2400_strobe(SRX);
			}
		}

		rx_tc = 0;
		rx_err = 0;
	}

cleanup:

	// disable USB interrupts
	ICER0 = ICER0_ICE_USB;

	// turn the radio off (keeps requested_mode and host settings)
	cc2400_radio_off();
	dio_ssp_stop();
	cs_trigger_disable();
}



/* low energy connection following
 * follows a known AA around */
int cb_follow_le() {
	int i, j, k;
	int idx = whitening_index[btle_channel_index(channel)];

	u32 access_address = 0;
	for (i = 0; i < 31; ++i) {
		access_address >>= 1;
		access_address |= (unpacked[i] << 31);
	}

	for (i = 31; i < DMA_SIZE * 8 + 32; i++) {
		access_address >>= 1;
		access_address |= (unpacked[i] << 31);
		if (access_address == le.access_address) {
			for (j = 0; j < 46; ++j) {
				u8 byte = 0;
				for (k = 0; k < 8; k++) {
					int offset = k + (j * 8) + i - 31;
					if (offset >= DMA_SIZE*8*2) break;
					int bit = unpacked[offset];
					if (j >= 4) { // unwhiten data bytes
						bit ^= whitening[idx];
						idx = (idx + 1) % sizeof(whitening);
					}
					byte |= bit << k;
				}
				idle_rxbuf[j] = byte;
			}

			// verify CRC
			if (le.crc_verify) {
				int len		 = idle_rxbuf[5] + 2;
				if (len > 46 - 4 - 3)
					break;
				u32 calc_crc = btle_crcgen_lut(le.crc_init_reversed, (uint8_t*)idle_rxbuf + 4, len);
				u32 wire_crc = (idle_rxbuf[4+len+2] << 16)
							 | (idle_rxbuf[4+len+1] << 8)
							 |  idle_rxbuf[4+len+0];
				if (calc_crc != wire_crc) // skip packets with a bad CRC
					break;
			}

			// send to PC
			enqueue(LE_PACKET, (uint8_t*)idle_rxbuf);
			RXLED_SET;

			packet_cb((uint8_t*)idle_rxbuf);

			break;
		}
	}

	return 1;
}

/**
 * Called when we receive a packet in connection following mode.
 */
void connection_follow_cb(u8 *packet) {
	int i;
	u32 aa = 0;

#define ADV_ADDRESS_IDX 0
#define HEADER_IDX 4
#define DATA_LEN_IDX 5
#define DATA_START_IDX 6

	// u8 *adv_addr = &packet[ADV_ADDRESS_IDX];
	u8 header = packet[HEADER_IDX];
	u8 *data_len = &packet[DATA_LEN_IDX];
	u8 *data = &packet[DATA_START_IDX];
	// u8 *crc = &packet[DATA_START_IDX + *data_len];

	if (le.link_state == LINK_CONN_PENDING) {
		// We received a packet in the connection pending state, so now the device *should* be connected
		le.link_state = LINK_CONNECTED;
		le.conn_epoch = clkn;
		le.interval_timer = le.conn_interval - 1;
		le.conn_count = 0;
		le.update_pending = 0;

		// hue hue hue
		if (jam_mode != JAM_NONE)
			le_jam_count = JAM_COUNT_DEFAULT;

	} else if (le.link_state == LINK_CONNECTED) {
		u8 llid =  header & 0x03;

		// Apply any connection parameter update if necessary
		if (le.update_pending && le.conn_count == le.update_instant) {
			// This is the first packet received in the connection interval for which the new parameters apply
			le.conn_epoch = clkn;
			le.conn_interval = le.interval_update;
			le.interval_timer = le.interval_update - 1;
			le.win_size = le.win_size_update;
			le.win_offset = le.win_offset_update;
			le.update_pending = 0;
		}

		if (llid == 0x03 && data[0] == 0x00 && packet[5] == 12 &&
				btle_crcgen_lut(le.crc_init_reversed, packet + 4, 14) ==
				((u32)packet[20] << 16 | (u32)packet[19] << 8 | packet[18])) {
			// This is a CONNECTION_UPDATE_REQ with a good CRC (packets
			// are not CRC checked while following: it may be forged).
			// The host is changing the connection parameters.
			u16 win_size = packet[7];
			u16 win_offset = packet[8] + ((u16)packet[9] << 8);
			u16 interval = packet[10] + ((u16)packet[11] << 8);
			u16 instant = packet[16] + ((u16)packet[17] << 8);
			if (interval >= 6 && interval <= 3200 && win_size >= 1 && win_size <= 8 &&
					win_size < interval && win_offset <= interval &&
					(u16)(instant - le.conn_count) < 32767) {
				le.win_size_update = win_size;
				le.win_offset_update = win_offset;
				le.interval_update = interval;
				le.update_instant = instant;
				le.update_pending = 1;
			}
		}

	} else if (le.link_state == LINK_LISTENING) {
		u8 pkt_type = packet[4] & 0x0F;
		if (pkt_type == 0x05) {
			uint16_t conn_interval;

			// ignore packets with incorrect length
			if (*data_len != 34)
				return;

			// conn interval must be [7.5 ms, 4.0s] in units of 1.25 ms
			conn_interval = (packet[29] << 8) | packet[28];
			if (conn_interval < 6 || conn_interval > 3200)
				return;

			// This is a connect packet
			// if we have a target, see if InitA or AdvA matches
			if (le.target_set &&
				memcmp(le.target, &packet[6], 6) &&  // Target address doesn't match Initiator.
				memcmp(le.target, &packet[12], 6)) {  // Target address doesn't match Advertiser.
				return;
			}

			le.link_state = LINK_CONN_PENDING;
			le.crc_verify = 0; // we will drop many packets if we attempt to filter by CRC

			for (i = 0; i < 4; ++i)
				aa |= packet[18+i] << (i*8);
			le_set_access_address(aa);

#define CRC_INIT (2+4+6+6+4)
			le.crc_init = (packet[CRC_INIT+2] << 16)
						| (packet[CRC_INIT+1] << 8)
						|  packet[CRC_INIT+0];
			le.crc_init_reversed = rbit(le.crc_init) >> 8; // 24 bit value

#define WIN_SIZE (2+4+6+6+4+3)
			le.win_size = packet[WIN_SIZE];

#define WIN_OFFSET (2+4+6+6+4+3+1)
			le.win_offset = packet[WIN_OFFSET];

#define CONN_INTERVAL (2+4+6+6+4+3+1+2)
			le.conn_interval = (packet[CONN_INTERVAL+1] << 8)
							 |  packet[CONN_INTERVAL+0];

#define CHANNEL_INC (2+4+6+6+4+3+1+2+2+2+2+5)
			le.channel_increment = packet[CHANNEL_INC] & 0x1f;
			le.channel_idx = le.channel_increment;

			// Hop to the initial channel immediately
			do_hop = 1;
		}
	}
}

void le_phy_main(void);
void bt_follow_le() {
	le_phy_main();

	/* old method
	reset_le();
	packet_cb = connection_follow_cb;
	bt_le_sync(MODE_BT_FOLLOW_LE);
	*/
}

// issue state change message
void le_promisc_state(u8 type, void *data, unsigned len) {
	u8 buf[50] = { 0, };
	if (len > 49)
		len = 49;

	buf[0] = type;
	memcpy(&buf[1], data, len);
	enqueue(LE_PROMISC, (uint8_t*)buf);
}

// divide, rounding to the nearest integer: round up at 0.5.
#define DIVIDE_ROUND(N, D) ((N) + (D)/2) / (D)

void promisc_recover_hop_increment(u8 *packet) {
	static u32 first_ts = 0;
	if (channel == 2404) {
		first_ts = CLK100NS;
		hop_direct_channel = 2406;
		do_hop = 1;
	} else if (channel == 2406) {
		u32 second_ts = CLK100NS;
		if (second_ts < first_ts)
			second_ts += 3276800000; // handle rollover
		// Number of channels hopped between previous and current timestamp.
		u32 channels_hopped = DIVIDE_ROUND(second_ts - first_ts,
										   le.conn_interval * LE_BASECLK);
		if (channels_hopped > 0 && channels_hopped < 37) {
			// Get the hop increment based on the number of channels hopped.
			le.channel_increment = hop_interval_lut[channels_hopped];
			le.interval_timer = le.conn_interval / 2;
			le.conn_count = 0;
			le.conn_epoch = 0;
			do_hop = 0;
			// Move on to regular connection following.
			le.channel_idx = (1 + le.channel_increment) % 37;
			le.link_state = LINK_CONNECTED;
			le.crc_verify = 0;
			hop_mode = HOP_BTLE;
			packet_cb = connection_follow_cb;
			le_promisc_state(3, &le.channel_increment, 1);

			if (jam_mode != JAM_NONE)
				le_jam_count = JAM_COUNT_DEFAULT;

			return;
		}
		hop_direct_channel = 2404;
		do_hop = 1;
	}
	else {
		hop_direct_channel = 2404;
		do_hop = 1;
	}
}

void promisc_recover_hop_interval(u8 *packet) {
	u32 prev_clk = le_promisc.prev_clk;
	u32 cur_clk = CLK100NS;
	if (cur_clk < prev_clk)
		cur_clk += 3276800000; // handle rollover
	u32 clk_diff = cur_clk - prev_clk;
	u16 obsv_hop_interval; // observed hop interval

	le_promisc.prev_clk = CLK100NS;

	// packets of the same connection event, or a first packet: a channel
	// recurs no sooner than 37 events of at least 7.5 ms
	if (prev_clk == 0 || clk_diff < 37 * 6 * LE_BASECLK)
		return;

	if (clk_diff < le_promisc.smallest_hop_interval)
		le_promisc.smallest_hop_interval = clk_diff;

	obsv_hop_interval = DIVIDE_ROUND(le_promisc.smallest_hop_interval, 37 * LE_BASECLK);
	if (obsv_hop_interval < 6 || obsv_hop_interval > 3200)
		return;

	if (le.conn_interval == obsv_hop_interval) {
		// 5 consecutive hop intervals: consider it legit and move on
		++le_promisc.consec_intervals;
		if (le_promisc.consec_intervals == 5) {
			packet_cb = promisc_recover_hop_increment;
			hop_direct_channel = 2404;
			hop_mode = HOP_DIRECT;
			do_hop = 1;
			le_promisc_state(2, &le.conn_interval, 2);
		}
	} else {
		le.conn_interval = obsv_hop_interval;
		le_promisc.consec_intervals = 0;
	}
}

// CRCInit candidate from empty PDUs: two must agree, since one forged or
// corrupted packet would make every later packet fail the CRC
static u32 crc_init_candidate;
static u8 crc_init_votes = 0;

void promisc_follow_cb(u8 *packet) {
	int i;

	// get the CRCInit
	if (!le.crc_verify && packet[4] == 0x01 && packet[5] == 0x00) {
		u32 crc = (packet[8] << 16) | (packet[7] << 8) | packet[6];
		u32 crc_init = btle_reverse_crc(crc, packet + 4, 2);

		if (crc_init_votes == 0 || crc_init != crc_init_candidate) {
			crc_init_candidate = crc_init;
			crc_init_votes = 1;
			return;
		}
		crc_init_votes = 0;
		le.crc_init = crc_init;
		le.crc_init_reversed = 0;
		for (i = 0; i < 24; ++i)
			le.crc_init_reversed |= ((le.crc_init >> i) & 1) << (23 - i);

		le.crc_verify = 1;
		packet_cb = promisc_recover_hop_interval;
		le_promisc_state(1, &le.crc_init, 3);
	}
}

// called when we see an AA, add it to the list
void see_aa(u32 aa, int8_t rssi) {
	int i, max = -1, killme = -1;
	for (i = 0; i < AA_LIST_SIZE; ++i)
		if (le_promisc.active_aa[i].count > 0 && le_promisc.active_aa[i].aa == aa) {
			++le_promisc.active_aa[i].count;
			if (rssi > le_promisc.active_aa[i].rssi_max)
				le_promisc.active_aa[i].rssi_max = rssi;
			return;
		}

	// evict someone
	for (i = 0; i < AA_LIST_SIZE; ++i)
		if (le_promisc.active_aa[i].count < max || max < 0) {
			killme = i;
			max = le_promisc.active_aa[i].count;
		}

	le_promisc.active_aa[killme].aa = aa;
	le_promisc.active_aa[killme].count = 1;
	le_promisc.active_aa[killme].rssi_max = rssi;
}

/* le promiscuous mode */
int cb_le_promisc(char *unpacked) {
	int i, j, k;
	int idx;

	// empty data PDU: 01 00
	char desired[4][16] = {
		{ 1, 0, 0, 0, 0, 0, 0, 0,
		  0, 0, 0, 0, 0, 0, 0, 0, },
		{ 1, 0, 0, 1, 0, 0, 0, 0,
		  0, 0, 0, 0, 0, 0, 0, 0, },
		{ 1, 0, 1, 0, 0, 0, 0, 0,
		  0, 0, 0, 0, 0, 0, 0, 0, },
		{ 1, 0, 1, 1, 0, 0, 0, 0,
		  0, 0, 0, 0, 0, 0, 0, 0, },
	};

	for (i = 0; i < 4; ++i) {
		idx = whitening_index[btle_channel_index(channel)];

		// whiten the desired data
		for (j = 0; j < (int)sizeof(desired[i]); ++j) {
			desired[i][j] ^= whitening[idx];
			idx = (idx + 1) % sizeof(whitening);
		}
	}

	// then look for that bitsream in our receive buffer. unpacked holds
	// the previous and the new DMA buffer: scan each absolute position
	// exactly once (a hit needs 32 bits before and 48 bits after it)
	for (i = DMA_SIZE*8 - 48; i < DMA_SIZE*8*2 - 48; i++) {
		int ok[4] = { 1, 1, 1, 1 };
		int matching = -1;

		for (j = 0; j < 4; ++j) {
			for (k = 0; k < (int)sizeof(desired[j]); ++k) {
				if (unpacked[i+k] != desired[j][k]) {
					ok[j] = 0;
					break;
				}
			}
		}

		// see if any match
		for (j = 0; j < 4; ++j) {
			if (ok[j]) {
				matching = j;
				break;
			}
		}

		// skip if no match
		if (matching < 0)
			continue;

		// found a match! unwhiten it and send it home
		idx = whitening_index[btle_channel_index(channel)];
		for (j = 0; j < 4+3+3; ++j) {
			u8 byte = 0;
			for (k = 0; k < 8; k++) {
				int offset = k + (j * 8) + i - 32;
				if (offset >= DMA_SIZE*8*2) break;
				int bit = unpacked[offset];
				if (j >= 4) { // unwhiten data bytes
					bit ^= whitening[idx];
					idx = (idx + 1) % sizeof(whitening);
				}
				byte |= bit << k;
			}
			idle_rxbuf[j] = byte;
		}

		u32 aa = (idle_rxbuf[3] << 24) |
				 (idle_rxbuf[2] << 16) |
				 (idle_rxbuf[1] <<  8) |
				 (idle_rxbuf[0]);
		// rssi_max is the strongest RSSI during this DMA buffer
		see_aa(aa, (int8_t)(rssi_max - 54));

		enqueue(LE_PACKET, (uint8_t*)idle_rxbuf);

	}

	// once an AA has been seen 4 times, strong enough, start following
	// it: the first one found, or the strongest with LE_PROMISC_STRONGEST
	int best = -1;
	for (i = 0; i < AA_LIST_SIZE; ++i) {
		le_promisc_active_aa_t *a = &le_promisc.active_aa[i];
		if (a->count <= 3 || a->rssi_max < le_promisc_min_rssi)
			continue;
		if (best < 0 || ((le_promisc_flags & LE_PROMISC_STRONGEST) &&
					a->rssi_max > le_promisc.active_aa[best].rssi_max))
			best = i;
		if (!(le_promisc_flags & LE_PROMISC_STRONGEST))
			break;
	}
	if (best >= 0) {
		le_set_access_address(le_promisc.active_aa[best].aa);
		le_promisc.aa_found = 1;
		le.crc_verify = 0;
		// quit using the old stuff and switch to sync mode
		return 0;
	}

	return 1;
}

void bt_promisc_le() {
	while (requested_mode == MODE_BT_PROMISC_LE) {
		reset_le_promisc();

		// jump to a random data channel and turn up the squelch
		if ((channel & 1) == 1)
			channel = 2440;

		// if the PC hasn't given us AA, determine by listening
		if (!le.aa_set) {
			// cs_threshold_req = -80;
			cs_threshold_calc_and_set(channel);
			data_cb = cb_le_promisc;
			bt_generic_le(MODE_BT_PROMISC_LE);
		}

		// could have got mode change in middle of above
		if (requested_mode != MODE_BT_PROMISC_LE)
			break;

		le_promisc_state(0, &le.access_address, 4);

		// the host recovers the rest (CSA#1 and CSA#2) with LE follow
		// mode's monitor and follow-params requests
		if (le_promisc_flags & LE_PROMISC_AA_ONLY) {
			requested_mode = MODE_IDLE;
			break;
		}
		packet_cb = promisc_follow_cb;
		le.crc_verify = 0;
		crc_init_votes = 0;
		bt_le_sync(MODE_BT_PROMISC_LE);
	}

	// a host-provided access address applies to this session only
	le.aa_set = 0;
}

#ifdef TX_ENABLE
void bt_slave_le() {
	u32 calc_crc;
	int i;
	// header (2) + AdvA (6) + AdvData (<= 31) + CRC (3)
	uint8_t adv_ind[2 + 6 + LE_ADV_DATA_MAX + 3] = { 0x00, };
	unsigned adv_ind_len;

	if (le_adv_len > LE_ADV_DATA_MAX) {
		requested_mode = MODE_IDLE;
		return;
	}

	adv_ind_len = 6 + le_adv_len;
	adv_ind[1] = adv_ind_len;

	// copy the user-specified mac address
	for (i = 0; i < 6; ++i)
		adv_ind[i+2] = slave_mac_address[5-i];

	// copy in the adv data
	memcpy(adv_ind + 2 + 6, le_adv_data, le_adv_len);

	// total: 2 + 6 + le_adv_len
	adv_ind_len += 2;

	// advertising channel PDUs always use CRCInit 0x555555 (bit-reversed)
	calc_crc = btle_calc_crc(0xAAAAAA, adv_ind, adv_ind_len);
	adv_ind[adv_ind_len + 0] = (calc_crc >>  0) & 0xff;
	adv_ind[adv_ind_len + 1] = (calc_crc >>  8) & 0xff;
	adv_ind[adv_ind_len + 2] = (calc_crc >> 16) & 0xff;

	clkn_start();

	// enable USB interrupts due to busy waits
	ISER0 = ISER0_ISE_USB;

	// spam advertising packets
	while (requested_mode == MODE_BT_SLAVE_LE) {
		le_transmit(0x8e89bed6, adv_ind_len+3, adv_ind);
		msleep(100);
	}

	// disable USB interrupts
	ICER0 = ICER0_ICE_USB;
}
#endif

void rx_generic_sync(void) {
	const u8 len = CC2400_FIFO_SIZE;
	/* enqueue() copies DMA_SIZE bytes: 4 syncword + 32 FIFO bytes + pad */
	u8 buf[DMA_SIZE] = { 0, };
	u16 reg_val;

	/* Put syncword at start of buffer
	 * DGS: fix this later, we don't know number of syncword bytes, etc
	 */
	reg_val = cc2400_get(SYNCH);
	buf[0] = (reg_val >> 8) & 0xFF;
	buf[1] = reg_val & 0xFF;
	reg_val = cc2400_get(SYNCL);
	buf[2] = (reg_val >> 8) & 0xFF;
	buf[3] = reg_val & 0xFF;

	usb_queue_init();
	clkn_start();

	cc2400_wait_status(XOSC16M_STABLE, 1);
	cc2400_strobe(SFSON);
	cc2400_wait_status(FS_LOCK, 1);
	RXLED_SET;
#ifdef UBERTOOTH_ONE
		PAEN_SET;
		HGM_SET;
#endif
	while (requested_mode == MODE_RX_GENERIC) {
		while ((cc2400_get(FSMSTATE) & 0x1f) != STATE_STROBE_FS_ON);
		cc2400_strobe(SRX);
		USRLED_CLR;
		/* keep servicing USB (and honor mode changes) while waiting */
		while (!(cc2400_status() & SYNC_RECEIVED)) {
			handle_usb(clkn);
			if (requested_mode != MODE_RX_GENERIC)
				return;
		}
		USRLED_SET;

		cc2400_fifo_read(len, buf+4);
		enqueue_meta(BR_PACKET, buf, (clkn >> 20) & 0xff, CLK100NS, channel);
		handle_usb(clkn);
	}
}

void rx_generic(void) {
	// Check for packet mode
	if(cc2400_get(GRMDM) & 0x0400) {
		rx_generic_sync();
	} else {
		modulation = MOD_NONE;
		bt_stream_rx();
	}
}

#ifdef TX_ENABLE
void tx_generic(void) {
	u16 synch, syncl;

	mode = MODE_TX_GENERIC;

	// Save existing syncword
	synch = cc2400_get(SYNCH);
	syncl = cc2400_get(SYNCL);

	cc2400_set(SYNCH, tx_pkt.synch);
	cc2400_set(SYNCL, tx_pkt.syncl);
	cc2400_set(MDMCTRL, 0x0057);
	cc2400_set(MDMTST0, 0x134b);
	cc2400_set(GRMDM, 0x0f61);
	cc2400_set(FSDIV, tx_pkt.channel);
	cc2400_set(FREND, tx_pkt.pa_level);

	cc2400_wait_status(XOSC16M_STABLE, 1);
	cc2400_strobe(SFSON);
	cc2400_wait_status(FS_LOCK, 1);
	TXLED_SET;
#ifdef UBERTOOTH_ONE
		PAEN_SET;
#endif
	while ((cc2400_get(FSMSTATE) & 0x1f) != STATE_STROBE_FS_ON);

	/* length was checked against the FIFO size by the vendor request */
	cc2400_fifo_write(MIN(tx_pkt.length, CC2400_FIFO_SIZE), tx_pkt.data);
	cc2400_strobe(STX);

	while ((cc2400_get(FSMSTATE) & 0x1f) != STATE_STROBE_FS_ON);
	TXLED_CLR;

	cc2400_strobe(SRFOFF);
	cc2400_wait_status(FS_LOCK, 0);

#ifdef UBERTOOTH_ONE
	PAEN_CLR;
#endif

	// Restore state
	cc2400_set(SYNCH, synch);
	cc2400_set(SYNCL, syncl);

	// resume the mode that was running before, unless the host has
	// requested something else in the meantime
	if (requested_mode == MODE_TX_GENERIC)
		requested_mode = last_mode;
}
#endif

/* spectrum analysis */
void specan()
{
	u16 f;
	u8 i = 0;
	u8 buf[DMA_SIZE];

	RXLED_SET;

	usb_queue_init();
	clkn_start();

#ifdef UBERTOOTH_ONE
	PAEN_SET;
	//HGM_SET;
#endif
	cc2400_set(LMTST,   0x2b22);
	cc2400_set(MDMTST0, 0x134b); // without PRNG
	cc2400_set(GRMDM,   0x0101); // un-buffered mode, GFSK
	cc2400_set(MDMCTRL, 0x0029); // 160 kHz frequency deviation
	//FIXME maybe set RSSI.RSSI_FILT
	cc2400_wait_status(XOSC16M_STABLE, 1);
	cc2400_wait_status(FS_LOCK, 0);

	while (requested_mode == MODE_SPECAN) {
		for (f = low_freq; f < high_freq + 1; f++) {
			cc2400_set(FSDIV, f - 1);
			cc2400_strobe(SFSON);
			cc2400_wait_status(FS_LOCK, 1);
			cc2400_strobe(SRX);

			/* give the CC2400 time to acquire RSSI reading */
			volatile u32 j = 500; while (--j); //FIXME crude delay
			buf[3 * i] = (f >> 8) & 0xFF;
			buf[(3 * i) + 1] = f  & 0xFF;
			buf[(3 * i) + 2] = cc2400_get(RSSI) >> 8;
			i++;
			if (i == 16) {
				enqueue(SPECAN, buf);
				i = 0;

				handle_usb(clkn);
			}

			cc2400_strobe(SRFOFF);
			cc2400_wait_status(FS_LOCK, 0);
		}
	}
	RXLED_CLR;
}

/* LED based spectrum analysis */
void led_specan()
{
	int8_t lvl;
	u8 i = 0;
	u16 channels[3] = {2412, 2437, 2462};
	//void (*set[3]) = {TXLED_SET, RXLED_SET, USRLED_SET};
	//void (*clr[3]) = {TXLED_CLR, RXLED_CLR, USRLED_CLR};

#ifdef UBERTOOTH_ONE
	PAEN_SET;
	//HGM_SET;
#endif
	cc2400_set(LMTST,   0x2b22);
	cc2400_set(MDMTST0, 0x134b); // without PRNG
	cc2400_set(GRMDM,   0x0101); // un-buffered mode, GFSK
	cc2400_set(MDMCTRL, 0x0029); // 160 kHz frequency deviation
	cc2400_set(RSSI,    0x00F1); // RSSI Sample over 2 symbols

	cc2400_wait_status(XOSC16M_STABLE, 1);
	cc2400_wait_status(FS_LOCK, 0);

	while (requested_mode == MODE_LED_SPECAN) {
		cc2400_set(FSDIV, channels[i] - 1);
		cc2400_strobe(SFSON);
		cc2400_wait_status(FS_LOCK, 1);
		cc2400_strobe(SRX);

		/* give the CC2400 time to acquire RSSI reading */
		volatile u32 j = 500; while (--j); //FIXME crude delay
		lvl = (int8_t)((cc2400_get(RSSI) >> 8) & 0xff);
		if (lvl > rssi_threshold) {
			switch (i) {
				case 0:
					TXLED_SET;
					break;
				case 1:
					RXLED_SET;
					break;
				case 2:
					USRLED_SET;
					break;
			}
		}
		else {
			switch (i) {
				case 0:
					TXLED_CLR;
					break;
				case 1:
					RXLED_CLR;
					break;
				case 2:
					USRLED_CLR;
					break;
			}
		}

		i = (i+1) % 3;

		handle_usb(clkn);

		cc2400_strobe(SRFOFF);
		cc2400_wait_status(FS_LOCK, 0);
	}
}

int main()
{
	// enable all fault handlers (see fault.c)
	SCB_SHCSR = (1 << 18) | (1 << 17) | (1 << 16);

	ubertooth_init();
	clkn_init();
	ubertooth_usb_init(vendor_request_handler);
	cc2400_idle();
	dma_poweron();

	debug_uart_init(0);
	debug_printf("\n\n****UBERTOOTH BOOT****\n%s\n", compile_info);

	while (1) {
		uint8_t req;

		handle_usb(clkn);

		req = requested_mode;
		if (req == mode)
			continue;

		if (req == MODE_IDLE) {
			cc2400_idle();
			continue;
		}

		/* mode is always the mode actually running */
		mode = req;

		switch (req) {
			case MODE_RESET:
				/* Allow time for the USB command to return correctly */
				wait(1);
				reset();
				break;
			case MODE_AFH:
			case MODE_RX_SYMBOLS:
			case MODE_BT_FOLLOW:
				bt_stream_rx();
				break;
#ifdef TX_ENABLE
			case MODE_TX_SYMBOLS:
				br_transmit();
				break;
#endif
			case MODE_BT_FOLLOW_LE:
				bt_follow_le();
				break;
			case MODE_BT_PROMISC_LE:
				bt_promisc_le();
				break;
#ifdef TX_ENABLE
			case MODE_BT_SLAVE_LE:
				bt_slave_le();
				break;
			case MODE_TX_TEST:
				cc2400_txtest(&modulation, &channel);
				/* the carrier stays on until another mode is requested */
				while (requested_mode == MODE_TX_TEST)
					handle_usb(clkn);
				break;
			case MODE_RANGE_TEST:
				cc2400_rangetest(&channel);
				if (requested_mode == MODE_RANGE_TEST)
					requested_mode = MODE_IDLE;
				break;
			case MODE_REPEATER:
				cc2400_repeater(&channel);
				break;
#endif
			case MODE_SPECAN:
				specan();
				break;
			case MODE_LED_SPECAN:
				led_specan();
				break;
			case MODE_EGO:
				ego_main(ego_mode);
				break;
			case MODE_RX_GENERIC:
				rx_generic();
				break;
#ifdef TX_ENABLE
			case MODE_TX_GENERIC:
				tx_generic();
				break;
#endif
			case MODE_XMAS:
				xmas_main();
				break;
			default:
				/* unknown mode: ignore the request */
				break;
		}

		/*
		 * The mode function has returned.  If nothing else has been
		 * requested meanwhile, the mode ended on its own: go idle
		 * rather than restarting it.
		 */
		if (requested_mode == req)
			requested_mode = MODE_IDLE;

		/* remember what ran, so tx_generic() can resume it */
		if (mode != MODE_TX_GENERIC)
			last_mode = mode;

		/* hop modes owned by the LE code must not leak into BR modes */
		if (hop_mode == HOP_BTLE || hop_mode == HOP_DIRECT)
			hop_mode = HOP_NONE;

		if (requested_mode == MODE_IDLE)
			cc2400_idle();
		else
			cc2400_radio_off();
		/* always: a stale mode makes re-requesting it a no-op */
		mode = MODE_IDLE;
	}
}
