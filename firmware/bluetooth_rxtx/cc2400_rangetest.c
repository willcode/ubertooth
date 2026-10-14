/*
 * Copyright 2013 Dominic Spill, Michael Ossmann
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

#include "cc2400_rangetest.h"
#include "ubertooth_usb.h"
#include "ubertooth_clock.h"

extern volatile uint8_t requested_mode;

/* how long cc2400_rangetest() waits for the repeated packet, in CLKN ticks */
#define RANGETEST_REPLY_TIMEOUT (2 * 3200) /* 2 s */

/*
 * This range test transmits a Bluetooth-like (but not Bluetooth compatible)
 * packet to a repeater and then receives the repeated packet.  It is only
 * useful if another device is running cc2400_repeater() some distance away.
 *
 * The outgoing packet is transmitted at each power amplifier (PA) level from
 * lowest to highest.  It is sent several times at each level.  The repeater
 * takes the first packet to be received correctly and repeats it in a similar
 * manner.  The result should indicate the lowest successful PA level in each
 * direction.
 *
 * The range test packet consists of:
 *   preamble: 4 bytes
 *   sync word: 4 bytes
 *   payload:
 *     length: 1 byte (21)
 *     packet type: 1 byte (0 = request; 1 = reply)
 *     LPC17xx serial number: 16 bytes
 *     request pa: 1 byte
 *     request number: 1 byte
 *     reply pa: 1 byte
 *     reply number: 1 byte
 *   crc: 2 bytes
 */

rangetest_result rr;

#ifdef TX_ENABLE
void cc2400_rangetest(volatile u16 *chan_ptr)
{
	u32 command[5];
	u32 result[5];
	int i;
	int j;
	u8 len = 22;
	u8 pa = 0;
	u8 txbuf[len];
	u8 rxbuf[len];
	u32 start;
	int got_reply = 0;

	rr.valid = 0;

	txbuf[0] = len - 1; // length of data (rest of payload)
	txbuf[1] = 0; // request

	// read device serial number
	command[0] = 58;
	iap_entry(command, result);
	if ((result[0] & 0xFF) != 0) //status check
		return;
	txbuf[2] = (result[1] >> 24) & 0xFF;
	txbuf[3] = (result[1] >> 16) & 0xFF;
	txbuf[4] = (result[1] >> 8) & 0xFF;
	txbuf[5] = result[1] & 0xFF;
	txbuf[6] = (result[2] >> 24) & 0xFF;
	txbuf[7] = (result[2] >> 16) & 0xFF;
	txbuf[8] = (result[2] >> 8) & 0xFF;
	txbuf[9] = result[2] & 0xFF;
	txbuf[10] = (result[3] >> 24) & 0xFF;
	txbuf[11] = (result[3] >> 16) & 0xFF;
	txbuf[12] = (result[3] >> 8) & 0xFF;
	txbuf[13] = result[3] & 0xFF;
	txbuf[14] = (result[4] >> 24) & 0xFF;
	txbuf[15] = (result[4] >> 16) & 0xFF;
	txbuf[16] = (result[4] >> 8) & 0xFF;
	txbuf[17] = result[4] & 0xFF;

	txbuf[18] = pa; // request pa
	txbuf[19] = 0; // request number
	txbuf[20] = 0xff; // reply pa
	txbuf[21] = 0xff; // reply number

	// Bluetooth-like modulation
	cc2400_set(LMTST,   0x2b22);
	cc2400_set(MDMTST0, 0x134b);
	cc2400_set(GRMDM,   0x0df1);  // default value
	cc2400_set(FSDIV,   *chan_ptr);
	cc2400_set(SYNCH,   0xf9ae);
	cc2400_set(SYNCL,   0x1584);
	cc2400_set(FREND,   8 | pa);
	cc2400_set(MDMCTRL, 0x0029);
	cc2400_wait_status(XOSC16M_STABLE, 1);
	cc2400_strobe(SFSON);
	cc2400_wait_status(FS_LOCK, 1);
	TXLED_SET;
#ifdef UBERTOOTH_ONE
	PAEN_SET;
#endif
	for (pa = 0; pa < 8; pa++) {
		cc2400_set(FREND, 8 | pa);
		txbuf[18] = pa;
		for (i = 0; i < 16; i++) {
			txbuf[19] = i;
			while ((cc2400_get(FSMSTATE) & 0x1f) != STATE_STROBE_FS_ON);
			// transmit a packet
			for (j = 0; j < len; j++)
				cc2400_set8(FIFOREG, txbuf[j]);
			cc2400_strobe(STX);
		}
	}
	// sent packet, now look for repeated packet
	while ((cc2400_get(FSMSTATE) & 0x1f) != STATE_STROBE_FS_ON);
	TXLED_CLR;
	cc2400_strobe(SRFOFF);
	cc2400_wait_status(FS_LOCK, 0);
	cc2400_set(FSDIV, *chan_ptr - 1);
	cc2400_wait_status(XOSC16M_STABLE, 1);
	cc2400_strobe(SFSON);
	cc2400_wait_status(FS_LOCK, 1);
	RXLED_SET;
	/* wait for the repeated packet, but not forever, and keep USB alive */
	clkn_start();
	start = clkn;
	while (!got_reply) {
		while ((cc2400_get(FSMSTATE) & 0x1f) != STATE_STROBE_FS_ON);
		cc2400_strobe(SRX);
		while (!(cc2400_status() & SYNC_RECEIVED)) {
			handle_usb(clkn);
			if (requested_mode != MODE_RANGE_TEST ||
					(u32)(clkn - start) > RANGETEST_REPLY_TIMEOUT)
				break;
		}
		if (!(cc2400_status() & SYNC_RECEIVED))
			break; /* timed out or canceled */
		USRLED_SET;
		for (j = 0; j < len; j++)
			rxbuf[j] = cc2400_get8(FIFOREG);
		if (cc2400_status() & STATUS_CRC_OK)
			got_reply = 1;
		USRLED_CLR;
	}

	// done
	cc2400_strobe(SRFOFF);
	cc2400_wait_status(FS_LOCK, 0);
#ifdef UBERTOOTH_ONE
	PAEN_CLR;
#endif
	RXLED_CLR;

	if (!got_reply)
		return; /* rr.valid stays 0 */

	// get test result
	rr.valid       = 1;
	rr.request_pa  = rxbuf[18];
	rr.request_num = rxbuf[19];
	rr.reply_pa    = rxbuf[20];
	rr.reply_num   = rxbuf[21];

	// make sure rx packet is as expected
	txbuf[1] = 1; // expected value in rxbuf
	for (i = 0; i < 18; i++)
		if (rxbuf[i] != txbuf[i])
			rr.valid = 2 + i;

	USRLED_CLR;
}
#endif

/* This is the repeater implementation to be used with cc2400_rangetest(). */
#ifdef TX_ENABLE
void cc2400_repeater(volatile u16 *chan_ptr)
{
	int i;
	int j;
	u8 len = 22;
	u8 pa = 0;
	u8 buf[len];

	clkn_start();

	while (requested_mode == MODE_REPEATER) {
		cc2400_set(LMTST,   0x2b22);
		cc2400_set(MDMTST0, 0x134b);
		cc2400_set(FSDIV,   *chan_ptr - 1);
		cc2400_set(SYNCH,   0xf9ae);
		cc2400_set(SYNCL,   0x1584);
		cc2400_set(FREND,   0x0008); // minimum tx power
		cc2400_set(MDMCTRL, 0x0029); // 160 kHz frequency deviation
		cc2400_wait_status(XOSC16M_STABLE, 1);
		cc2400_strobe(SFSON);
		cc2400_wait_status(FS_LOCK, 1);
		RXLED_SET;
		TXLED_CLR;
		USRLED_CLR;
#ifdef UBERTOOTH_ONE
		PAEN_SET;
#endif
		while (1) {
			while ((cc2400_get(FSMSTATE) & 0x1f) != STATE_STROBE_FS_ON);
			USRLED_CLR;
			cc2400_strobe(SRX);
			/* keep USB alive and allow the repeater to be stopped */
			while (!(cc2400_status() & SYNC_RECEIVED)) {
				handle_usb(clkn);
				if (requested_mode != MODE_REPEATER)
					return;
			}
			USRLED_SET;
			for (i = 0; i < len; i++)
				buf[i] = cc2400_get8(FIFOREG);
			if (cc2400_status() & STATUS_CRC_OK)
				break;
		}
		// got packet, now repeat it
		i = 2000000; while (--i); // allow time for requester to switch to rx
		USRLED_CLR;
		RXLED_CLR;
		cc2400_strobe(SRFOFF);
		cc2400_wait_status(FS_LOCK, 0);
		cc2400_wait_status(XOSC16M_STABLE, 1);
		cc2400_set(FSDIV, *chan_ptr);
		cc2400_wait_status(XOSC16M_STABLE, 1);
		cc2400_strobe(SFSON);
		TXLED_SET;
		buf[0] = len - 1; // length of data (rest of payload)
		buf[1] = 1; // reply
		for (pa = 0; pa < 8; pa++) {
			cc2400_set(FREND, 8 | pa);
			buf[20] = pa;
			for (i = 0; i < 16; i++) {
				buf[21] = i;
				while ((cc2400_get(FSMSTATE) & 0x1f) != STATE_STROBE_FS_ON);
				for (j = 0; j < len; j++)
					cc2400_set8(FIFOREG, buf[j]);
				cc2400_strobe(STX);
			}
		}
		TXLED_CLR;
		cc2400_strobe(SRFOFF);
		cc2400_wait_status(FS_LOCK, 0);
	}
}
#endif

#ifdef TX_ENABLE
void cc2400_txtest(volatile u8 *mod_ptr, volatile u16 *chan_ptr)
{
	u16 mdmctrl;
	if (*mod_ptr == MOD_BT_BASIC_RATE) {
		mdmctrl = 0x0029; // 160 kHz frequency deviation
	} else if (*mod_ptr == MOD_BT_LOW_ENERGY) {
		mdmctrl = 0x0040; // 250 kHz frequency deviation
	} else {
		/* oops */
		return;
	}
	cc2400_set(LMTST,   0x2b22);
	cc2400_set(MDMTST0, 0x334b); // with PRNG
	cc2400_set(GRMDM,   0x0df1); // default value
	cc2400_set(FSDIV,   *chan_ptr);
	cc2400_set(MDMCTRL, mdmctrl);

	cc2400_wait_status(XOSC16M_STABLE, 1);
	cc2400_strobe(SFSON);
	cc2400_wait_status(FS_LOCK, 1);
	TXLED_SET;
	cc2400_strobe(STX);
#ifdef UBERTOOTH_ONE
	PAEN_SET;
#endif
}
#endif
