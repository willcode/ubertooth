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

#ifndef __UBERTOOTH_CLOCK_H
#define __UBERTOOTH_CLOCK_H value

#include "inttypes.h"
#include "lpc17.h"

/*
 * CLK100NS is a free-running clock with a period of 100 ns.  It resets every
 * 2^15 * 10^5 cycles (about 5.5 minutes) - computed from clkn and timer0 (T0TC)
 *
 * clkn is the native (local) clock as defined in the Bluetooth specification.
 * It advances 3200 times per second.  Two clkn periods make a Bluetooth time
 * slot.
 */
extern volatile uint32_t clkn;
extern volatile uint32_t last_hop;

extern volatile uint32_t clkn_offset;
extern volatile uint16_t clk100ns_offset;

// linear clock drift
extern volatile int16_t clk_drift_ppm;
extern volatile uint16_t clk_drift_correction;

extern volatile uint32_t clkn_last_drift_fix;
extern volatile uint32_t clkn_next_drift_fix;

/*
 * Current time in 100 ns units: 3125 * clkn[19:0] + T0TC, wrapping at
 * 3276800000.  clkn and T0TC are read consistently:
 *  - retry if TIMER0_IRQHandler ran between the two reads;
 *  - if a tick has happened but its interrupt is still pending (caller
 *    runs at TIMER0's priority or above, or with interrupts masked),
 *    account for it;
 *  - T0TC exceeds 3124 only while a positive clock trim stretches the
 *    tick; clamp so the value never runs ahead of the next tick.
 */
static inline uint32_t clk100ns_read(void)
{
	uint32_t c, t, pend;

	do {
		c = clkn;
		pend = T0IR & TIR_MR0_Interrupt;
		t = T0TC;
	} while (c != clkn || pend != (T0IR & TIR_MR0_Interrupt));

	if (pend)
		c += 1;
	if (t > 3124)
		t = 3124;

	return 3125 * (c & 0xfffff) + t;
}

/*
 * The T0MR0 match value for the CLKN tick that has just started, from the
 * pending trim and the count already into the tick.  The tick's length in
 * 100 ns units:
 *   offset 0           -> 3125 (nominal)
 *   offset 1..3124     -> 3125 + offset (delay the clock)
 *   offset 3125..6249  -> 3125 - (6250 - offset) (advance it)
 * The match value is one less than the length and must stay ahead of the
 * count, so a shortened tick ends no sooner than 16 counts from now.  The
 * shortened length is computed signed: 3125 asks for a tick of length 0,
 * and 3124 - 3125 in unsigned arithmetic would wrap to 0xFFFFFFFF, pass
 * the clamp and stop the clock until the counter wraps (about 429 s).
 */
static inline uint32_t clkn_tick_match(uint32_t offset, uint32_t count)
{
	int32_t mr0 = 3124;

	if (offset > 0 && offset <= 3124) {
		mr0 = 3124 + offset;
	} else if (offset > 3124 && offset < 6250) {
		const int32_t min_mr0 = count + 16;
		mr0 = 3124 - (int32_t)(6250 - offset);
		if (mr0 < min_mr0)
			mr0 = min_mr0;
	}
	return mr0;
}

#define CLK100NS (clk100ns_read())
#define LE_BASECLK (12500)                    // 1.25 ms in units of 100ns

void clkn_stop();
void clkn_start();
void clkn_init();
void clkn_disable();

#endif
