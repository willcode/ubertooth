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
/*
  Copyright 2010-07 By Opendous Inc. (www.MicropendousX.org)
  NVIC handler info copied from NXP User Manual UM10360

  Start-up code for LPC17xx.  See TODOs for
  modification instructions.

  Permission to use, copy, modify, and distribute this software
  and its documentation for any purpose and without fee is hereby
  granted, provided that the above copyright notice appear in all
  copies and that both that the copyright notice and this
  permission notice and warranty disclaimer appear in supporting
  documentation, and that the name of the author not be used in
  advertising or publicity pertaining to distribution of the
  software without specific, written prior permission.

  The author disclaim all warranties with regard to this
  software, including all implied warranties of merchantability
  and fitness.  In no event shall the author be liable for any
  special, indirect or consequential damages or any damages
  whatsoever resulting from loss of use, data or profits, whether
  in an action of contract, negligence or other tortious action,
  arising out of or in connection with the use or performance of
  this software.
*/

#include <lpc17.h>

/* Reset_Handler variables defined in linker script */
extern unsigned long _interrupt_vector_table;
extern unsigned long _data;
extern unsigned long _edata;
extern unsigned long _etext;
extern unsigned long _bss;
extern unsigned long _ebss;
extern unsigned long _ahb_bss;
extern unsigned long _ahb_ebss;

/* constructor tables, defined in sections.ld */
typedef void (*init_fn_t)(void);
extern init_fn_t __preinit_array_start[];
extern init_fn_t __preinit_array_end[];
extern init_fn_t __init_array_start[];
extern init_fn_t __init_array_end[];

extern int main(void);

/* Reset Handler */
void Reset_Handler(void)
{
	unsigned long *src, *dest;

	// Copy the data segment initializers from flash to SRAM
	src = &_etext;
	for(dest = &_data; dest < &_edata; )
	{
		*dest++ = *src++;
	}

	// Initialize the .bss segment of memory to zeros
	src = &_bss;
	while (src < &_ebss)
	{
		*src++ = 0;
	}

	// Initialize the AHB SRAM .ahb_bss segment to zeros
	src = &_ahb_bss;
	while (src < &_ahb_ebss)
	{
		*src++ = 0;
	}

	// Run constructors.  Done here rather than with newlib's
	// __libc_init_array(), which needs crti's _init() and drags the
	// startup files (and newlib's stdio/atexit state) into the link.
	for (init_fn_t *fn = __preinit_array_start; fn < __preinit_array_end; fn++)
		(*fn)();
	for (init_fn_t *fn = __init_array_start; fn < __init_array_end; fn++)
		(*fn)();

	// Set the vector table location.
	SCB_VTOR = (uint32_t)&_interrupt_vector_table;

	main();

	// In case main() fails, have something to breakpoint
	while (1) {;}
}
