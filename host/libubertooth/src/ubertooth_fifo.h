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

#ifndef __UBERTOOTH_FIFO_H__
#define __UBERTOOTH_FIFO_H__

#include <pthread.h>
#include <stddef.h>
#include "ubertooth_control.h"

/*
 * Single-producer / single-consumer ring of USB packets.
 *
 * The producer is the libusb event thread (or the main thread when
 * reading from a file), the consumer is the thread calling
 * ubertooth_bulk_receive().  The indices are accessed with
 * acquire/release atomics so the consumer never sees an index before
 * the packet it publishes.  FIFO_SIZE must be a power of two.
 *
 * 262144 packets (16 MiB) is 7 s of a 15-channel HackRF stream and
 * 100 s of an Ubertooth's: enough to ride out a stall such as the clock
 * search's rescoring without dropping packets.
 */
#define FIFO_SIZE 262144
#define FIFO_MASK (FIFO_SIZE - 1)

typedef struct {
	usb_pkt_rx packets[FIFO_SIZE];
	size_t read_ptr;           /* only accessed through __atomic builtins */
	size_t write_ptr;          /* only accessed through __atomic builtins */
	unsigned long overflows;   /* packets dropped because the ring was full */
	pthread_mutex_t lock;      /* protects the condition variable only */
	pthread_cond_t  cond;
} fifo_t;

fifo_t* fifo_init(void);
void fifo_free(fifo_t* fifo);
/* Discard all queued packets. Only call while no producer is running. */
void fifo_reset(fifo_t* fifo);

/* Publish the element returned by fifo_get_write_element().
 * Returns 0 (and counts an overflow) if the ring is full. */
int fifo_inc_write_ptr(fifo_t* fifo);

/* Copy a packet in and publish it. Returns 0 if the ring is full. */
int fifo_push(fifo_t* fifo, const usb_pkt_rx* packet);
/* Remove the oldest packet. Must only be called when !fifo_empty(). */
usb_pkt_rx fifo_pop(fifo_t* fifo);
usb_pkt_rx* fifo_get_write_element(fifo_t* fifo);

uint8_t fifo_empty(fifo_t* fifo);
/* Packets waiting (a snapshot). */
size_t fifo_count(fifo_t* fifo);
/* The consumer is falling behind: more than FIFO_BEHIND packets wait. */
#define FIFO_BEHIND (FIFO_SIZE / 16)

/* Block until the ring is non-empty, fifo_wake() is called or
 * timeout_ms elapses. Returns non-zero if a packet is available. */
int fifo_wait(fifo_t* fifo, unsigned timeout_ms);
/* Wake up a thread blocked in fifo_wait(). */
void fifo_wake(fifo_t* fifo);

#endif /* __UBERTOOTH_FIFO_H__ */
