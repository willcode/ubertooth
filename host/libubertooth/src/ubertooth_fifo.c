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

#include "ubertooth_fifo.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

fifo_t* fifo_init(void)
{
	pthread_condattr_t attr;
	fifo_t* fifo = (fifo_t*)malloc(sizeof(fifo_t));

	if (fifo == NULL)
		return NULL;

	fifo->read_ptr = 0;
	fifo->write_ptr = 0;
	fifo->overflows = 0;

	pthread_mutex_init(&fifo->lock, NULL);
	pthread_condattr_init(&attr);
	pthread_condattr_setclock(&attr, CLOCK_MONOTONIC);
	pthread_cond_init(&fifo->cond, &attr);
	pthread_condattr_destroy(&attr);

	return fifo;
}

void fifo_free(fifo_t* fifo)
{
	if (fifo == NULL)
		return;
	pthread_cond_destroy(&fifo->cond);
	pthread_mutex_destroy(&fifo->lock);
	free(fifo);
}

void fifo_reset(fifo_t* fifo)
{
	size_t w = __atomic_load_n(&fifo->write_ptr, __ATOMIC_ACQUIRE);
	__atomic_store_n(&fifo->read_ptr, w, __ATOMIC_RELEASE);
}

void fifo_wake(fifo_t* fifo)
{
	pthread_mutex_lock(&fifo->lock);
	pthread_cond_broadcast(&fifo->cond);
	pthread_mutex_unlock(&fifo->lock);
}

int fifo_inc_write_ptr(fifo_t* fifo)
{
	size_t w = __atomic_load_n(&fifo->write_ptr, __ATOMIC_RELAXED);
	size_t next = (w + 1) & FIFO_MASK;

	if (next == __atomic_load_n(&fifo->read_ptr, __ATOMIC_ACQUIRE)) {
		fifo->overflows++;
		return 0;
	}

	/* release: the packet contents become visible before the index */
	__atomic_store_n(&fifo->write_ptr, next, __ATOMIC_RELEASE);
	fifo_wake(fifo);
	return 1;
}

uint8_t fifo_empty(fifo_t* fifo)
{
	return __atomic_load_n(&fifo->read_ptr, __ATOMIC_RELAXED) ==
	       __atomic_load_n(&fifo->write_ptr, __ATOMIC_ACQUIRE);
}

size_t fifo_count(fifo_t* fifo)
{
	return (__atomic_load_n(&fifo->write_ptr, __ATOMIC_ACQUIRE) -
	        __atomic_load_n(&fifo->read_ptr, __ATOMIC_RELAXED)) & FIFO_MASK;
}

int fifo_push(fifo_t* fifo, const usb_pkt_rx* packet)
{
	memcpy(fifo_get_write_element(fifo), packet, sizeof(usb_pkt_rx));
	return fifo_inc_write_ptr(fifo);
}

usb_pkt_rx fifo_pop(fifo_t* fifo)
{
	size_t r = __atomic_load_n(&fifo->read_ptr, __ATOMIC_RELAXED);
	usb_pkt_rx pkt = fifo->packets[r];

	/* release: we are done reading the slot before the producer may reuse it */
	__atomic_store_n(&fifo->read_ptr, (r + 1) & FIFO_MASK, __ATOMIC_RELEASE);
	return pkt;
}

usb_pkt_rx* fifo_get_write_element(fifo_t* fifo)
{
	return &fifo->packets[__atomic_load_n(&fifo->write_ptr, __ATOMIC_RELAXED)];
}

int fifo_wait(fifo_t* fifo, unsigned timeout_ms)
{
	struct timespec deadline;

	if (!fifo_empty(fifo))
		return 1;

	clock_gettime(CLOCK_MONOTONIC, &deadline);
	deadline.tv_sec += timeout_ms / 1000;
	deadline.tv_nsec += (long)(timeout_ms % 1000) * 1000000L;
	if (deadline.tv_nsec >= 1000000000L) {
		deadline.tv_sec++;
		deadline.tv_nsec -= 1000000000L;
	}

	pthread_mutex_lock(&fifo->lock);
	/* The producer publishes the index before taking the lock to
	 * signal, so checking under the lock cannot miss a wakeup. A wakeup
	 * without data (fifo_wake) also returns, so callers can re-check
	 * their stop flag. */
	if (fifo_empty(fifo))
		pthread_cond_timedwait(&fifo->cond, &fifo->lock, &deadline);
	pthread_mutex_unlock(&fifo->lock);

	return !fifo_empty(fifo);
}
