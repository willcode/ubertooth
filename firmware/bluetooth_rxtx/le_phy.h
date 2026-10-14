/*
 * Copyright 2017 Mike Ryan
 *
 * This file is part of Project Ubertooth and is released under the
 * terms of the GPL. Refer to COPYING for more information.
 */

#ifndef __LE_PHY_H__
#define __LE_PHY_H__

#include <stdint.h>
#include "ubertooth_interface.h"

void le_phy_main(void);

/*
 * Requests from the vendor request handler. They are applied by the
 * le_phy main loop, and are ignored unless LE follow mode is running.
 */
void le_phy_request_monitor(uint16_t channel_mhz, uint32_t access_address);
void le_phy_request_follow(const le_follow_params_t *params);
void le_phy_set_adv_config(uint16_t flags, uint16_t dwell_ms);
// The host uses the API 1.08 LE commands: send LE_STATE messages,
// LE_PACKET_CONT chunks and packet flags. Stock host tools (API 1.07)
// would read them as LE packets, so they are off until such a command,
// and again after UBERTOOTH_STOP.
void le_phy_set_host_ext(int on);

#endif /* __LE_PHY_H__ */
