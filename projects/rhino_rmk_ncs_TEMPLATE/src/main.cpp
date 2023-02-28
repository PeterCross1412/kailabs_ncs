/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief 
 */
// #include "uart_async_adapter.h"

#include <zephyr/types.h>
#include <zephyr/kernel.h>

#include <stdio.h>

#include <zephyr/logging/log.h>

#define LOG_MODULE_NAME main
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#define RUN_LED_BLINK_INTERVAL 1000


int main(void)
{
	k_msleep(1000);

	//TODO:
	// Read user data in flash
	// Read Pi list in flash
	// Read Pi list in flash

	// Turn ON wifi module power
	// Turn OFF user led
	// Disable wifi module (IO)

	// Button handler init
	// - Factory reset module wifi.
	// 

	for (;;) {
		LOG_INF("Bluetooth initialized");
		k_msleep(RUN_LED_BLINK_INTERVAL);
	}
}
