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
#include "bsp.h"
#include <zephyr/logging/log.h>

#define LOG_MODULE_NAME main
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#define RUN_LED_BLINK_INTERVAL 1000


static void ButtonEventHandler(uint32_t buttonState, uint32_t hasChanged)
{
	if (hasChanged & DK_BTN1_MSK) {
		if (buttonState & DK_BTN1_MSK)
		{
			LOG_INF("Button 1 press");
			// BSP::SetOutputs(DK_LED1_MSK);
			BSP::dk_set_led_on(DK_LED1);
		}
		else
		{
			LOG_INF("Button 1 release");
			// BSP::SetOutputs(DK_NO_LEDS_MSK);
			BSP::dk_set_led_off(DK_LED1);
		}

		// BSP::SetOutput(DK_LED1, (bool)(hasChanged & DK_BTN1_MSK));
	}
	else
	if (hasChanged & DK_BTN2_MSK) {
		LOG_INF("Button 2");
	}
	else
	if (hasChanged & DK_BTN3_MSK) {
		LOG_INF("Button 3");
	}
}


int main(void)
{
	BSP::InitGpio();
	BSP::InitInputsAndButtons(ButtonEventHandler);

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

	LOG_INF("Initialized");

	for (;;) {
		// LOG_INF("Bluetooth initialized");
		k_msleep(RUN_LED_BLINK_INTERVAL);
	}
}
