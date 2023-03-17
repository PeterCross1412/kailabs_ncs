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
	if (hasChanged & BTN_ONBOARD_MSK) {
		if (buttonState & BTN_ONBOARD_MSK)
		{
			LOG_INF("Button Onboard press");
			// BSP::SetOutputs(DK_LED1_MSK);
			BSP::dk_set_led_on(LED_ONBOARD);
		}
		else
		{
			LOG_INF("Button Onboard release");
			// BSP::SetOutputs(DK_NO_LEDS_MSK);
			BSP::dk_set_led_off(LED_ONBOARD);
		}

		// BSP::SetOutput(LED_ONBOARD, (bool)(buttonState & BTN_ONBOARD_MSK));
	}
	else
	if (hasChanged & BTN_UP_MSK) {
		if (buttonState & BTN_UP_MSK)
		{
			LOG_INF("Button UP press");
		} else
		{
			LOG_INF("Button UP release");
		}

		BSP::SetOutput(RELAY_UP, (bool)(buttonState & RELAY_UP_MSK));
	}
	else
	if (hasChanged & BTN_DOWN_MSK) {
		if (buttonState & BTN_DOWN_MSK)
		{
			LOG_INF("Button DOWN press");
		} else
		{
			LOG_INF("Button DOWN release");
		}

		BSP::SetOutput(RELAY_DOWN, (bool)(buttonState & RELAY_DOWN_MSK));
	}
	else
	if (hasChanged & BTN_STOP_MSK) {
		if (buttonState & BTN_STOP_MSK)
		{
			LOG_INF("Button STOP press");
		} else
		{
			LOG_INF("Button STOP release");
		}

		BSP::SetOutput(RELAY_STOP, (bool)(buttonState & RELAY_STOP_MSK));
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
