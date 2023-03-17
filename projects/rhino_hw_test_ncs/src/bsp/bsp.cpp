#include <zephyr/kernel.h>
#include <soc.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>
#include <nrfx.h>
#include "buzzer.h"
#include "bsp.h"

LOG_MODULE_REGISTER(bsp, 2);

#define BUTTONS_NODE DT_PATH(b_buttons)
#define LEDS_NODE DT_PATH(b_leds)

#define OUTPUTS_NODE DT_PATH(outputs)
// #define INPUTS_NODE DT_PATH(inputs)

#define GPIO_SPEC_AND_COMMA(button_or_led) GPIO_DT_SPEC_GET(button_or_led, gpios),

// #define BUTTON_NUM	ARRAY_SIZE(buttons)
#define BUTTON_NUM	4

// #define OUTPUT_NUM		ARRAY_SIZE(leds)
#define OUTPUT_NUM	4

static const struct gpio_dt_spec buttons[] = {
	// DT_FOREACH_CHILD(BUTTONS_NODE, GPIO_SPEC_AND_COMMA)
	GPIO_DT_SPEC_GET(DT_NODELABEL(button), gpios),
#if DT_NODE_EXISTS(DT_ALIAS(button_up))
	GPIO_DT_SPEC_GET(DT_ALIAS(button_up), gpios),
#endif
#if DT_NODE_EXISTS(DT_ALIAS(button_down))
	GPIO_DT_SPEC_GET(DT_ALIAS(button_down), gpios),
#endif
#if DT_NODE_EXISTS(DT_ALIAS(button_stop))
	GPIO_DT_SPEC_GET(DT_ALIAS(button_stop), gpios),
#endif
};




static const struct gpio_dt_spec leds[] = {
// #if DT_NODE_EXISTS(LEDS_NODE)
	DT_FOREACH_CHILD(LEDS_NODE, GPIO_SPEC_AND_COMMA)
	DT_FOREACH_CHILD(OUTPUTS_NODE, GPIO_SPEC_AND_COMMA)
// #endif
};


enum state {
	STATE_WAITING,
	STATE_SCANNING,
};

static enum state state;
static struct k_work_delayable buttons_scan;
static BSP::button_handler_t button_handler_cb;
static atomic_t my_buttons;
static struct gpio_callback gpio_cb;
static struct k_spinlock lock;
static sys_slist_t button_handlers;
static struct k_mutex button_handler_mut;



void BSP::InitGpio()
{
	int ret;
	InitOutputsAndLeds();
	ret = BuzzerInit();
	if (ret) {
		LOG_ERR("Buzzer init failed");
		return;
	}
}

int BSP::InitOutputsAndLeds()
{
    int err;

	for (size_t i = 0; i < OUTPUT_NUM; i++) {
		err = gpio_pin_configure_dt(&leds[i], GPIO_OUTPUT);
		if (err) {
			LOG_ERR("Cannot configure LED gpio");
			return err;
		}
	}

	SetOutputsState(DK_NO_LEDS_MSK, DK_ALL_OUTPUTS_MSK);
    return err;
}


int BSP::callback_ctrl(bool enable)
{
	gpio_flags_t flags = enable ? GPIO_INT_LEVEL_ACTIVE : GPIO_INT_DISABLE;
	int err = 0;

	/* This must be done with irqs disabled to avoid pin callback
	 * being fired before others are still not activated.
	 */
	for (size_t i = 0; (i < BUTTON_NUM) && !err; i++) {
		err = gpio_pin_interrupt_configure_dt(&buttons[i], flags);
	}

	return err;
}

uint32_t BSP::get_buttons(void)
{
	uint32_t ret = 0;
	for (size_t i = 0; i < BUTTON_NUM; i++) {
		int val;

		val = gpio_pin_get_dt(&buttons[i]);
		if (val < 0) {
			LOG_ERR("Cannot read gpio pin");
			return 0;
		}
		if (val) {
			ret |= 1U << i;
		}
	}

	return ret;
}

void BSP::button_handlers_call(uint32_t button_state, uint32_t has_changed)
{
	struct button_handler *handler;

	if (button_handler_cb != NULL) {
		button_handler_cb(button_state, has_changed);
	}
#ifdef CONFIG_DK_LIBRARY_DYNAMIC_BUTTON_HANDLERS
	if (IS_ENABLED(CONFIG_DK_LIBRARY_DYNAMIC_BUTTON_HANDLERS)) {
		k_mutex_lock(&button_handler_mut, K_FOREVER);
		SYS_SLIST_FOR_EACH_CONTAINER(&button_handlers, handler, node) {
			handler->cb(button_state, has_changed);
		}
		k_mutex_unlock(&button_handler_mut);
	}
#endif
}

void BSP::buttons_scan_fn(struct k_work *work)
{
	static uint32_t last_button_scan;
	static bool initial_run = true;
	uint32_t button_scan;

	button_scan = get_buttons();
	atomic_set(&my_buttons, (atomic_val_t)button_scan);

	if (!initial_run) {
		if (button_scan != last_button_scan) {
			uint32_t has_changed = (button_scan ^ last_button_scan);

			LOG_INF("has_changed= %d", has_changed);

			button_handlers_call(button_scan, has_changed);
		}
	} else {
		initial_run = false;
	}

	last_button_scan = button_scan;

	if (button_scan != 0) {
		k_work_reschedule(&buttons_scan,
		  K_MSEC(10));
	} else {
		/* If no button is pressed module can switch to callbacks */
		int err = 0;

		k_spinlock_key_t key = k_spin_lock(&lock);

		switch (state) {
		case STATE_SCANNING:
			state = STATE_WAITING;
			err = callback_ctrl(true);
			break;

		default:
			__ASSERT_NO_MSG(false);
			break;
		}

		k_spin_unlock(&lock, key);

		if (err) {
			LOG_ERR("Cannot enable callbacks");
		}
	}
}

void BSP::button_pressed(const struct device *gpio_dev, struct gpio_callback *cb,
		    uint32_t pins)
{
	k_spinlock_key_t key = k_spin_lock(&lock);

	/* Disable GPIO interrupt */
	int err = callback_ctrl(false);

	if (err) {
		LOG_ERR("Cannot disable callbacks");
	}

	switch (state) {
	case STATE_WAITING:
		state = STATE_SCANNING;
		k_work_reschedule(&buttons_scan, K_MSEC(1));
		break;

	case STATE_SCANNING:
	default:
		/* Invalid state */
		__ASSERT_NO_MSG(false);
		break;
	}

	k_spin_unlock(&lock, key);
}

int BSP::InitInputsAndButtons(button_handler_t button_handler)
{
	int err;

	button_handler_cb = button_handler;

#ifdef CONFIG_DK_LIBRARY_DYNAMIC_BUTTON_HANDLERS
	if (IS_ENABLED(CONFIG_DK_LIBRARY_DYNAMIC_BUTTON_HANDLERS)) {
		k_mutex_init(&button_handler_mut);
	}
#endif

	for (size_t i = 0; i < BUTTON_NUM; i++) {
		/* Enable pull resistor towards the inactive voltage. */
		gpio_flags_t flags =
			buttons[i].dt_flags & GPIO_ACTIVE_LOW ?
			GPIO_PULL_UP : GPIO_PULL_DOWN;
		err = gpio_pin_configure_dt(&buttons[i], GPIO_INPUT | flags);

		if (err) {
			LOG_ERR("Cannot configure button gpio");
			return err;
		}
	}

	uint32_t pin_mask = 0;

	for (size_t i = 0; i < BUTTON_NUM; i++) {
		/* Module starts in scanning mode and will switch to
		 * callback mode if no button is pressed.
		 */
		err = gpio_pin_interrupt_configure_dt(&buttons[i],
						      GPIO_INT_DISABLE);
		if (err) {
			LOG_ERR("Cannot disable callbacks()");
			return err;
		}

		pin_mask |= BIT(buttons[i].pin);
	}

	gpio_init_callback(&gpio_cb, button_pressed, pin_mask);

	for (size_t i = 0; i < BUTTON_NUM; i++) {
		err = gpio_add_callback(buttons[i].port, &gpio_cb);
		if (err) {
			LOG_ERR("Cannot add callback");
			return err;
		}
	}

	k_work_init_delayable(&buttons_scan, buttons_scan_fn);

	state = STATE_SCANNING;

	k_work_schedule(&buttons_scan, K_NO_WAIT);

	dk_read_buttons(NULL, NULL);

	atomic_set(&my_buttons, (atomic_val_t)get_buttons());

	return 0;
}

//==================

void BSP::Init()
{

}

//-----

int BSP::SetOutputsState(uint32_t outputs_on_mask, uint32_t outputs_off_mask)
{
	if ((outputs_on_mask & ~DK_ALL_OUTPUTS_MSK) != 0 ||
	   (outputs_off_mask & ~DK_ALL_OUTPUTS_MSK) != 0) {
		return -EINVAL;
	}

	for (size_t i = 0; i < OUTPUT_NUM; i++) {
		int val, err;

		if (BIT(i) & outputs_on_mask) {
			val = 1;
		} else if (BIT(i) & outputs_off_mask) {
			val = 0;
		} else {
			continue;
		}

		err = gpio_pin_set_dt(&leds[i], val);
		if (err) {
			LOG_ERR("Cannot write gpio");
			return err;
		}
	}

	return 0;
}

int BSP::SetOutputs(uint32_t outputs)
{
	return SetOutputsState(outputs, DK_ALL_OUTPUTS_MSK);
}

int BSP::SetOutput(uint8_t output_idx, uint32_t val)
{
	int err;

	if (output_idx >= OUTPUT_NUM) {
		LOG_ERR("OUTPUT index out of the range");
		return -EINVAL;
	}
	err = gpio_pin_set_dt(&leds[output_idx], val);
	if (err) {
		LOG_ERR("Cannot write OUTPUT gpio");
	}
	return err;
}

int BSP::dk_set_led_on(uint8_t led_idx)
{
	return SetOutput(led_idx, 1);
}

int BSP::dk_set_led_off(uint8_t led_idx)
{
	return SetOutput(led_idx, 0);
}
//-----

void BSP::dk_read_buttons(uint32_t *button_state, uint32_t *has_changed)
{
	static uint32_t last_state;
	uint32_t current_state = atomic_get(&my_buttons);

	if (button_state != NULL) {
		*button_state = current_state;
	}

	if (has_changed != NULL) {
		*has_changed = (current_state ^ last_state);
	}

	last_state = current_state;
}

uint32_t BSP::dk_get_buttons(void)
{
	return atomic_get(&my_buttons);
}