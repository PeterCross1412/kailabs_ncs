#pragma once

#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include <zephyr/sys/slist.h>

class BSP {
public:
    #define DK_NO_LEDS_MSK    (0)
    #define LED_ONBOARD         0
    #define RELAY_UP            1
    #define RELAY_DOWN          2
    #define RELAY_STOP          3
    #define LED_ONBOARD_MSK     BIT(LED_ONBOARD)
    #define RELAY_UP_MSK        BIT(RELAY_UP)
    #define RELAY_DOWN_MSK      BIT(RELAY_DOWN)
    #define RELAY_STOP_MSK      BIT(RELAY_STOP)

    #define DK_ALL_LEDS_MSK     (LED_ONBOARD_MSK)
    #define DK_ALL_RELAYS_MSK   (RELAY_UP_MSK | RELAY_DOWN_MSK |\
                                 RELAY_STOP_MSK)
    #define DK_ALL_OUTPUTS_MSK  (DK_ALL_LEDS_MSK | DK_ALL_RELAYS_MSK)
    //---------------------------------------------------------------
    #define NO_BTNS_MSK   (0)
    // typedef enum btn_t {
    //     BTN_ONBOARD = 0,
    //     BTN_UP,
    //     BTN_DOWN,
    //     BTN_STOP
    // }btn_t;
    #define BTN_ONBOARD 0
    #define BTN_UP      1
    #define BTN_DOWN    2
    #define BTN_STOP    3

    #define BTN_ONBOARD_MSK     BIT(BTN_ONBOARD)
    #define BTN_UP_MSK          BIT(BTN_UP)
    #define BTN_DOWN_MSK        BIT(BTN_DOWN)
    #define BTN_STOP_MSK        BIT(BTN_STOP)

    #define DK_ALL_BTNS_MSK  (BTN_ONBOARD_MSK | BTN_UP_MSK | BTN_DOWN_MSK | BTN_STOP_MSK)

    /**
     * @typedef button_handler_t
     * @brief Callback that is executed when a button state change is detected.
     *
     * @param button_state Bitmask of button states.
     * @param has_changed Bitmask that shows which buttons have changed.
     */
    typedef void (*button_handler_t)(uint32_t button_state, uint32_t has_changed);

    /** Button handler list entry. */
    struct button_handler {
        button_handler_t cb; /**< Callback function. */
        sys_snode_t node; /**< Linked list node, for internal use. */
    };

    static void InitGpio();
    static int InitOutputsAndLeds();
    static int InitInputsAndButtons(button_handler_t button_handler);

    void Init();

    /** @brief Set value of LED pins as specified in two bitmasks.
     *
     *  @param  outputs_on_mask  Bitmask that defines which LEDs to turn on.
     *                        If this bitmask overlaps with @p outputs_off_mask,
     *                        @p outputs_on_mask has priority.
     *
     *  @param  outputs_off_mask Bitmask that defines which LEDs to turn off.
     *                        If this bitmask overlaps with @p outputs_on_mask,
     *                        @p outputs_on_mask has priority.
     *
     *  @retval 0           If the operation was successful.
     *                      Otherwise, a (negative) error code is returned.
     */
    static int SetOutputsState(uint32_t outputs_on_mask, uint32_t outputs_off_mask);
    static int SetOutputs(uint32_t outputs);

    /** @brief Set a single LED value.
     *
     *  This function turns a single LED on or off.
     *
     *  @param output_idx Index of the LED.
     *  @param val     Value for the LED: 1 - turn on, 0 - turn off
     *
     *  @retval 0           If the operation was successful.
     *                      Otherwise, a (negative) error code is returned.
     *
     *  @sa dk_set_led_on, dk_set_led_off
     */
    static int SetOutput(uint8_t output_idx, uint32_t val);

    /** @brief Turn a single LED on.
     *
     *  @param led_idx Index of the LED.
     *
     *  @retval 0           If the operation was successful.
     *                      Otherwise, a (negative) error code is returned.
     */
    static int dk_set_led_on(uint8_t led_idx);

    /** @brief Turn a single LED off.
     *
     *  @param led_idx Index of the LED.
     *
     *  @retval 0           If the operation was successful.
     *                      Otherwise, a (negative) error code is returned.
     */
    static int dk_set_led_off(uint8_t led_idx);


    // static void dk_read_buttons(uint32_t *button_state, uint32_t *has_changed);

    /** @brief Read current button states.
     *
     *  @param button_state Bitmask of button states.
     *  @param has_changed Bitmask that shows which buttons have changed.
     */
    static void dk_read_buttons(uint32_t *button_state, uint32_t *has_changed);

    /** @brief Get current button state from internal variable.
     *
     *  @return Bitmask of button states.
     */
    static uint32_t dk_get_buttons(void);

private:
    static int callback_ctrl(bool enable);
    static uint32_t get_buttons(void);
    static void button_pressed(const struct device *gpio_dev, struct gpio_callback *cb,
		                        uint32_t pins);
    static void buttons_scan_fn(struct k_work *work);
    static void button_handlers_call(uint32_t button_state, uint32_t has_changed);
};
