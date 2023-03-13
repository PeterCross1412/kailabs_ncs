#pragma once

#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include <zephyr/sys/slist.h>

class BSP {
public:
    #define DK_NO_LEDS_MSK    (0)
    #define DK_LED1           0
    // #define DK_LED2           1
    // #define DK_LED3           2
    // #define DK_LED4           3
    #define DK_LED1_MSK       BIT(DK_LED1)
    // #define DK_LED2_MSK       BIT(DK_LED2)
    // #define DK_LED3_MSK       BIT(DK_LED3)
    // #define DK_LED4_MSK       BIT(DK_LED4)
    // #define DK_ALL_LEDS_MSK   (DK_LED1_MSK | DK_LED2_MSK |\
    //                            DK_LED3_MSK | DK_LED4_MSK)

    #define DK_ALL_LEDS_MSK (DK_LED1_MSK)

    #define DK_NO_BTNS_MSK   (0)
    #define DK_BTN1          0
    #define DK_BTN2          1
    #define DK_BTN3          2
    // #define DK_BTN4          3
    #define DK_BTN1_MSK      BIT(DK_BTN1)
    #define DK_BTN2_MSK      BIT(DK_BTN2)
    #define DK_BTN3_MSK      BIT(DK_BTN3)
    // #define DK_BTN4_MSK      BIT(DK_BTN4)
    // #define DK_ALL_BTNS_MSK  (DK_BTN1_MSK | DK_BTN2_MSK | \
    //             DK_BTN3_MSK | DK_BTN4_MSK)
    #define DK_ALL_BTNS_MSK  (DK_BTN1_MSK | DK_BTN2_MSK | DK_BTN3_MSK)

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
     *  @param  leds_on_mask  Bitmask that defines which LEDs to turn on.
     *                        If this bitmask overlaps with @p leds_off_mask,
     *                        @p leds_on_mask has priority.
     *
     *  @param  leds_off_mask Bitmask that defines which LEDs to turn off.
     *                        If this bitmask overlaps with @p leds_on_mask,
     *                        @p leds_on_mask has priority.
     *
     *  @retval 0           If the operation was successful.
     *                      Otherwise, a (negative) error code is returned.
     */
    static int SetOutputsState(uint32_t leds_on_mask, uint32_t leds_off_mask);
    static int SetOutputs(uint32_t leds);

    /** @brief Set a single LED value.
     *
     *  This function turns a single LED on or off.
     *
     *  @param led_idx Index of the LED.
     *  @param val     Value for the LED: 1 - turn on, 0 - turn off
     *
     *  @retval 0           If the operation was successful.
     *                      Otherwise, a (negative) error code is returned.
     *
     *  @sa dk_set_led_on, dk_set_led_off
     */
    static int SetOutput(uint8_t led_idx, uint32_t val);

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





    static void dk_read_buttons(uint32_t *button_state, uint32_t *has_changed);

private:
    static int callback_ctrl(bool enable);
    static uint32_t get_buttons(void);
    static void button_pressed(const struct device *gpio_dev, struct gpio_callback *cb,
		                        uint32_t pins);
    static void buttons_scan_fn(struct k_work *work);
    static void button_handlers_call(uint32_t button_state, uint32_t has_changed);
};
