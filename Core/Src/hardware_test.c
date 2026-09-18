#include "hardware_test.h"
#include "config.h"
#include "encoder.h"
#include "motor.h"
#include "speed_control.h"
#include "motion.h"
#include "oled.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <string.h>

static uint8_t dip_read(void)
{
    uint8_t dip = 0U;

    /* Active-low: switch ON pulls the pin to GND. */
    if (HAL_GPIO_ReadPin(DIP_ALG0_PORT, DIP_ALG0_PIN) == GPIO_PIN_RESET)
        dip |= 1U;
    if (HAL_GPIO_ReadPin(DIP_ALG1_PORT, DIP_ALG1_PIN) == GPIO_PIN_RESET)
        dip |= 2U;

    return dip;
}

static const char *dip_name(uint8_t dip)
{
    switch (dip & 3U) {
    case 0: return "FLOOD";
    case 1: return "LEFT";
    case 2: return "RIGHT";
    default: return "A STAR";
    }
}

void hardware_test_init(void)
{
    /* LED is toggled before any I2C transaction so MCU execution can be
       verified even if the OLED is disconnected or miswired. */
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
    HAL_Delay(100);
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);

    oled_init();
}

void hardware_test_run(void)
{
    static uint32_t last_ui_ms = 0U;
    static uint32_t last_led_ms = 0U;
    static bool led_state = false;
    static bool button_stable = false;
    static bool button_last_raw = false;
    static uint32_t button_change_ms = 0U;
    static bool start_pending = false;
    static uint32_t start_delay_ms = 0U;

    const uint32_t now = HAL_GetTick();

    /* Encoder state is updated once by the main control loop. */
    /* 250 ms heartbeat proves the MCU main loop is alive. */
    if ((now - last_led_ms) >= 250U) {
        last_led_ms = now;
        led_state = !led_state;
        HAL_GPIO_WritePin(LED_PORT, LED_PIN,
                          led_state ? GPIO_PIN_RESET : GPIO_PIN_SET);
    }

    const bool button_pressed =
        (HAL_GPIO_ReadPin(KEY_PORT, KEY_PIN) == GPIO_PIN_RESET);
    const uint8_t dip = dip_read();

    /*
     * One-cell motion test:
     *   - A debounced PA0 press starts a 3-second stationary countdown.
     *   - The encoder is reset only when the move actually starts.
     *   - PA0 is ignored while the move runs; a new press while moving stops it.
     *   - After completion, release and press PA0 again for another run.
     */
    if (button_pressed != button_last_raw) {
        button_last_raw = button_pressed;
        button_change_ms = now;
    }

    if ((now - button_change_ms) >= 30U && button_stable != button_last_raw) {
        button_stable = button_last_raw;

        if (button_stable) {
            if (motion_is_complete() && !start_pending) {
                start_pending = true;
                start_delay_ms = now;
                motor_stop();
            } else if (!motion_is_complete() && !start_pending) {
                motion_stop();
            }
        }
    }

    if (start_pending && (now - start_delay_ms) >= 3000U) {
        start_pending = false;
        motion_move_cell();
    }

    /* OLED is deliberately refreshed slowly to keep I2C traffic low. */
    if ((now - last_ui_ms) < 100U)
        return;

    last_ui_ms = now;

    char l[24];
    char r[24];
    char ls[24];
    char rs[24];

    snprintf(l, sizeof(l), "L:%ld", (long)encoder_get_left_count());
    snprintf(r, sizeof(r), "R:%ld", (long)encoder_get_right_count());
    snprintf(ls, sizeof(ls), "LS:%ld", (long)encoder_get_left_speed());
    snprintf(rs, sizeof(rs), "RS:%ld", (long)encoder_get_right_speed());

    oled_show_hardware_test(button_pressed, dip, dip_name(dip), l, r, ls, rs);
}
