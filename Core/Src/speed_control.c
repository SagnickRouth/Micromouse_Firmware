#include "speed_control.h"
#include "config.h"
#include "encoder.h"
#include "motor.h"
#include "pid.h"
#include "stm32f4xx_hal.h"

#define SPEED_CONTROL_PERIOD_MS 10U
#define SPEED_KP                2.0f
#define SPEED_KI                0.6f
#define SPEED_KD                0.02f

static PidController left_pid;
static PidController right_pid;
static uint32_t last_update_ms;
static bool active;

void speed_control_init(void)
{
    pid_init(&left_pid, SPEED_KP, SPEED_KI, SPEED_KD,
             -(float)MOTOR_PWM_MAX, (float)MOTOR_PWM_MAX);
    pid_init(&right_pid, SPEED_KP, SPEED_KI, SPEED_KD,
             -(float)MOTOR_PWM_MAX, (float)MOTOR_PWM_MAX);
    last_update_ms = HAL_GetTick();
    active = false;
}

void speed_control_update(float left_target_mmps, float right_target_mmps)
{
    const uint32_t now = HAL_GetTick();
    const uint32_t elapsed_ms = now - last_update_ms;

    if (elapsed_ms < SPEED_CONTROL_PERIOD_MS)
        return;

    last_update_ms = now;
    const float dt = (float)elapsed_ms * 0.001f;

    const float left_output = pid_compute(&left_pid, left_target_mmps,
                                          encoder_get_left_speed(), dt);
    const float right_output = pid_compute(&right_pid, right_target_mmps,
                                           encoder_get_right_speed(), dt);

    motor_enable();
    motor_set((int16_t)left_output, (int16_t)right_output);
    active = true;
}

void speed_control_stop(void)
{
    pid_reset(&left_pid);
    pid_reset(&right_pid);
    motor_stop();
    last_update_ms = HAL_GetTick();
    active = false;
}

bool speed_control_is_active(void)
{
    return active;
}
