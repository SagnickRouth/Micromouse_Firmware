#include "motion.h"
#include "encoder.h"
#include "motor.h"
#include "pid.h"
#include <math.h>

RobotPose robot_pose = {0U, 0U, DIR_NORTH, 0.0f, 0.0f};

static PidController distance_pid;
static bool moving;
static bool turning;
static float target_distance;
static float target_angle;
static int32_t start_left;
static int32_t start_right;
static uint32_t motion_start_ms;

static float ticks_to_distance(int32_t ticks)
{
    return encoder_ticks_to_mm(ticks);
}

static float average_distance(void)
{
    const int32_t dl = encoder_get_left_count() - start_left;
    const int32_t dr = encoder_get_right_count() - start_right;
    return 0.5f * (ticks_to_distance(dl) + ticks_to_distance(dr));
}

static void reset_motion_state(void)
{
    moving = false;
    turning = false;
    target_distance = 0.0f;
    target_angle = 0.0f;
    pid_reset(&distance_pid);
    motor_stop();
}

void motion_init(void)
{
    pid_init(&distance_pid, KP_SPEED, 0.0f, 0.0f,
             -(float)MOTOR_PWM_MAX, (float)MOTOR_PWM_MAX);
    reset_motion_state();
}

void motion_move(float distance_mm, float end_speed)
{
    (void)end_speed;
    encoder_reset();
    start_left = 0;
    start_right = 0;
    target_distance = distance_mm;
    target_angle = 0.0f;
    motion_start_ms = HAL_GetTick();
    moving = true;
    turning = false;
    pid_reset(&distance_pid);
}

void motion_move_cell(void)
{
    motion_move((float)CELL_SIZE_MM, 0.0f);
}

void motion_turn(float angle_deg)
{
    encoder_reset();
    start_left = 0;
    start_right = 0;
    target_angle = angle_deg;
    target_distance = 0.0f;
    motion_start_ms = HAL_GetTick();
    moving = false;
    turning = true;
    pid_reset(&distance_pid);
}

void motion_turn_left(void)  { motion_turn(-TURN_ANGLE_90); }
void motion_turn_right(void) { motion_turn( TURN_ANGLE_90); }
void motion_turn_180(void)   { motion_turn( TURN_ANGLE_180); }

void motion_update(void)
{
    if (!moving && !turning)
        return;

    encoder_update();

    if ((HAL_GetTick() - motion_start_ms) >= TURN_TIMEOUT_MS) {
        motion_stop();
        return;
    }

    const float max_pwm = (float)MOTOR_PWM_MAX;

    if (moving) {
        const float travelled = average_distance();
        const float error = target_distance - travelled;

        if (fabsf(error) <= 2.0f) {
            motion_stop();
            return;
        }

        float command = KP_SPEED * error;
        if (command > max_pwm) command = max_pwm;
        if (command < -max_pwm) command = -max_pwm;

        const int32_t dl = encoder_get_left_count() - start_left;
        const int32_t dr = encoder_get_right_count() - start_right;
        const float sync_error = ticks_to_distance(dl - dr);
        const float sync = 2.0f * sync_error;

        motor_enable();
        motor_set_left((int16_t)(command - sync));
        motor_set_right((int16_t)(command + sync));
        return;
    }

    const float wheel_distance =
        (float)M_PI * (float)WHEEL_TRACK_MM *
        fabsf(target_angle) / 360.0f;

    const int32_t dl_ticks = encoder_get_left_count() - start_left;
    const int32_t dr_ticks = encoder_get_right_count() - start_right;
    const float left_dist = ticks_to_distance(dl_ticks);
    const float right_dist = ticks_to_distance(dr_ticks);
    const float turned = 0.5f * (fabsf(left_dist) + fabsf(right_dist));

    if (turned >= wheel_distance) {
        motion_stop();
        return;
    }

    const float remaining = wheel_distance - turned;
    float command = KP_TURN * remaining;
    if (command < 120.0f) command = 120.0f;
    if (command > 500.0f) command = 500.0f;

    const int16_t signed_cmd = (int16_t)command;

    if (target_angle > 0.0f) {
        motor_enable();
        motor_set_left(signed_cmd);
        motor_set_right((int16_t)-signed_cmd);
    } else {
        motor_enable();
        motor_set_left((int16_t)-signed_cmd);
        motor_set_right(signed_cmd);
    }
}

bool motion_is_complete(void)
{
    return !moving && !turning;
}

void motion_execute_direction(Direction target_dir)
{
    int delta = (int)target_dir - (int)robot_pose.facing;
    while (delta > 2) delta -= 4;
    while (delta < -2) delta += 4;

    if (delta == 1) motion_turn_right();
    else if (delta == -1) motion_turn_left();
    else if (delta == 2 || delta == -2) motion_turn_180();
    else motion_move_cell();

    robot_pose.facing = target_dir;
}

void motion_square_up(void)
{
    /* Reserved for wall-sensor/IMU integration. */
}

void motion_stop(void)
{
    reset_motion_state();
}
