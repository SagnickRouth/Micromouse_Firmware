#include "motion.h"
#include "encoder.h"
#include "motor.h"
#include "pid.h"
#include "speed_control.h"
#include <math.h>

RobotPose robot_pose = {0U, 0U, DIR_NORTH, 0.0f, 0.0f};

static PidController distance_pid;
static float commanded_speed_mmps;
static bool moving;
static bool turning;
static float target_distance;
static float target_angle;
static int32_t start_left;
static int32_t start_right;
static uint32_t motion_start_ms;
static uint32_t profile_last_ms;

static float ticks_to_distance(int32_t ticks)
{
    /* Motion calibration is deliberately separate from speed calibration.
     * The assembled robot has shown a different effective count scale during
     * straight motion than the nominal encoder speed scale. */
    return (float)ticks *
           ((3.14159265f * WHEEL_DIAMETER_MM) / MOTION_TICKS_PER_REV);
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
    speed_control_stop();
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
    commanded_speed_mmps = 0.0f;
    target_angle = 0.0f;
    motion_start_ms = HAL_GetTick();
    profile_last_ms = motion_start_ms;
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

    const uint32_t now = HAL_GetTick();

    /* Turn timeout is only applicable to turns. A straight cell move can
     * legitimately take longer because its speed profile includes braking. */
    if (turning && (now - motion_start_ms) >= TURN_TIMEOUT_MS) {
        motion_stop();
        return;
    }

    if (moving) {
        const float travelled = average_distance();
        const float error = target_distance - travelled;

        if (fabsf(error) <= 2.0f) {
            motion_stop();
            return;
        }

        /*
         * Trapezoidal-style speed profile. Update the profile from elapsed
         * wall-clock time rather than assuming motion_update() is called at
         * CONTROL_DT; the main loop runs much faster than 1 ms.
         */
        const uint32_t profile_elapsed_ms = now - profile_last_ms;
        if (profile_elapsed_ms < 10U)
            return;

        const float profile_dt = (float)profile_elapsed_ms * 0.001f;
        profile_last_ms = now;
        const float accel_step = ACCEL_MMPS2 * profile_dt;
        const float decel_speed = sqrtf(fmaxf(0.0f, 2.0f * DECEL_MMPS2 * fabsf(error)));
        const float speed_limit = fminf((float)CELL_MOVE_SPEED_MMPS, decel_speed);

        if (commanded_speed_mmps < speed_limit)
            commanded_speed_mmps = fminf(commanded_speed_mmps + accel_step, speed_limit);
        else
            commanded_speed_mmps = fmaxf(commanded_speed_mmps - accel_step, speed_limit);

        /* Allow the speed command to fall to zero for controlled braking. */
        if (error > 5.0f && commanded_speed_mmps < 25.0f)
            commanded_speed_mmps = 25.0f;

        const int32_t dl = encoder_get_left_count() - start_left;
        const int32_t dr = encoder_get_right_count() - start_right;
        const float sync_error = ticks_to_distance(dl - dr);

        /*
         * Small differential trim keeps the two wheel distances together
         * while the speed PID controls each wheel's absolute speed.
         */
        const float sync_trim = 1.5f * sync_error;
        float left_target = commanded_speed_mmps - sync_trim;
        float right_target = commanded_speed_mmps + sync_trim;

        if (left_target < 0.0f) left_target = 0.0f;
        if (right_target < 0.0f) right_target = 0.0f;

        /*
         * Start active braking earlier. At the current test speed, an 8 mm
         * braking window is too short to absorb drivetrain inertia.
         */
        if (error <= 20.0f) {
            speed_control_update(0.0f, 0.0f);
            if (fabsf(encoder_get_left_speed()) < 8.0f &&
                fabsf(encoder_get_right_speed()) < 8.0f) {
                motion_stop();
            }
        } else {
            speed_control_update(left_target, right_target);
        }
        return;
    }

    /* Encoder-based in-place turn. Use the calibrated motion tick scale
     * directly and a bounded PWM profile. This deliberately avoids feeding
     * signed turn commands through the wheel-speed PID: the speed PID was
     * tuned for forward wheel motion and can produce unstable behavior when
     * both wheels are commanded in opposite directions. */
    const float wheel_distance =
        3.14159265f * WHEEL_TRACK_MM *
        fabsf(target_angle) / 360.0f;
    const float mm_per_motion_tick =
        (3.14159265f * WHEEL_DIAMETER_MM) / MOTION_TICKS_PER_REV;

    const int32_t dl_ticks = encoder_get_left_count() - start_left;
    const int32_t dr_ticks = encoder_get_right_count() - start_right;
    const float left_dist = fabsf((float)dl_ticks * mm_per_motion_tick);
    const float right_dist = fabsf((float)dr_ticks * mm_per_motion_tick);
    const float turned = 0.5f * (left_dist + right_dist);

    if (turned >= wheel_distance) {
        /* Hard dynamic brake removes the remaining rotational momentum. */
        motor_enable();
        motor_brake();
        HAL_Delay(60U);
        motion_stop();
        return;
    }

    const float remaining = wheel_distance - turned;
    const float braking_distance = 12.0f;
    float pwm = (float)TURN_SPEED_MMPS;

    /* Convert the requested turn speed into a conservative PWM command.
     * The exact PWM-to-speed relationship is not linear, so keep this
     * intentionally bounded and use encoder position for the stop point. */
    pwm = 260.0f;
    if (remaining < braking_distance) {
        pwm = 260.0f * (remaining / braking_distance);
        if (pwm < 70.0f) pwm = 70.0f;
    }

    const int16_t command = (int16_t)pwm;
    motor_enable();

    if (target_angle > 0.0f) {
        motor_set_left(command);
        motor_set_right((int16_t)-command);
    } else {
        motor_set_left((int16_t)-command);
        motor_set_right(command);
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
