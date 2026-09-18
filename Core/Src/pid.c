#include "pid.h"

static float clampf(float v, float lo, float hi)
{
    if (v > hi) return hi;
    if (v < lo) return lo;
    return v;
}

void pid_init(PidController *pid, float kp, float ki, float kd,
              float out_min, float out_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->out_min = out_min;
    pid->out_max = out_max;
}

float pid_compute(PidController *pid, float setpoint,
                  float measured, float dt)
{
    if (dt <= 0.0f)
        return 0.0f;

    const float error = setpoint - measured;
    pid->integral += error * dt;

    if (pid->ki != 0.0f) {
        const float ki_abs = pid->ki < 0.0f ? -pid->ki : pid->ki;
        const float i_limit = (pid->out_max > 0.0f ? pid->out_max : -pid->out_min) / ki_abs;
        pid->integral = clampf(pid->integral, -i_limit, i_limit);
    }

    const float derivative = (error - pid->prev_error) / dt;
    pid->prev_error = error;

    return clampf(pid->kp * error +
                  pid->ki * pid->integral +
                  pid->kd * derivative,
                  pid->out_min, pid->out_max);
}

void pid_reset(PidController *pid)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}
