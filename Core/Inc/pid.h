/**
 * @file    pid.h
 * @brief   PID controller interface with anti-windup.
 */

#ifndef PID_H
#define PID_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** @brief PID controller state. */
typedef struct {
    float kp;           /**< Proportional gain          */
    float ki;           /**< Integral gain              */
    float kd;           /**< Derivative gain            */
    float integral;     /**< Accumulated integral       */
    float prev_error;   /**< Previous error for D term  */
    float out_min;      /**< Minimum output clamp       */
    float out_max;      /**< Maximum output clamp       */
} PidController;

/**
 * @brief  Initialize PID controller.
 * @param  pid      Pointer to PID struct.
 * @param  kp       Proportional gain.
 * @param  ki       Integral gain.
 * @param  kd       Derivative gain.
 * @param  out_min  Minimum output.
 * @param  out_max  Maximum output.
 */
void pid_init(PidController *pid, float kp, float ki, float kd,
              float out_min, float out_max);

/**
 * @brief  Compute PID output.
 * @param  pid       Pointer to PID struct.
 * @param  setpoint  Desired value.
 * @param  measured  Current measured value.
 * @param  dt        Time delta (seconds).
 * @return Clamped PID output.
 */
float pid_compute(PidController *pid, float setpoint,
                  float measured, float dt);

/**
 * @brief  Reset PID state (integral and previous error).
 * @param  pid  Pointer to PID struct.
 */
void pid_reset(PidController *pid);

#ifdef __cplusplus
}
#endif

#endif /* PID_H */
