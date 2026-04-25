#include "pid_controller.h"

void pid_controller_init(
    pid_controller_t *pid,
    float kp,
    float ki,
    float kd,
    float out_min,
    float out_max,
    float integral_min,
    float integral_max)
{
    if (pid == NULL) {
        return;
    }

    /* #P3 — silently swap inverted limits so clamp always works correctly */
    if (out_min > out_max)           { float t = out_min; out_min = out_max; out_max = t; }
    if (integral_min > integral_max) { float t = integral_min; integral_min = integral_max; integral_max = t; }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->out_min = out_min;
    pid->out_max = out_max;
    pid->integral_min = integral_min;
    pid->integral_max = integral_max;

    pid_controller_reset(pid);
}

void pid_controller_reset(pid_controller_t *pid)
{
    if (pid == NULL) {
        return;
    }

    pid->integral = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->initialized = 0;
}

float pid_controller_update(pid_controller_t *pid, float setpoint, float measurement, float dt_s)
{
    if (pid == NULL || dt_s <= 0.0f) {
        return 0.0f;
    }

    /* #P2 — refuse to corrupt integral state with NaN/Inf inputs */
    if (!isfinite(setpoint) || !isfinite(measurement)) {
        return 0.0f;
    }

    float error = setpoint - measurement;
    pid->integral = pid_clampf(
        pid->integral + (error * dt_s),
        pid->integral_min,
        pid->integral_max);

    /* #P1 — derivative-on-measurement eliminates setpoint-step kick */
    float derivative = 0.0f;
    if (pid->initialized) {
        derivative = -(measurement - pid->prev_measurement) / dt_s;
    }

    pid->prev_measurement = measurement;
    pid->initialized = 1;

    float output = (pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * derivative);
    return pid_clampf(output, pid->out_min, pid->out_max);
}