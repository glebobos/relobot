#include "pid_controller.h"

static float clampf(float value, float min_value, float max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

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
    if (pid == 0) {
        return;
    }

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
    if (pid == 0) {
        return;
    }

    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->initialized = 0;
}

float pid_controller_update(pid_controller_t *pid, float setpoint, float measurement, float dt_s)
{
    if (pid == 0 || dt_s <= 0.0f) {
        return 0.0f;
    }

    float error = setpoint - measurement;
    pid->integral = clampf(
        pid->integral + (error * dt_s),
        pid->integral_min,
        pid->integral_max);

    float derivative = 0.0f;
    if (pid->initialized) {
        derivative = (error - pid->prev_error) / dt_s;
    }

    pid->prev_error = error;
    pid->initialized = 1;

    float output = (pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * derivative);
    return clampf(output, pid->out_min, pid->out_max);
}