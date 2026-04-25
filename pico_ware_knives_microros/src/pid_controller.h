#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <math.h>   /* isfinite */

#ifdef __cplusplus
extern "C" {
#endif

/* Canonical clamp shared across translation units — avoids duplicate definitions. */
static inline float pid_clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float prev_measurement;   /* used for derivative-on-measurement (#P1) */
    float out_min;
    float out_max;
    float integral_min;
    float integral_max;
    int initialized;
} pid_controller_t;

void pid_controller_init(
    pid_controller_t *pid,
    float kp,
    float ki,
    float kd,
    float out_min,
    float out_max,
    float integral_min,
    float integral_max);

void pid_controller_reset(pid_controller_t *pid);

float pid_controller_update(pid_controller_t *pid, float setpoint, float measurement, float dt_s);

#ifdef __cplusplus
}
#endif

#endif