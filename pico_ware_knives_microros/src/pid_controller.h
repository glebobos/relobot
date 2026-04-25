#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float prev_error;
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