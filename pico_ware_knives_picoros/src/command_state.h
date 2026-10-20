#ifndef KNIVES_COMMAND_STATE_H
#define KNIVES_COMMAND_STATE_H

#include <stdbool.h>
#include <stdint.h>

#define KNIVES_COMMAND_TIMEOUT_MS 5000u
#define KNIVES_MAX_ABS_RPM 3500.0f
#define KNIVES_ZERO_EPSILON_RPM 0.01f

typedef struct {
    float target_rpm;
    uint32_t received_at_ms;
    bool received;
    bool armed;
} knives_command_state_t;

void knives_command_disarm(knives_command_state_t *state);
bool knives_command_accept(knives_command_state_t *state, float rpm, uint32_t now_ms);
float knives_command_target(knives_command_state_t *state, uint32_t now_ms);

#endif
