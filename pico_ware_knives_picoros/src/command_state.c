#include "command_state.h"

#include <math.h>

#include "pid_controller.h"

void knives_command_disarm(knives_command_state_t *state)
{
    *state = (knives_command_state_t){0};
}

bool knives_command_accept(knives_command_state_t *state, float rpm, uint32_t now_ms)
{
    if (!isfinite(rpm)) {
        return false;
    }
    state->target_rpm = pid_clampf(rpm, -KNIVES_MAX_ABS_RPM, KNIVES_MAX_ABS_RPM);
    state->received_at_ms = now_ms;
    state->received = true;
    state->armed = true;
    return true;
}

float knives_command_target(knives_command_state_t *state, uint32_t now_ms)
{
    if (!state->received ||
        (uint32_t)(now_ms - state->received_at_ms) >= KNIVES_COMMAND_TIMEOUT_MS) {
        state->received = false;
        state->target_rpm = 0.0f;
    }
    return state->target_rpm;
}
