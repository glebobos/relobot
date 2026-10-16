#include "command_probe.h"

#include <math.h>

void command_probe_tick(command_probe_t *probe, uint32_t now_ms)
{
    if (probe->active &&
        (uint32_t)(now_ms - probe->last_accept_ms) >= COMMAND_PROBE_TIMEOUT_MS) {
        probe->active = false;
        probe->timeouts++;
    }
}

bool command_probe_receive(command_probe_t *probe, float value, uint32_t now_ms)
{
    command_probe_tick(probe, now_ms);
    if (!isfinite(value) || value < 1.0f ||
        value > (float)COMMAND_PROBE_MAX_SEQUENCE || value != (float)(uint32_t)value) {
        probe->invalid++;
        return false;
    }

    uint32_t sequence = (uint32_t)value;
    if (sequence <= probe->last_sequence) {
        probe->rejected++;
        return false;
    }

    probe->last_sequence = sequence;
    probe->last_accept_ms = now_ms;
    probe->accepted++;
    probe->active = true;
    return true;
}
