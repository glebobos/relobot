#ifndef PICOROS_COMMAND_PROBE_H
#define PICOROS_COMMAND_PROBE_H

#include <stdbool.h>
#include <stdint.h>

#define COMMAND_PROBE_TIMEOUT_MS 1000u
#define COMMAND_PROBE_MAX_SEQUENCE 16777215u

typedef struct {
    uint32_t last_sequence;
    uint32_t last_accept_ms;
    uint32_t accepted;
    uint32_t rejected;
    uint32_t invalid;
    uint32_t timeouts;
    bool active;
} command_probe_t;

bool command_probe_receive(command_probe_t *probe, float value, uint32_t now_ms);
void command_probe_tick(command_probe_t *probe, uint32_t now_ms);

#endif
