#include "command_probe.h"

#include <assert.h>
#include <math.h>

int main(void)
{
    command_probe_t probe = {0};
    command_probe_tick(&probe, 5000);
    assert(!probe.active && probe.timeouts == 0);
    assert(command_probe_receive(&probe, 101.0f, 100));
    assert(probe.active && probe.last_sequence == 101 && probe.accepted == 1);
    assert(!command_probe_receive(&probe, 101.0f, 1099));
    assert(!command_probe_receive(&probe, 100.0f, 1099));
    assert(probe.rejected == 2 && probe.last_accept_ms == 100);
    command_probe_tick(&probe, 1100);
    assert(!probe.active && probe.timeouts == 1);
    assert(!command_probe_receive(&probe, 101.0f, 1200));
    command_probe_tick(&probe, 1300);
    assert(!probe.active && probe.timeouts == 1);
    assert(command_probe_receive(&probe, 102.0f, 1400));
    assert(probe.active && probe.accepted == 2);

    float invalid_values[] = {NAN, INFINITY, -INFINITY, -1.0f, 0.0f, 1.5f, 16777216.0f};
    for (unsigned int index = 0; index < sizeof(invalid_values) / sizeof(invalid_values[0]); ++index) {
        assert(!command_probe_receive(&probe, invalid_values[index], 1500));
    }
    assert(probe.invalid == 7 && probe.last_sequence == 102 && probe.last_accept_ms == 1400);
    assert(command_probe_receive(&probe, 103.0f, 2400));
    assert(probe.timeouts == 2 && probe.active);

    probe = (command_probe_t){0};
    assert(command_probe_receive(&probe, 1.0f, UINT32_MAX - 499));
    command_probe_tick(&probe, 499);
    assert(probe.active);
    command_probe_tick(&probe, 500);
    assert(!probe.active && probe.timeouts == 1);
    assert(command_probe_receive(&probe, (float)COMMAND_PROBE_MAX_SEQUENCE, 600));
    assert(!command_probe_receive(&probe, (float)COMMAND_PROBE_MAX_SEQUENCE, 700));
    return 0;
}
