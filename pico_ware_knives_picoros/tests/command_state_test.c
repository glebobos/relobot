#include "command_state.h"

#include <assert.h>
#include <math.h>

int main(void)
{
    knives_command_state_t state = {0};
    assert(knives_command_target(&state, 10) == 0.0f);
    assert(!state.received);
    assert(!knives_command_accept(&state, 1200.0f, 15));
    assert(!state.armed);
    assert(knives_command_accept(&state, 0.0f, 19));
    assert(knives_command_accept(&state, 1200.0f, 20));
    assert(knives_command_target(&state, 5019) == 1200.0f);
    assert(knives_command_target(&state, 5020) == 0.0f);
    assert(!state.received);
    assert(!state.armed);
    assert(!knives_command_accept(&state, 1200.0f, 5021));
    assert(knives_command_accept(&state, 0.0f, 5999));
    assert(knives_command_accept(&state, 9000.0f, 6000));
    assert(state.target_rpm == KNIVES_MAX_ABS_RPM);
    assert(knives_command_accept(&state, -9000.0f, 6100));
    assert(state.target_rpm == -KNIVES_MAX_ABS_RPM);
    assert(!knives_command_accept(&state, NAN, 10000));
    assert(!knives_command_accept(&state, INFINITY, 10000));
    assert(!knives_command_accept(&state, -INFINITY, 10000));
    assert(state.received_at_ms == 6100);
    assert(knives_command_target(&state, 11100) == 0.0f);
    assert(!state.armed);
    assert(knives_command_accept(&state, 0.0f, UINT32_MAX - 1001));
    assert(knives_command_accept(&state, 1200.0f, UINT32_MAX - 1000));
    assert(knives_command_target(&state, 3998) == 1200.0f);
    assert(knives_command_target(&state, 3999) == 0.0f);
    assert(knives_command_accept(&state, 0.0f, 5000));
    assert(knives_command_target(&state, 5001) == 0.0f);
    assert(knives_command_accept(&state, -500.0f, 5100));
    assert(knives_command_target(&state, 5101) == -500.0f);
    assert(!knives_command_accept(&state, -500.0f, 10100));
    assert(!state.armed);
    assert(!state.received);
    assert(!knives_command_accept(&state, NAN, 10101));
    assert(!knives_command_accept(&state, INFINITY, 10102));
    assert(!state.armed);
    assert(knives_command_accept(&state, -0.0f, 10103));
    assert(knives_command_accept(&state, -500.0f, 10104));
    knives_command_disarm(&state);
    assert(!knives_command_accept(&state, -500.0f, 10105));
    assert(knives_command_target(&state, 10106) == 0.0f);
    return 0;
}
