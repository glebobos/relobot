#include <assert.h>
#include <stdio.h>

#include "serial_rx.h"
#include "zenoh-pico/runtime/executor.h"

typedef struct {
    serial_rx_t receiver;
    unsigned int reads;
    bool timer_armed;
    bool timer_fired;
    z_clock_t started;
} experiment_t;

static int silent_input(void *context)
{
    (void)context;
    return SERIAL_RX_EMPTY;
}

static _z_fut_fn_result_t reader(void *context, _z_executor_t *executor)
{
    (void)executor;
    experiment_t *experiment = context;
    uint8_t payload[16];
    size_t received;
    assert(serial_rx_poll(&experiment->receiver, silent_input, NULL, payload,
                          sizeof(payload), &received) == SERIAL_RX_PENDING);
    ++experiment->reads;
    return _z_fut_fn_result_wake_up_after(5);
}

static _z_fut_fn_result_t timer(void *context, _z_executor_t *executor)
{
    (void)executor;
    experiment_t *experiment = context;
    if (!experiment->timer_armed) {
        experiment->timer_armed = true;
        return _z_fut_fn_result_wake_up_after(40);
    }
    experiment->timer_fired = true;
    return _z_fut_fn_result_ready();
}

int main(void)
{
    experiment_t experiment = {0};
    experiment.receiver.encoded[0] = 4;
    experiment.receiver.used = 1;
    _z_executor_t executor = _z_executor_new();
    _z_fut_t read_future = _z_fut_new(&experiment, reader, NULL);
    _z_fut_t timer_future = _z_fut_new(&experiment, timer, NULL);
    assert(!_z_fut_handle_is_null(_z_executor_spawn(&executor, &read_future)));
    assert(!_z_fut_handle_is_null(_z_executor_spawn(&executor, &timer_future)));
    experiment.started = z_clock_now();
    while (!experiment.timer_fired) {
        (void)_z_executor_spin(&executor);
        z_sleep_ms(1);
    }
    assert(experiment.reads > 0);
    assert(experiment.receiver.used == 1 && experiment.receiver.encoded[0] == 4);
    printf("PASS: pinned shared-executor timer fired at %lu ms while RX retained a partial frame (%u polls).\n",
           z_clock_elapsed_ms(&experiment.started), experiment.reads);
    _z_executor_destroy(&executor);
    return 0;
}
