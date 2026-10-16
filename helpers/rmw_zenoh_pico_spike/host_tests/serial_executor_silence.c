#include <assert.h>
#include <errno.h>
#include <poll.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdio.h>
#include <time.h>
#include <unistd.h>

#include "zenoh-pico/link/transport/serial.h"
#include "zenoh-pico/link/transport/serial_protocol.h"
#include "zenoh-pico/runtime/executor.h"

enum { TIMER_DELAY_MS = 40, SILENCE_MS = 400, READ_POLL_MS = 5 };

typedef struct {
    int pipe_write;
    uint64_t started_ms;
    atomic_uint heartbeats;
    atomic_bool reader_entered;
    atomic_bool released;
    bool timer_armed;
    bool timer_fired;
    uint64_t timer_elapsed_ms;
    unsigned int heartbeats_at_timer;
    bool released_at_timer;
    _z_sys_net_socket_t socket;
} experiment_t;

static int read_poll_timeout_ms;

static uint64_t monotonic_ms(void)
{
    struct timespec timestamp;
    assert(clock_gettime(CLOCK_MONOTONIC, &timestamp) == 0);
    return (uint64_t)timestamp.tv_sec * 1000 + (uint64_t)timestamp.tv_nsec / 1000000;
}

size_t __wrap__z_serial_read(_z_sys_net_socket_t socket, uint8_t *buffer, size_t length)
{
    struct pollfd descriptor = {.fd = socket._fd, .events = POLLIN};
    int ready;
    do {
        ready = poll(&descriptor, 1, read_poll_timeout_ms);
    } while (ready < 0 && errno == EINTR);
    if (ready <= 0) {
        return SIZE_MAX;
    }
    ssize_t received = read(socket._fd, buffer, length);
    return received < 0 ? SIZE_MAX : (size_t)received;
}

static void *heartbeat_and_release(void *argument)
{
    experiment_t *experiment = argument;
    struct timespec pause = {.tv_nsec = 1000000};
    while (!atomic_load(&experiment->reader_entered)) {
        nanosleep(&pause, NULL);
    }
    uint64_t silence_started_ms = monotonic_ms();
    pause.tv_nsec = 10000000;
    while (monotonic_ms() - silence_started_ms < SILENCE_MS) {
        atomic_fetch_add(&experiment->heartbeats, 1);
        nanosleep(&pause, NULL);
    }
    atomic_store(&experiment->released, true);
    uint8_t delimiter = 0;
    assert(write(experiment->pipe_write, &delimiter, sizeof(delimiter)) == sizeof(delimiter));
    return NULL;
}

static _z_fut_fn_result_t serial_reader(void *argument, _z_executor_t *executor)
{
    (void)executor;
    experiment_t *experiment = argument;
    atomic_store(&experiment->reader_entered, true);
    uint8_t payload[16];
    (void)_z_read_serial(experiment->socket, payload, sizeof(payload));
    return atomic_load(&experiment->released) ? _z_fut_fn_result_ready() : _z_fut_fn_result_continue();
}

static _z_fut_fn_result_t timer_probe(void *argument, _z_executor_t *executor)
{
    (void)executor;
    experiment_t *experiment = argument;
    if (!experiment->timer_armed) {
        experiment->timer_armed = true;
        return _z_fut_fn_result_wake_up_after(TIMER_DELAY_MS);
    }
    experiment->timer_fired = true;
    experiment->timer_elapsed_ms = monotonic_ms() - experiment->started_ms;
    experiment->heartbeats_at_timer = atomic_load(&experiment->heartbeats);
    experiment->released_at_timer = atomic_load(&experiment->released);
    return _z_fut_fn_result_ready();
}

static void run_case(bool bounded)
{
    int descriptors[2];
    assert(pipe(descriptors) == 0);
    experiment_t experiment = {
        .pipe_write = descriptors[1],
        .socket = {._fd = descriptors[0]},
    };
    atomic_init(&experiment.heartbeats, 0);
    atomic_init(&experiment.reader_entered, false);
    atomic_init(&experiment.released, false);
    read_poll_timeout_ms = bounded ? READ_POLL_MS : -1;
    _z_executor_t executor = _z_executor_new();
    _z_fut_t timer = _z_fut_new(&experiment, timer_probe, NULL);
    assert(!_z_fut_handle_is_null(_z_executor_spawn(&executor, &timer)));
    experiment.started_ms = monotonic_ms();
    (void)_z_executor_spin(&executor);
    assert(experiment.timer_armed);
    _z_fut_t reader = _z_fut_new(&experiment, serial_reader, NULL);
    assert(!_z_fut_handle_is_null(_z_executor_spawn(&executor, &reader)));
    pthread_t feeder;
    assert(pthread_create(&feeder, NULL, heartbeat_and_release, &experiment) == 0);
    while (!experiment.timer_fired) {
        (void)_z_executor_spin(&executor);
    }
    assert(pthread_join(feeder, NULL) == 0);
    printf("%s: timer=%llu ms (requested %d ms), external heartbeat=%u, released=%s\n",
           bounded ? "bounded" : "blocking", (unsigned long long)experiment.timer_elapsed_ms,
           TIMER_DELAY_MS, experiment.heartbeats_at_timer,
           experiment.released_at_timer ? "yes" : "no");
    if (bounded) {
        assert(!experiment.released_at_timer);
    } else {
        assert(experiment.released_at_timer);
        assert(experiment.timer_elapsed_ms >= SILENCE_MS);
        assert(experiment.heartbeats_at_timer > 0);
    }
    _z_executor_destroy(&executor);
    assert(close(descriptors[0]) == 0);
    assert(close(descriptors[1]) == 0);
}

int main(void)
{
    run_case(false);
    run_case(true);
    puts("PASS: blocking serial model starves a shared executor timer while an external heartbeat runs.");
    puts("This is not MCU root-cause proof, a USB implementation test, or a recovery fix.");
    return 0;
}
