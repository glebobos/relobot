#include <assert.h>
#include <stdatomic.h>
#include <stdio.h>

#include "picoros_connectivity.h"

typedef struct {
    atomic_uint opened;
    atomic_uint closed;
} event_counts_t;

void picoros_diagnostic_stage(unsigned int stage)
{
    (void)stage;
}

static void observe_event(z_loaned_transport_event_t *event, void *context)
{
    event_counts_t *counts = context;
    assert(z_transport_whatami(z_transport_event_transport(event)) == Z_WHATAMI_PEER);
    if (z_transport_event_kind(event) == Z_SAMPLE_KIND_PUT) {
        atomic_fetch_add(&counts->opened, 1);
    } else {
        assert(z_transport_event_kind(event) == Z_SAMPLE_KIND_DELETE);
        atomic_fetch_add(&counts->closed, 1);
    }
}

static bool wait_count(atomic_uint *counter, unsigned int expected)
{
    for (unsigned int attempt = 0; attempt < 200; ++attempt) {
        if (atomic_load(counter) >= expected) {
            return true;
        }
        z_sleep_ms(10);
    }
    return false;
}

static void count_transport(z_loaned_transport_t *transport, void *context)
{
    (void)transport;
    unsigned int *count = context;
    ++*count;
}

static unsigned int transport_count(void)
{
    unsigned int count = 0;
    z_owned_closure_transport_t closure;
    assert(z_closure_transport(&closure, count_transport, NULL, &count) == Z_OK);
    assert(z_info_transports(picoros_interface_session(), z_closure_transport_move(&closure)) == Z_OK);
    return count;
}

static void open_remote(z_owned_session_t *remote)
{
    z_owned_config_t config;
    z_config_default(&config);
    assert(zp_config_insert(z_config_loan_mut(&config), Z_CONFIG_MODE_KEY, "peer") == Z_OK);
    assert(zp_config_insert(z_config_loan_mut(&config), Z_CONFIG_MULTICAST_SCOUTING_KEY, "false") == Z_OK);
    assert(zp_config_insert(z_config_loan_mut(&config), Z_CONFIG_CONNECT_KEY, "tcp/127.0.0.1:17467") == Z_OK);
    assert(z_open(remote, z_config_move(&config), NULL) == Z_OK);
    assert(zp_start_read_task(z_session_loan_mut(remote), NULL) == Z_OK);
}

int main(void)
{
    event_counts_t counts = {0};
    z_owned_transport_events_listener_t listener;
    assert(picoros_interface_session() == NULL);
    assert(!picoros_connectivity_observe(NULL, observe_event, &counts));
    assert(!picoros_connectivity_observe(&listener, observe_event, &counts));
    assert(!z_internal_transport_events_listener_check(&listener));
    z_transport_events_listener_drop(z_transport_events_listener_move(&listener));

    picoros_interface_t interface = {.mode = "peer", .locator = "tcp/127.0.0.1:17467"};
    assert(picoros_interface_init(&interface) == PICOROS_OK);
    assert(picoros_interface_session() != NULL);
    assert(transport_count() == 0);
    assert(!picoros_connectivity_observe(&listener, NULL, &counts));
    assert(!z_internal_transport_events_listener_check(&listener));

    z_owned_session_t remote;
    open_remote(&remote);
    assert(transport_count() == 1);
    assert(picoros_connectivity_observe(&listener, observe_event, &counts));
    assert(atomic_load(&counts.opened) == 1);
    assert(atomic_load(&counts.closed) == 0);
    z_session_drop(z_session_move(&remote));
    assert(wait_count(&counts.closed, 1));
    assert(transport_count() == 0);

    open_remote(&remote);
    assert(wait_count(&counts.opened, 2));
    assert(transport_count() == 1);
    z_session_drop(z_session_move(&remote));
    assert(wait_count(&counts.closed, 2));
    assert(z_undeclare_transport_events_listener(z_transport_events_listener_move(&listener)) == Z_OK);
    assert(!z_internal_transport_events_listener_check(&listener));

    open_remote(&remote);
    assert(transport_count() == 1);
    z_session_drop(z_session_move(&remote));
    assert(atomic_load(&counts.opened) == 2);
    assert(atomic_load(&counts.closed) == 2);
    picoros_interface_close();
    assert(picoros_interface_session() == NULL);
    assert(!picoros_connectivity_observe(&listener, observe_event, &counts));
    z_transport_events_listener_drop(z_transport_events_listener_move(&listener));
    puts("PASS: actual Pico-ROS session, transport history, TCP open/close events, listener cleanup and closed-session guard.");
    return 0;
}
