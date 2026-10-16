#include <assert.h>
#include <stdatomic.h>
#include <stdio.h>
#include <string.h>

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
    if (z_transport_event_kind(event) == Z_SAMPLE_KIND_PUT) {
        atomic_fetch_add(&counts->opened, 1);
    } else {
        assert(z_transport_event_kind(event) == Z_SAMPLE_KIND_DELETE);
        atomic_fetch_add(&counts->closed, 1);
    }
}

static bool wait_count(atomic_uint *counter, unsigned int expected)
{
    for (unsigned int attempt = 0; attempt < 1000; ++attempt) {
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

static void receive_value(z_loaned_sample_t *sample, void *context)
{
    atomic_uint *last_value = context;
    z_owned_slice_t payload;
    assert(z_bytes_to_slice(z_sample_payload(sample), &payload) == Z_OK);
    assert(z_slice_len(z_slice_loan(&payload)) == 1);
    atomic_store(last_value, z_slice_data(z_slice_loan(&payload))[0]);
    z_slice_drop(z_slice_move(&payload));
}

static void subscribe(const z_loaned_session_t *session, const char *key,
    z_owned_subscriber_t *subscriber, atomic_uint *last_value)
{
    z_view_keyexpr_t keyexpr;
    assert(z_view_keyexpr_from_str(&keyexpr, key) == Z_OK);
    z_owned_closure_sample_t closure;
    assert(z_closure_sample(&closure, receive_value, NULL, last_value) == Z_OK);
    assert(z_declare_subscriber(session, subscriber, z_view_keyexpr_loan(&keyexpr),
        z_closure_sample_move(&closure), NULL) == Z_OK);
}

static void deliver(const z_loaned_session_t *session, const char *key,
    atomic_uint *last_value, uint8_t expected)
{
    z_view_keyexpr_t keyexpr;
    assert(z_view_keyexpr_from_str(&keyexpr, key) == Z_OK);
    for (unsigned int attempt = 0; attempt < 100; ++attempt) {
        z_owned_bytes_t payload;
        assert(z_bytes_copy_from_buf(&payload, &expected, sizeof(expected)) == Z_OK);
        assert(z_put(session, z_view_keyexpr_loan(&keyexpr), z_bytes_move(&payload), NULL) == Z_OK);
        z_sleep_ms(20);
        if (atomic_load(last_value) == expected) {
            return;
        }
    }
    assert(!"Fresh payload was not delivered");
}

static void open_server(z_owned_session_t *server)
{
    z_owned_config_t config;
    z_config_default(&config);
    assert(zp_config_insert(z_config_loan_mut(&config), Z_CONFIG_MODE_KEY, "peer") == Z_OK);
    assert(zp_config_insert(z_config_loan_mut(&config), Z_CONFIG_MULTICAST_SCOUTING_KEY, "false") == Z_OK);
    assert(zp_config_insert(z_config_loan_mut(&config), Z_CONFIG_LISTEN_KEY, "tcp/127.0.0.1:17468") == Z_OK);
    assert(z_open(server, z_config_move(&config), NULL) == Z_OK);
    assert(zp_start_read_task(z_session_loan_mut(server), NULL) == Z_OK);
    assert(zp_start_lease_task(z_session_loan_mut(server), NULL) == Z_OK);
}

int main(int argc, char **argv)
{
    if (argc != 2 || (strcmp(argv[1], "--expect-missing-reopen-event") != 0 &&
        strcmp(argv[1], "--require-reopen-event") != 0)) {
        fprintf(stderr, "Usage: %s <--expect-missing-reopen-event|--require-reopen-event>\n", argv[0]);
        return 2;
    }
    bool expect_missing = strcmp(argv[1], "--expect-missing-reopen-event") == 0;
    setvbuf(stdout, NULL, _IONBF, 0);
    event_counts_t original = {0};
    event_counts_t snapshot = {0};
    atomic_uint client_received = 0;
    atomic_uint server_received = 0;
    z_owned_session_t server;
    open_server(&server);
    picoros_interface_t interface = {.mode = "client", .locator = "tcp/127.0.0.1:17468"};
    assert(picoros_interface_init(&interface) == PICOROS_OK);
    z_owned_transport_events_listener_t listener;
    assert(picoros_connectivity_observe(&listener, observe_event, &original));
    assert(atomic_load(&original.opened) == 1);
    assert(atomic_load(&original.closed) == 0);
    assert(transport_count() == 1);

    z_owned_subscriber_t client_subscriber;
    z_owned_subscriber_t server_subscriber;
    subscribe(picoros_interface_session(), "connectivity_gap/to_client", &client_subscriber, &client_received);
    subscribe(z_session_loan(&server), "connectivity_gap/to_server", &server_subscriber, &server_received);
    deliver(z_session_loan(&server), "connectivity_gap/to_client", &client_received, 10);
    deliver(picoros_interface_session(), "connectivity_gap/to_server", &server_received, 11);
    puts("BASELINE: client transport=1, history opened=1 closed=0; fresh payloads 10/11 delivered.");

    z_subscriber_drop(z_subscriber_move(&server_subscriber));
    z_session_drop(z_session_move(&server));
    assert(wait_count(&original.closed, 1));
    open_server(&server);
    subscribe(z_session_loan(&server), "connectivity_gap/to_server", &server_subscriber, &server_received);
    deliver(z_session_loan(&server), "connectivity_gap/to_client", &client_received, 20);
    deliver(picoros_interface_session(), "connectivity_gap/to_server", &server_received, 21);
    assert(transport_count() == 1);

    z_owned_transport_events_listener_t history_listener;
    assert(picoros_connectivity_observe(&history_listener, observe_event, &snapshot));
    assert(atomic_load(&snapshot.opened) == 1);
    assert(atomic_load(&snapshot.closed) == 0);
    unsigned int reopened = atomic_load(&original.opened);
    printf("RECONNECTED: transport=1, original opened=%u closed=%u, new history opened=%u; fresh payloads 20/21 delivered.\n",
        reopened, atomic_load(&original.closed), atomic_load(&snapshot.opened));
    assert(atomic_load(&original.closed) == 1);

    assert(z_undeclare_transport_events_listener(z_transport_events_listener_move(&history_listener)) == Z_OK);
    assert(z_undeclare_transport_events_listener(z_transport_events_listener_move(&listener)) == Z_OK);
    z_subscriber_drop(z_subscriber_move(&client_subscriber));
    picoros_interface_close();
    z_subscriber_drop(z_subscriber_move(&server_subscriber));
    z_session_drop(z_session_move(&server));

    if (expect_missing && reopened == 1) {
        puts("PASS: reproduced the known client reconnect notification gap, not a healthy connectivity result.");
        return 0;
    }
    if (!expect_missing && reopened == 2) {
        puts("PASS: original listener received the client reconnect opening event.");
        return 0;
    }
    fprintf(stderr, "FAIL: expected opened=%u after reconnect, observed %u.\n", expect_missing ? 1 : 2, reopened);
    return 1;
}
