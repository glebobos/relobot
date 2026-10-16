#include "picoros_connectivity.h"

bool picoros_connectivity_observe(z_owned_transport_events_listener_t *listener,
    z_closure_transport_event_callback_t callback, void *context)
{
    if (listener == NULL) {
        return false;
    }
    z_internal_transport_events_listener_null(listener);
    const z_loaned_session_t *session = picoros_interface_session();
    if (session == NULL || callback == NULL) {
        return false;
    }
    z_owned_closure_transport_event_t closure;
    if (z_closure_transport_event(&closure, callback, NULL, context) != Z_OK) {
        return false;
    }
    z_transport_events_listener_options_t options;
    z_transport_events_listener_options_default(&options);
    options.history = true;
    z_result_t result = z_declare_transport_events_listener(session, listener,
        z_closure_transport_event_move(&closure), &options);
    z_closure_transport_event_drop(z_closure_transport_event_move(&closure));
    return result == Z_OK;
}
