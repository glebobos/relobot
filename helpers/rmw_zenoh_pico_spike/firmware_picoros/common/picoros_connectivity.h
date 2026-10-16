#ifndef PICOROS_CONNECTIVITY_H
#define PICOROS_CONNECTIVITY_H

#include <picoros.h>

bool picoros_connectivity_observe(z_owned_transport_events_listener_t *listener,
    z_closure_transport_event_callback_t callback, void *context);

#endif
