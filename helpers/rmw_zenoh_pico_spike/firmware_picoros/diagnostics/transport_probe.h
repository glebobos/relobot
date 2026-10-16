#ifndef PICOROS_TRANSPORT_PROBE_H
#define PICOROS_TRANSPORT_PROBE_H

typedef enum {
    TRANSPORT_PROBE_RECENT_PROGRESS,
    TRANSPORT_PROBE_SERIAL_WAIT,
    TRANSPORT_PROBE_NO_EXECUTOR_PROGRESS,
} transport_probe_status_t;

transport_probe_status_t transport_probe_status(void);

#endif
