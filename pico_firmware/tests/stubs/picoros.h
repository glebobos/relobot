#ifndef TEST_PICOROS_H
#define TEST_PICOROS_H

typedef enum {
    PICOROS_OK,
    PICOROS_NOT_READY,
    PICOROS_ERROR,
} picoros_res_t;

typedef struct {
    char *mode;
    char *locator;
} picoros_interface_t;

picoros_res_t picoros_interface_init(picoros_interface_t *interface);

#endif
