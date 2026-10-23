#include <assert.h>
#include <stddef.h>
#include <string.h>

#include <FreeRTOS.h>
#include <task.h>

#include "relobot/picoros_runtime.h"

static picoros_res_t results[3];
static size_t result_count;
static size_t attempts;
static size_t delays;

picoros_res_t picoros_interface_init(picoros_interface_t *interface)
{
    assert(interface != NULL);
    assert(strcmp(interface->mode, "client") == 0);
    assert(strcmp(interface->locator, "serial/usb#baudrate=115200") == 0);
    assert(attempts < result_count);
    return results[attempts++];
}

void vTaskDelay(TickType_t ticks)
{
    assert(ticks == pdMS_TO_TICKS(1000));
    assert(attempts > 0);
    assert(results[attempts - 1] == PICOROS_NOT_READY);
    delays++;
}

int main(int argument_count, char **arguments)
{
    assert(argument_count == 2);
    picoros_res_t expected;
    size_t expected_delays;
    if (strcmp(arguments[1], "ready") == 0) {
        results[0] = PICOROS_OK;
        result_count = 1;
        expected = PICOROS_OK;
        expected_delays = 0;
    } else if (strcmp(arguments[1], "wait_ready") == 0) {
        results[0] = PICOROS_NOT_READY;
        results[1] = PICOROS_NOT_READY;
        results[2] = PICOROS_OK;
        result_count = 3;
        expected = PICOROS_OK;
        expected_delays = 2;
    } else if (strcmp(arguments[1], "error") == 0) {
        results[0] = PICOROS_ERROR;
        result_count = 1;
        expected = PICOROS_ERROR;
        expected_delays = 0;
    } else {
        assert(strcmp(arguments[1], "wait_error") == 0);
        results[0] = PICOROS_NOT_READY;
        results[1] = PICOROS_ERROR;
        result_count = 2;
        expected = PICOROS_ERROR;
        expected_delays = 1;
    }

    assert(relobot_picoros_init() == expected);
    assert(attempts == result_count);
    assert(delays == expected_delays);
    return 0;
}
