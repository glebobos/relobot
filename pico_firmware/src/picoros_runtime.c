#include "relobot/picoros_runtime.h"

#include <FreeRTOS.h>
#include <task.h>

picoros_res_t relobot_picoros_init(void)
{
    picoros_interface_t interface = {
        .mode = "client",
        .locator = "serial/usb#baudrate=115200",
    };
    picoros_res_t status;
    do {
        status = picoros_interface_init(&interface);
        if (status == PICOROS_NOT_READY) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    } while (status == PICOROS_NOT_READY);
    return status;
}
