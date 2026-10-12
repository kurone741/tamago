
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

extern "C" void app_main(void)
{
    printf("tamago booted successfully, nya~\n");

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

