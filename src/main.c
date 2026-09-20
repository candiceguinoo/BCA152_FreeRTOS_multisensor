#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


void taskA(void *pvParameters)
{
    while (1)
    {
        printf("Task A running\n");

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


void taskB(void *pvParameters)
{
    while (1)
    {
        printf("Task B running\n");

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}


void app_main(void)
{
    printf("BCA152 FreeRTOS Multisensor\n");
    printf("System starting...\n");

    xTaskCreate(
        taskA,
        "Task A",
        2048,
        NULL,
        1,
        NULL
    );

    xTaskCreate(
        taskB,
        "Task B",
        2048,
        NULL,
        2,
        NULL
    );
}