#include <stdio.h>
#include <stdarg.h>

#include "rtos_objects.h"
#include "sensors.h"
#include "input.h"

QueueHandle_t displayQueue = NULL;
QueueHandle_t alarmQueue = NULL;
QueueHandle_t modeQueue = NULL;
QueueHandle_t encoderQueue = NULL;

SemaphoreHandle_t serialMutex = NULL;

EventGroupHandle_t systemEvents = NULL;

void serialPrintf(const char *format, ...)
{
    va_list args;

    va_start(args, format);

    if (
        serialMutex != NULL &&
        xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE
    )
    {
        vprintf(format, args);
        xSemaphoreGive(serialMutex);
    }
    else
    {
        vprintf(format, args);
    }

    va_end(args);
}

bool rtosObjectsCreate()
{
    // --------------------------------------------
    // CREATE QUEUES
    // --------------------------------------------

    displayQueue =
        xQueueCreate(
            1,
            sizeof(SensorData)
        );

    alarmQueue =
        xQueueCreate(
            1,
            sizeof(SensorData)
        );

    modeQueue =
        xQueueCreate(
            1,
            sizeof(DisplayMode)
        );

    encoderQueue =
        xQueueCreate(
            8,
            sizeof(int8_t)
        );

    if (
        displayQueue == NULL ||
        alarmQueue == NULL ||
        modeQueue == NULL ||
        encoderQueue == NULL
    )
    {
        serialPrintf(
            "ERROR: Failed to create queues\n"
        );

        return false;
    }

    serialPrintf(
        "Queues created successfully.\n"
    );

    // --------------------------------------------
    // CREATE EVENT GROUP - Part X
    // --------------------------------------------

    systemEvents =
        xEventGroupCreate();

    if (systemEvents == NULL)
    {
        serialPrintf(
            "ERROR: Failed to create Event Group\n"
        );

        return false;
    }

    serialPrintf(
        "Event Group created successfully.\n"
    );

    serialPrintf(
        "EVENT_ACTIVE = BIT0\n"
    );

    serialPrintf(
        "EVENT_MOTION = BIT1\n"
    );

    serialPrintf(
        "EVENT_ALARM = BIT2\n"
    );

    // --------------------------------------------
    // CREATE SERIAL MUTEX - Part XI
    // --------------------------------------------

    serialMutex =
        xSemaphoreCreateMutex();

    if (serialMutex == NULL)
    {
        serialPrintf(
            "ERROR: Failed to create serial mutex\n"
        );

        return false;
    }

    serialPrintf(
        "Serial mutex created successfully.\n"
    );

    return true;
}