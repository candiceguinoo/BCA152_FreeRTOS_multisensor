#ifndef RTOS_OBJECTS_H
#define RTOS_OBJECTS_H

#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "esp_bit_defs.h"

// ====================================================
// QUEUES
// ====================================================
//
// displayQueue (depth 1, SensorData)
//     Producer: SensorTask   Consumer: DisplayTask
// alarmQueue   (depth 1, SensorData)
//     Producer: SensorTask   Consumer: AlarmTask
//     Depth 1 + xQueueOverwrite: consumers always see the NEWEST
//     reading; a slow consumer never builds up a backlog.
// modeQueue    (depth 1, DisplayMode)
//     Producer: InputTask    Consumer: DisplayTask
// encoderQueue (depth 8, int8_t direction +1/-1)
//     Producer: encoder ISR  Consumer: InputTask

extern QueueHandle_t displayQueue;
extern QueueHandle_t alarmQueue;
extern QueueHandle_t modeQueue;
extern QueueHandle_t encoderQueue;

// ====================================================
// SERIAL MUTEX - Part XI
// ====================================================
//
// Protects the console (UART). Every task prints diagnostics; without
// the mutex two tasks can interleave their characters mid-line.
// Always print through serialPrintf(), never printf() directly.

extern SemaphoreHandle_t serialMutex;

// ====================================================
// EVENT GROUP - Part X
// ====================================================
//
// EVENT_ACTIVE:
//     Producer: app_main (startup) / MotionTask
//     Consumers: DisplayTask / InputTask / AlarmTask
//     Set: system startup or motion detected
//     Clear: 15-second inactivity timeout (MotionTask)
//
// EVENT_MOTION:
//     Producer: MotionTask
//     Consumer: DisplayTask
//     Set: PIR rising edge
//     Clear: 15-second inactivity timeout (MotionTask)
//
// EVENT_ALARM:
//     Producer: AlarmTask (sets and clears)
//     Also cleared by: MotionTask on the inactivity timeout
//     Consumer: DisplayTask
//     Set: LOW_TEMPERATURE or HIGH_TEMPERATURE
//     Clear: NORMAL temperature or system inactive

#define EVENT_ACTIVE BIT0
#define EVENT_MOTION BIT1
#define EVENT_ALARM  BIT2

extern EventGroupHandle_t systemEvents;

// ====================================================
// TASK PRIORITIES - Part XII
// ====================================================
//
// Higher number = more urgent. Priority expresses how quickly a
// task must respond, NOT how "important" it is. Priorities only
// compete between tasks pinned to the SAME core:
//
//   Core 0: MotionTask (3) preempts SensorTask (2)
//   Core 1: InputTask (3) > AlarmTask (2) > DisplayTask (1)
//
// MotionTask  (3): tiny job every 100 ms; owns the inactivity timer
//                  and wake-up, must never wait behind sensor work.
// InputTask   (3): human input; encoder events must be consumed
//                  before the 8-slot encoderQueue overflows.
// SensorTask  (2): periodic, 2 s deadline; tolerates jitter but
//                  should still preempt screen drawing.
// AlarmTask   (2): runs briefly per sensor update; should preempt a
//                  screen redraw so alarm handling is prompt.
// DisplayTask (1): heaviest job (I2C redraw, tens of ms) and the
//                  least time-critical; lowest priority means it can
//                  never delay input or alarm handling.

#define PRIORITY_MOTION  3
#define PRIORITY_INPUT   3
#define PRIORITY_SENSOR  2
#define PRIORITY_ALARM   2
#define PRIORITY_DISPLAY 1

// ====================================================
// FUNCTIONS
// ====================================================

// Create every queue, the event group and the serial mutex.
// Returns false (after printing the reason) if any creation fails.
bool rtosObjectsCreate();

// printf() protected by serialMutex. Falls back to an unprotected
// print only before the mutex exists (early in app_main).
void serialPrintf(const char *format, ...);

static inline TickType_t ms_to_ticks(uint32_t ms)
{
    TickType_t ticks = pdMS_TO_TICKS(ms);

    return (ticks == 0) ? 1 : ticks;
}

#endif // RTOS_OBJECTS_H