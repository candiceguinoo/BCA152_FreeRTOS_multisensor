#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "alarm.h"
#include "sensors.h"
#include "input.h"
#include "system_state.h"
#include "rtos_objects.h"
#include "motion.h"
#include "display.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "esp_attr.h"
#include "esp_err.h"
// ====================================================
// PIN DEFINITIONS
// ====================================================
#define DHT_PIN GPIO_NUM_4
#define LDR_ADC_CHANNEL ADC_CHANNEL_6 // GPIO34
#define OLED_SDA GPIO_NUM_21
#define OLED_SCL GPIO_NUM_22
#define OLED_ADDR 0x3C
#define I2C_PORT I2C_NUM_0
// Rotary Encoder
#define ENCODER_CLK GPIO_NUM_32
#define ENCODER_DT    GPIO_NUM_33
#define ENCODER_SW    GPIO_NUM_25
// PIR motion sensor
#define PIR_PIN GPIO_NUM_27
// ====================================================
// DHT22 SETTINGS
// ====================================================
#define DHT_TIMEOUT_US 150
#define DHT_BIT_TIMEOUT_US 120
#define DHT_ONE_THRESHOLD_US 50
// ====================================================
// ENCODER SETTINGS
// ====================================================
constexpr bool ENCODER_REVERSE = false;
// ====================================================
// MAIN
// ====================================================
extern "C" void app_main(void)
{
serialPrintf("\n");
serialPrintf(
"====================================\n"
);
serialPrintf(
"BCA152 FreeRTOS Multisensor\n"
);
serialPrintf(
"System starting...\n"
);
serialPrintf(
"====================================\n"
);
// --------------------------------------------
// SENSORS
// --------------------------------------------
sensorsInit();
// --------------------------------------------
// PIR
// --------------------------------------------
motionInit();
// --------------------------------------------
// BUZZER INITIALIZATION
// --------------------------------------------
alarmInit();
// --------------------------------------------
// CREATE FREERTOS OBJECTS
// --------------------------------------------
if (!rtosObjectsCreate())
{
return;
}
// --------------------------------------------
// SYSTEM STARTS ACTIVE
// --------------------------------------------
xEventGroupSetBits(
systemEvents,
EVENT_ACTIVE
);
serialPrintf(
"EVENT_ACTIVE SET - system startup\n"
);
// --------------------------------------------
// CPU ASSIGNMENT
// --------------------------------------------
//
// CPU0:
// MotionTask    (priority 3)
// SensorTask    (priority 2)
//
// CPU1:
// InputTask     (priority 3)
// AlarmTask     (priority 2)
// DisplayTask (priority 1)
//
// --------------------------------------------
BaseType_t result;
// --------------------------------------------
// SENSOR TASK
// --------------------------------------------
result =
xTaskCreatePinnedToCore(
sensorTask,
"SensorTask",
6144,
NULL,
PRIORITY_SENSOR,
NULL,
0
);
if (result != pdPASS)
{
serialPrintf(
"ERROR: SensorTask creation failed\n"
);
return;
}
// --------------------------------------------
// MOTION TASK
// --------------------------------------------
result =
xTaskCreatePinnedToCore(
motionTask,
"MotionTask",
4096,
NULL,
PRIORITY_MOTION,
NULL,
0
);
if (result != pdPASS)
{
serialPrintf(
"ERROR: MotionTask creation failed\n"
);
return;
}
// --------------------------------------------
// INPUT TASK
// --------------------------------------------
result =
xTaskCreatePinnedToCore(
inputTask,
"InputTask",
4096,
NULL,
PRIORITY_INPUT,
NULL,
1
);
if (result != pdPASS)
{
serialPrintf(
"ERROR: InputTask creation failed\n"
);
return;
}
// --------------------------------------------
// DISPLAY TASK
// --------------------------------------------
result =
xTaskCreatePinnedToCore(
displayTask,
"DisplayTask",
4096,
NULL,
PRIORITY_DISPLAY,
NULL,
1
);
if (result != pdPASS)
{
serialPrintf(
"ERROR: DisplayTask creation failed\n"
);
return;
}
// --------------------------------------------
// ALARM TASK
// --------------------------------------------
result =
xTaskCreatePinnedToCore(
alarmTask,
"AlarmTask",
4096,
NULL,
PRIORITY_ALARM,
NULL,
1
);
if (result != pdPASS)
{
serialPrintf(
"ERROR: AlarmTask creation failed\n"
);
return;
}
serialPrintf(
"All tasks started successfully.\n"
);
}
