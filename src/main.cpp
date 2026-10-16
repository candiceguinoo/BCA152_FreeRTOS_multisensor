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

#include "driver/ledc.h"

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

#define LDR_ADC_CHANNEL ADC_CHANNEL_6   // GPIO34

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

// Buzzer (driven by LEDC PWM; owned exclusively by AlarmTask)

#define BUZZER_PIN                GPIO_NUM_26

#define BUZZER_LEDC_MODE          LEDC_LOW_SPEED_MODE

#define BUZZER_LEDC_TIMER         LEDC_TIMER_0

#define BUZZER_LEDC_CHANNEL       LEDC_CHANNEL_0

#define BUZZER_DUTY_ON            512      // 50% of 10-bit range

#define BUZZER_FREQ_HIGH_TEMP     2000     // Hz: high-pitched = too hot

#define BUZZER_FREQ_LOW_TEMP      800      // Hz: low-pitched   = too cold

#define BUZZER_BEEP_MS            250      // on/off time of each beep

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

// BUZZER DRIVER

// ====================================================

//

// Only AlarmTask calls buzzer_on()/buzzer_off() after startup, so

// the buzzer has a single owner and needs no mutex. The alarm

// DECISION lives in alarm.cpp (evaluateTemperature); this section

// only turns a decision into sound.

static void buzzer_init()

{

    ledc_timer_config_t timer = {};

    timer.speed_mode          = BUZZER_LEDC_MODE;

    timer.duty_resolution = LEDC_TIMER_10_BIT;

    timer.timer_num           = BUZZER_LEDC_TIMER;

    timer.freq_hz             = BUZZER_FREQ_HIGH_TEMP;

    timer.clk_cfg             = LEDC_AUTO_CLK;

    ledc_timer_config(&timer);

    ledc_channel_config_t channel = {};

    channel.gpio_num      = BUZZER_PIN;

    channel.speed_mode = BUZZER_LEDC_MODE;

    channel.channel       = BUZZER_LEDC_CHANNEL;

    channel.intr_type     = LEDC_INTR_DISABLE;

    channel.timer_sel     = BUZZER_LEDC_TIMER;

    channel.duty          = 0;   // silent at startup

    channel.hpoint        = 0;

    ledc_channel_config(&channel);

}

static void buzzer_on(uint32_t frequencyHz)

{

    ledc_set_freq(

         BUZZER_LEDC_MODE,

         BUZZER_LEDC_TIMER,

         frequencyHz

    );

    ledc_set_duty(

         BUZZER_LEDC_MODE,

         BUZZER_LEDC_CHANNEL,

         BUZZER_DUTY_ON

    );

    ledc_update_duty(

         BUZZER_LEDC_MODE,

         BUZZER_LEDC_CHANNEL

    );

}

static void buzzer_off()

{

     ledc_set_duty(

          BUZZER_LEDC_MODE,

          BUZZER_LEDC_CHANNEL,

          0

     );

     ledc_update_duty(

          BUZZER_LEDC_MODE,

          BUZZER_LEDC_CHANNEL

     );

}

static uint32_t buzzerFrequencyFor(AlarmState state)

{

     return (state == AlarmState::HIGH_TEMPERATURE)

          ? BUZZER_FREQ_HIGH_TEMP

          : BUZZER_FREQ_LOW_TEMP;

}

// ====================================================

// ALARM TASK

// ====================================================

//

// Blocks forever on alarmQueue while the temperature is normal.

// While an alarm is active it wakes every BUZZER_BEEP_MS (queue

// timeout) to toggle the beep, and it also checks EVENT_ACTIVE on

// every wake-up so the buzzer goes silent promptly when the system

// becomes INACTIVE.

void alarmTask(void *parameter)

{

     SensorData sensorData = {};

     AlarmState lastState =

          AlarmState::NORMAL;

// Alarm currently being signalled (NORMAL = silent).

     AlarmState soundingState =

     AlarmState::NORMAL;

bool beepOn = false;

bool firstEvaluation = true;

serialPrintf(

     "AlarmTask: Started on CPU %d\n",

     xPortGetCoreID()

);

while (1)

{

     TickType_t waitTime =

         (soundingState == AlarmState::NORMAL)

             ? portMAX_DELAY

             : pdMS_TO_TICKS(BUZZER_BEEP_MS);

     if (

         xQueueReceive(

             alarmQueue,

             &sensorData,

             waitTime

         ) != pdPASS

     )

     {

// Timeout: only happens while an alarm is sounding.

         if (

             (xEventGroupGetBits(systemEvents)

                & EVENT_ACTIVE) == 0

         )

         {

             buzzer_off();

             beepOn = false;

             soundingState = AlarmState::NORMAL;

             firstEvaluation = true;

             serialPrintf(

                  "Buzzer: OFF - system inactive\n"

             );

         }

    else

    {

           beepOn = !beepOn;

           if (beepOn)

           {

                buzzer_on(

                     buzzerFrequencyFor(soundingState)

                );

           }

           else

           {

                buzzer_off();

           }

    }

    continue;

}

// Alarm is active only when EVENT_ACTIVE is set.

if (

    (xEventGroupGetBits(systemEvents)

       & EVENT_ACTIVE) == 0

)

{

    xEventGroupClearBits(

           systemEvents,

           EVENT_ALARM

    );

    if (soundingState != AlarmState::NORMAL)

    {

           buzzer_off();

           beepOn = false;

           soundingState = AlarmState::NORMAL;

           serialPrintf(

                "Buzzer: OFF - system inactive\n"

           );

    }

    firstEvaluation = true;

    continue;

}

AlarmState state =

    evaluateTemperature(

         sensorData.temperature

    );

// --------------------------------------------

// UPDATE EVENT_ALARM

// --------------------------------------------

EventBits_t currentEvents =

    xEventGroupGetBits(systemEvents);

bool alarmActive =

    state == AlarmState::LOW_TEMPERATURE ||

    state == AlarmState::HIGH_TEMPERATURE;

if (alarmActive)

{

    if (

         (currentEvents & EVENT_ALARM) == 0

    )

    {

         xEventGroupSetBits(

              systemEvents,

              EVENT_ALARM

         );

         serialPrintf(

              "EVENT_ALARM SET\n"

         );

    }

}

else

{

    if (currentEvents & EVENT_ALARM)

    {

         xEventGroupClearBits(

              systemEvents,

              EVENT_ALARM

         );

         serialPrintf(

              "EVENT_ALARM CLEARED\n"

         );

    }

}

// --------------------------------------------

// ALARM STATE CHANGE -> BUZZER

// --------------------------------------------

if (

    firstEvaluation ||

    state != lastState

)

{

    serialPrintf(

         "Alarm State: %s (%.1f C)\n",

         alarmStateToString(state),

         sensorData.temperature

    );

    soundingState = state;

    if (alarmActive)

    {

         beepOn = true;

         buzzer_on(

              buzzerFrequencyFor(state)

         );

         serialPrintf(

              "Buzzer: ON (%u Hz)\n",

                          (unsigned)buzzerFrequencyFor(state)

                     );

              }

              else

              {

                     beepOn = false;

                     buzzer_off();

                     serialPrintf(

                          "Buzzer: OFF\n"

                     );

              }

              lastState = state;

              firstEvaluation = false;

          }

    }

}

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

// BUZZER (LEDC PWM, silent at startup)

// --------------------------------------------

buzzer_init();

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
