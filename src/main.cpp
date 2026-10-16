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
// OLED
// ====================================================


static bool i2cErrorLogged = false;


static void report_i2c_error(esp_err_t err)
{
    if (err != ESP_OK && !i2cErrorLogged)
    {
           i2cErrorLogged = true;

        serialPrintf(
             "OLED I2C error: %s "
             "(check SDA=21, SCL=22, address 0x3C)\n",
             esp_err_to_name(err)
        );
    }
}


static void oled_command(uint8_t command)
{
    uint8_t data[2] = {0x00, command};


    esp_err_t err =
        i2c_master_write_to_device(
             I2C_PORT,
             OLED_ADDR,
             data,
             sizeof(data),

             pdMS_TO_TICKS(100)
        );


    report_i2c_error(err);
}


static void oled_data(const uint8_t *data, size_t length)
{
    uint8_t buffer[129];


    if (length > 128)
        return;


    buffer[0] = 0x40;


    for (size_t i = 0; i < length; i++)
        buffer[i + 1] = data[i];


    esp_err_t err =
        i2c_master_write_to_device(
             I2C_PORT,

              OLED_ADDR,
              buffer,
              length + 1,
              pdMS_TO_TICKS(100)
         );


    report_i2c_error(err);
}


static void oled_init()

{
    i2c_config_t config = {};


    config.mode = I2C_MODE_MASTER;
    config.sda_io_num = OLED_SDA;
    config.scl_io_num = OLED_SCL;
    config.sda_pullup_en = GPIO_PULLUP_ENABLE;
    config.scl_pullup_en = GPIO_PULLUP_ENABLE;
    config.master.clk_speed = 400000;


    esp_err_t err = i2c_param_config(I2C_PORT, &config);


    if (err != ESP_OK)
    {
         serialPrintf(
              "i2c_param_config failed: %s\n",
              esp_err_to_name(err)
         );
    }


    err = i2c_driver_install(
         I2C_PORT,
         config.mode,
         0,
         0,
         0
    );


    if (err != ESP_OK)
    {

        serialPrintf(
             "i2c_driver_install failed: %s\n",
             esp_err_to_name(err)
        );

    }


    vTaskDelay(pdMS_TO_TICKS(100));


    oled_command(0xAE);
    oled_command(0xD5);
    oled_command(0x80);
    oled_command(0xA8);
    oled_command(0x3F);
    oled_command(0xD3);
    oled_command(0x00);
    oled_command(0x40);
    oled_command(0x8D);
    oled_command(0x14);
    oled_command(0x20);
    oled_command(0x00);
    oled_command(0xA1);
    oled_command(0xC8);
    oled_command(0xDA);
    oled_command(0x12);
    oled_command(0x81);
    oled_command(0xCF);
    oled_command(0xD9);
    oled_command(0xF1);
    oled_command(0xDB);
    oled_command(0x40);
    oled_command(0xA4);
    oled_command(0xA6);
    oled_command(0xAF);
}


static void oled_power(bool on)
{
    oled_command(on ? 0xAF : 0xAE);
}

static void oled_clear()

{
    uint8_t blank[128] = {0};


    for (int page = 0; page < 8; page++)
    {
         oled_command(0xB0 + page);
         oled_command(0x00);
         oled_command(0x10);
         oled_data(blank, 128);
    }
}


static void oled_set_cursor(int x, int page)
{
    oled_command(0xB0 + page);
    oled_command(0x00 + (x & 0x0F));
    oled_command(0x10 + ((x >> 4) & 0x0F));
}


// ====================================================
// OLED FONT
// ====================================================


static void get_glyph(char c, uint8_t p[6])
{
    for (int i = 0; i < 6; i++)
         p[i] = 0;


    switch (c)
    {
         case 'A': p[0]=0x7E; p[1]=0x11; p[2]=0x11; p[3]=0x11; p[4]=0x7E;
break;
         case 'B': p[0]=0x7F; p[1]=0x49; p[2]=0x49; p[3]=0x49; p[4]=0x36;
break;
         case 'C': p[0]=0x3E; p[1]=0x41; p[2]=0x41; p[3]=0x41; p[4]=0x22;

break;
         case 'D': p[0]=0x7F; p[1]=0x41; p[2]=0x41; p[3]=0x22; p[4]=0x1C;
break;

         case 'E': p[0]=0x7F; p[1]=0x49; p[2]=0x49; p[3]=0x49; p[4]=0x41;
break;
         case 'F': p[0]=0x7F; p[1]=0x09; p[2]=0x09; p[3]=0x09; p[4]=0x01;
break;
         case 'G': p[0]=0x3E; p[1]=0x41; p[2]=0x49; p[3]=0x49; p[4]=0x7A;
break;
         case 'H': p[0]=0x7F; p[1]=0x08; p[2]=0x08; p[3]=0x08; p[4]=0x7F;
break;
         case 'I': p[0]=0x00; p[1]=0x41; p[2]=0x7F; p[3]=0x41; p[4]=0x00;
break;
         case 'J': p[0]=0x20; p[1]=0x40; p[2]=0x41; p[3]=0x3F; p[4]=0x01;
break;
         case 'K': p[0]=0x7F; p[1]=0x08; p[2]=0x14; p[3]=0x22; p[4]=0x41;
break;
         case 'L': p[0]=0x7F; p[1]=0x40; p[2]=0x40; p[3]=0x40; p[4]=0x40;
break;
         case 'M': p[0]=0x7F; p[1]=0x02; p[2]=0x0C; p[3]=0x02; p[4]=0x7F;
break;
         case 'N': p[0]=0x7F; p[1]=0x02; p[2]=0x0C; p[3]=0x18; p[4]=0x7F;
break;
         case 'O': p[0]=0x3E; p[1]=0x41; p[2]=0x41; p[3]=0x41; p[4]=0x3E;
break;
         case 'P': p[0]=0x7F; p[1]=0x09; p[2]=0x09; p[3]=0x09; p[4]=0x06;
break;
         case 'Q': p[0]=0x3E; p[1]=0x41; p[2]=0x51; p[3]=0x21; p[4]=0x5E;
break;
         case 'R': p[0]=0x7F; p[1]=0x09; p[2]=0x19; p[3]=0x29; p[4]=0x46;
break;
         case 'S': p[0]=0x46; p[1]=0x49; p[2]=0x49; p[3]=0x49; p[4]=0x31;
break;
         case 'T': p[0]=0x01; p[1]=0x01; p[2]=0x7F; p[3]=0x01; p[4]=0x01;
break;
         case 'U': p[0]=0x3F; p[1]=0x40; p[2]=0x40; p[3]=0x40; p[4]=0x3F;
break;
         case 'V': p[0]=0x1F; p[1]=0x20; p[2]=0x40; p[3]=0x20; p[4]=0x1F;
break;

         case 'W': p[0]=0x7F; p[1]=0x20; p[2]=0x18; p[3]=0x20; p[4]=0x7F;
break;
         case 'X': p[0]=0x63; p[1]=0x14; p[2]=0x08; p[3]=0x14; p[4]=0x63;
break;

         case 'Y': p[0]=0x07; p[1]=0x08; p[2]=0x70; p[3]=0x08; p[4]=0x07;
break;
         case 'Z': p[0]=0x61; p[1]=0x51; p[2]=0x49; p[3]=0x45; p[4]=0x43;
break;


         case '0': p[0]=0x3E; p[1]=0x51; p[2]=0x49; p[3]=0x45; p[4]=0x3E;
break;
         case '1': p[0]=0x00; p[1]=0x42; p[2]=0x7F; p[3]=0x40; p[4]=0x00;
break;
         case '2': p[0]=0x42; p[1]=0x61; p[2]=0x51; p[3]=0x49; p[4]=0x46;
break;
         case '3': p[0]=0x21; p[1]=0x41; p[2]=0x45; p[3]=0x4B; p[4]=0x31;
break;
         case '4': p[0]=0x18; p[1]=0x14; p[2]=0x12; p[3]=0x7F; p[4]=0x10;
break;
         case '5': p[0]=0x27; p[1]=0x45; p[2]=0x45; p[3]=0x45; p[4]=0x39;
break;
         case '6': p[0]=0x3C; p[1]=0x4A; p[2]=0x49; p[3]=0x49; p[4]=0x30;
break;
         case '7': p[0]=0x01; p[1]=0x71; p[2]=0x09; p[3]=0x05; p[4]=0x03;
break;
         case '8': p[0]=0x36; p[1]=0x49; p[2]=0x49; p[3]=0x49; p[4]=0x36;
break;
         case '9': p[0]=0x06; p[1]=0x49; p[2]=0x49; p[3]=0x29; p[4]=0x1E;
break;


         case '%': p[0]=0x63; p[1]=0x13; p[2]=0x08; p[3]=0x64; p[4]=0x63;
break;
         case '.': p[0]=0x00; p[1]=0x60; p[2]=0x60; p[3]=0x00; p[4]=0x00;
break;
         case '-': p[0]=0x08; p[1]=0x08; p[2]=0x08; p[3]=0x08; p[4]=0x08;
break;


         default:
            break;
    }
}


static void oled_char(int x, int page, char c)
{

    uint8_t p[6];


    get_glyph(c, p);


    oled_set_cursor(x, page);
    oled_data(p, 6);
}


static void oled_text(int x, int page, const char *text)
{
    while (*text)
    {
        oled_char(x, page, *text);
        x += 6;
        text++;
    }
}


static void oled_char_big(int x, int page, char c)
{
    uint8_t g[6];
    uint8_t top[12];
    uint8_t bottom[12];


    get_glyph(c, g);


    for (int i = 0; i < 6; i++)
    {
        uint8_t low = 0;
        uint8_t high = 0;


        for (int b = 0; b < 4; b++)
        {
            if (g[i] & (1 << b))
                  low |= (uint8_t)(3 << (2 * b));


            if (g[i] & (1 << (b + 4)))
                  high |= (uint8_t)(3 << (2 * b));
        }

        top[2 * i] = low;
        top[2 * i + 1] = low;
        bottom[2 * i] = high;
        bottom[2 * i + 1] = high;
    }


    oled_set_cursor(x, page);
    oled_data(top, 12);


    oled_set_cursor(x, page + 1);
    oled_data(bottom, 12);
}


static void oled_text_big(int x, int page, const char *text)
{
    while (*text)
    {
        oled_char_big(x, page, *text);
        x += 12;

        text++;
    }
}


// ====================================================
// DISPLAY PAGE DRAWING
// ====================================================


static void drawPage(
    DisplayMode mode,
    const SensorData &data
)
{
    char line[16];


    oled_clear();


    oled_text(
        0,
        0,

     "ROOM MONITOR"
);


switch (mode)
{
     case DisplayMode::TEMPERATURE:


        oled_text_big(
             0,
             2,
             "TEMP"
        );


        snprintf(
             line,
             sizeof(line),
             "%.1f C",
             data.temperature

        );


        oled_text_big(
             0,
             5,
             line
        );


        serialPrintf(
             "DisplayTask: "
             "Temperature page = %.2f C\n",
             data.temperature
        );


        break;


     case DisplayMode::HUMIDITY:


        oled_text_big(
             0,
             2,
             "HUMIDITY"

   );


   snprintf(
        line,
        sizeof(line),
        "%d %%",
        (int)data.humidity
   );


   oled_text_big(

        0,
        5,
        line
   );


   serialPrintf(
        "DisplayTask: "
        "Humidity page = %.2f %%\n",
        data.humidity
   );


   break;


case DisplayMode::LIGHT:


   oled_text_big(
        0,
        2,
        "LIGHT"
   );


   snprintf(
        line,
        sizeof(line),
        "%d %%",
        data.lightLevel
   );


   oled_text_big(
        0,

                5,
                line
           );


           serialPrintf(
                "DisplayTask: "
                "Light page = %d %%\n",
                data.lightLevel
           );


           break;


        case DisplayMode::MOTION:


           oled_text_big(
                0,
                2,
                "MOTION"
           );


           oled_text_big(
                0,
                5,
                motionDetected
                     ? "DETECTED"
                     : "NONE"
           );


           serialPrintf(
                "DisplayTask: Motion page = %s\n",
                motionDetected
                     ? "DETECTED"
                     : "NONE"
           );


           break;

    }
}


// ====================================================

// DISPLAY TASK
// ====================================================


void displayTask(void *parameter)
{
    SensorData sensorData = {};


    DisplayMode currentMode =
         DisplayMode::TEMPERATURE;


    bool haveData = false;
    bool displayOn = true;
    bool needRedraw = true;
    bool lastMotionShown = false;


    oled_init();
    oled_clear();


    serialPrintf(
         "DisplayTask: Started on CPU %d\n",
         xPortGetCoreID()
    );


    while (1)
    {
         if (
             xQueueReceive(
                   displayQueue,

                   &sensorData,
                   ms_to_ticks(20)
             ) == pdPASS
         )
         {
             haveData = true;
             needRedraw = true;
         }


         DisplayMode newMode;


         if (

    xQueueReceive(
         modeQueue,
         &newMode,
         0
    ) == pdPASS
)
{
    currentMode = newMode;
    needRedraw = true;
}


// --------------------------------------------
// EVENT GROUP STATE
// --------------------------------------------


EventBits_t events =
    xEventGroupGetBits(systemEvents);


bool active =
    (events & EVENT_ACTIVE) != 0;


static EventBits_t previousEvents = 0;


if (events != previousEvents)
{
    serialPrintf(
         "DisplayTask Events: "
         "ACTIVE=%d MOTION=%d ALARM=%d\n",
         (events & EVENT_ACTIVE) ? 1 : 0,
         (events & EVENT_MOTION) ? 1 : 0,
         (events & EVENT_ALARM) ? 1 : 0
    );


    previousEvents = events;
}


// --------------------------------------------
// INACTIVE
// --------------------------------------------

if (!active)
{
    if (displayOn)
    {
         oled_clear();
         oled_power(false);


         displayOn = false;


         serialPrintf(
              "DisplayTask: OLED OFF - "
              "system INACTIVE\n"
         );
    }


    continue;
}


// --------------------------------------------
// ACTIVE
// --------------------------------------------


if (!displayOn)
{
    oled_power(true);


    displayOn = true;
    needRedraw = true;


    serialPrintf(
         "DisplayTask: OLED ON - "
         "system ACTIVE\n"
    );
}


if (
    currentMode == DisplayMode::MOTION &&
    motionDetected != lastMotionShown
)
{

              needRedraw = true;
         }


         if (!haveData || !needRedraw)
              continue;


         needRedraw = false;


         lastMotionShown =
              motionDetected;


#if ENABLE_TIMING_DEBUG
         int64_t drawStartUs =
              esp_timer_get_time();
#endif


         drawPage(
              currentMode,
              sensorData
         );


#if ENABLE_TIMING_DEBUG
         serialPrintf(
              "TIMING: OLED redraw took %lld us\n",
              (long long)(esp_timer_get_time() - drawStartUs)
         );
#endif
     }
}


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
