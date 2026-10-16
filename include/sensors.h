#ifndef SENSORS_H
#define SENSORS_H

// Snapshot of all measurements. SensorTask produces it and sends it
// through displayQueue and alarmQueue (a copy is delivered to each).
// No ESP-IDF includes here, so every module can use this type.

struct SensorData
{
    float temperature;      // degrees Celsius
    float humidity;         // percent relative humidity
    int lightLevel;         // relative light, 0-100 % (NOT calibrated lux)
    bool motionDetected;
};

// Sets the DHT22 data line to its idle state. Call once from app_main.
void sensorsInit();

// SensorTask: reads DHT22 + LDR every 2 s (vTaskDelayUntil) and sends
// the result to displayQueue and alarmQueue.
void sensorTask(void *parameter);

#endif // SENSORS_H