#ifndef ALARM_H
#define ALARM_H

// ----------------------------------------------------
// Pure decision logic (alarm.cpp, top part) - no ESP-IDF includes,
// unit testable on a PC.
// ----------------------------------------------------

enum class AlarmState
{
    NORMAL,
    LOW_TEMPERATURE,
    HIGH_TEMPERATURE
};

AlarmState evaluateTemperature(float temperature);

const char *alarmStateToString(AlarmState state);

// ----------------------------------------------------
// Hardware / task side (alarm.cpp, bottom part)
// ----------------------------------------------------

// Configure the buzzer output (silent). Call once from app_main.
void alarmInit();

// AlarmTask: evaluates temperature, owns the buzzer.
void alarmTask(void *parameter);

#endif // ALARM_H