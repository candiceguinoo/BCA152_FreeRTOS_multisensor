
#ifndef ALARM_H
#define ALARM_H

// ----------------------------------------------------
// Pure decision logic (alarm.cpp, top part)
// No ESP-IDF includes; unit testable on a PC.
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

#ifndef PIO_UNIT_TESTING

// Configure the buzzer output (silent).
// Call once from app_main.
void alarmInit();

// AlarmTask: evaluates temperature and owns the buzzer.
void alarmTask(void *parameter);

#endif // PIO_UNIT_TESTING

#endif // ALARM_H
