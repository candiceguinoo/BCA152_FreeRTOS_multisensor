#include "alarm.h"

// ====================================================
// ALARM DECISION LOGIC (pure - compiled for unit tests too)
// ====================================================

namespace
{
    constexpr float LOW_TEMPERATURE_LIMIT = 18.0f;
    constexpr float HIGH_TEMPERATURE_LIMIT = 30.0f;
}

AlarmState evaluateTemperature(float temperature)
{
    if (temperature < LOW_TEMPERATURE_LIMIT)
    {
        return AlarmState::LOW_TEMPERATURE;
    }

    if (temperature > HIGH_TEMPERATURE_LIMIT)
    {
        return AlarmState::HIGH_TEMPERATURE;
    }

    return AlarmState::NORMAL;
}

const char *alarmStateToString(AlarmState state)
{
    switch (state)
    {
        case AlarmState::NORMAL:
            return "NORMAL";

        case AlarmState::LOW_TEMPERATURE:
            return "LOW_TEMPERATURE";

        case AlarmState::HIGH_TEMPERATURE:
            return "HIGH_TEMPERATURE";
    }

    return "UNKNOWN";
}