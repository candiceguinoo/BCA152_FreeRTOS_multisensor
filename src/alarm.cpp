#include "alarm.h"

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