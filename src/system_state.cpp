#include "system_state.h"

volatile SystemState systemState = SystemState::ACTIVE;
volatile bool motionDetected = false;

SystemState evaluateSystemState(
    SystemState current,
    bool motionPresent,
    int64_t microsSinceLastMotion
)
{
    if (motionPresent)
    {
        return SystemState::ACTIVE;
    }

    if (
        current == SystemState::ACTIVE &&
        microsSinceLastMotion >= INACTIVITY_TIMEOUT_US
    )
    {
        return SystemState::INACTIVE;
    }

    return current;
}