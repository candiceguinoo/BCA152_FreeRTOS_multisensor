#ifndef SYSTEM_STATE_H
#define SYSTEM_STATE_H

#include <stdint.h>

// ACTIVE/INACTIVE state machine (FR-08, FR-09, FR-10).
// This module has no ESP-IDF includes, so the decision logic can be
// unit tested on a PC.

enum class SystemState
{
    ACTIVE,
    INACTIVE
};

// No motion for this long -> INACTIVE. 15 s for laboratory testing.
constexpr int64_t INACTIVITY_TIMEOUT_US = 15000000LL;

// Pure decision function (no side effects):
//   motionPresent            - PIR output is currently high
//   microsSinceLastMotion    - time since the PIR was last high
//
//   motion present                  -> ACTIVE   (from either state)
//   ACTIVE,   timeout reached       -> INACTIVE
//   ACTIVE,   timeout not reached   -> ACTIVE
//   INACTIVE, no motion             -> INACTIVE
SystemState evaluateSystemState(
    SystemState current,
    bool motionPresent,
    int64_t microsSinceLastMotion
);

// Shared state. Single writer: MotionTask. Readers: SensorTask,
// DisplayTask. Both are word-sized values, so a single read or
// write is atomic on the ESP32 and cannot be torn.
extern volatile SystemState systemState;
extern volatile bool motionDetected;

#endif // SYSTEM_STATE_H