#ifndef INPUT_H
#define INPUT_H

// ----------------------------------------------------
// Page navigation (hardware-independent, unit testable)
// ----------------------------------------------------

enum class DisplayMode
{
    TEMPERATURE,
    HUMIDITY,
    LIGHT,
    MOTION
};

// Clockwise:         TEMPERATURE -> HUMIDITY -> LIGHT -> MOTION -> TEMPERATURE
// Counter-clockwise: the reverse, including wrap-around.
DisplayMode nextDisplayMode(DisplayMode mode);
DisplayMode previousDisplayMode(DisplayMode mode);

const char *displayModeToString(DisplayMode mode);

// ----------------------------------------------------
// Task (input.cpp, hardware side)
// ----------------------------------------------------

// InputTask: rotary-encoder navigation. The encoder ISR posts +1/-1
// into encoderQueue; InputTask turns that into a DisplayMode.
void inputTask(void *parameter);

#endif // INPUT_H