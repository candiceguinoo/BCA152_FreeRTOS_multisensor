#include <unity.h>

#include "input.h"

// Requirement FR-06: the rotary encoder shall switch among Temperature,
// Humidity, Light and Motion. Clockwise = next page, counter-clockwise =
// previous page, both wrapping around.
// These tests exercise nextDisplayMode() and previousDisplayMode().

void setUp(void) {}
void tearDown(void) {}

static void assertMode(DisplayMode expected, DisplayMode actual)
{
    TEST_ASSERT_EQUAL_STRING(
        displayModeToString(expected),
        displayModeToString(actual)
    );
}

void test_clockwise_moves_to_next_page(void)
{
    assertMode(DisplayMode::HUMIDITY, nextDisplayMode(DisplayMode::TEMPERATURE));
    assertMode(DisplayMode::LIGHT,    nextDisplayMode(DisplayMode::HUMIDITY));
    assertMode(DisplayMode::MOTION,   nextDisplayMode(DisplayMode::LIGHT));
}

void test_clockwise_wraps_motion_to_temperature(void)
{
    assertMode(DisplayMode::TEMPERATURE, nextDisplayMode(DisplayMode::MOTION));
}

void test_counter_clockwise_moves_to_previous_page(void)
{
    assertMode(DisplayMode::TEMPERATURE, previousDisplayMode(DisplayMode::HUMIDITY));
    assertMode(DisplayMode::HUMIDITY,    previousDisplayMode(DisplayMode::LIGHT));
    assertMode(DisplayMode::LIGHT,       previousDisplayMode(DisplayMode::MOTION));
}

void test_counter_clockwise_wraps_temperature_to_motion(void)
{
    assertMode(DisplayMode::MOTION, previousDisplayMode(DisplayMode::TEMPERATURE));
}

void test_four_clockwise_steps_return_to_start(void)
{
    DisplayMode mode = DisplayMode::LIGHT;

    for (int i = 0; i < 4; i++)
    {
        mode = nextDisplayMode(mode);
    }

    assertMode(DisplayMode::LIGHT, mode);
}

void test_previous_undoes_next_for_every_page(void)
{
    const DisplayMode all[] = {
        DisplayMode::TEMPERATURE,
        DisplayMode::HUMIDITY,
        DisplayMode::LIGHT,
        DisplayMode::MOTION
    };

    for (DisplayMode start : all)
    {
        assertMode(start, previousDisplayMode(nextDisplayMode(start)));
        assertMode(start, nextDisplayMode(previousDisplayMode(start)));
    }
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_clockwise_moves_to_next_page);
    RUN_TEST(test_clockwise_wraps_motion_to_temperature);
    RUN_TEST(test_counter_clockwise_moves_to_previous_page);
    RUN_TEST(test_counter_clockwise_wraps_temperature_to_motion);
    RUN_TEST(test_four_clockwise_steps_return_to_start);
    RUN_TEST(test_previous_undoes_next_for_every_page);

    return UNITY_END();
}