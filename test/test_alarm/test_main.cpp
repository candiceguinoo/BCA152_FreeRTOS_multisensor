#include <unity.h>

#include "alarm.h"

// Requirement FR-07: the buzzer shall activate when the temperature is
// outside the configured normal range (LOW = 18 C, HIGH = 30 C).
// These tests exercise evaluateTemperature(), the pure decision logic.

void setUp(void) {}
void tearDown(void) {}

// Compares by name so a failure prints e.g. expected HIGH_TEMPERATURE,
// was NORMAL, instead of two bare numbers.
static void assertAlarm(AlarmState expected, float temperature)
{
    TEST_ASSERT_EQUAL_STRING(
        alarmStateToString(expected),
        alarmStateToString(evaluateTemperature(temperature))
    );
}

void test_below_lower_threshold_is_low(void)
{
    assertAlarm(AlarmState::LOW_TEMPERATURE, 17.9f);
}

void test_exactly_lower_threshold_is_normal(void)
{
    // 18.0 C is the first acceptable value: the limit itself is normal.
    assertAlarm(AlarmState::NORMAL, 18.0f);
}

void test_normal_value_is_normal(void)
{
    assertAlarm(AlarmState::NORMAL, 24.0f);
}

void test_exactly_upper_threshold_is_normal(void)
{
    // 30.0 C is the last acceptable value: the limit itself is normal.
    assertAlarm(AlarmState::NORMAL, 30.0f);
}

void test_above_upper_threshold_is_high(void)
{
    assertAlarm(AlarmState::HIGH_TEMPERATURE, 30.1f);
}

void test_sensor_range_extremes(void)
{
    // -40 C and 80 C are the limits SensorTask accepts from the DHT22.
    assertAlarm(AlarmState::LOW_TEMPERATURE, -40.0f);
    assertAlarm(AlarmState::HIGH_TEMPERATURE, 80.0f);
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_below_lower_threshold_is_low);
    RUN_TEST(test_exactly_lower_threshold_is_normal);
    RUN_TEST(test_normal_value_is_normal);
    RUN_TEST(test_exactly_upper_threshold_is_normal);
    RUN_TEST(test_above_upper_threshold_is_high);
    RUN_TEST(test_sensor_range_extremes);

    return UNITY_END();
}