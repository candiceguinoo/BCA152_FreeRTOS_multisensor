#include <unity.h>

#include "system_state.h"

// Requirements FR-08, FR-09, FR-10: the system is ACTIVE or INACTIVE,
// goes INACTIVE after 15 s without motion, and motion returns it to
// ACTIVE. These tests exercise evaluateSystemState().

void setUp(void) {}
void tearDown(void) {}

static const char *stateName(SystemState state)
{
    return (state == SystemState::ACTIVE) ? "ACTIVE" : "INACTIVE";
}

static void assertState(
    SystemState expected,
    SystemState current,
    bool motionPresent,
    int64_t microsSinceLastMotion
)
{
    TEST_ASSERT_EQUAL_STRING(
        stateName(expected),
        stateName(
            evaluateSystemState(
                current,
                motionPresent,
                microsSinceLastMotion
            )
        )
    );
}

void test_active_without_timeout_stays_active(void)
{
    // 5 s without motion is well inside the 15 s window.
    assertState(SystemState::ACTIVE, SystemState::ACTIVE, false, 5000000LL);
}

void test_active_with_timeout_becomes_inactive(void)
{
    // Exactly 15 s without motion: the timeout has been reached.
    assertState(
        SystemState::INACTIVE,
        SystemState::ACTIVE,
        false,
        INACTIVITY_TIMEOUT_US
    );
}

void test_inactive_without_motion_stays_inactive(void)
{
    assertState(SystemState::INACTIVE, SystemState::INACTIVE, false, 100000000LL);
}

void test_inactive_with_motion_becomes_active(void)
{
    assertState(SystemState::ACTIVE, SystemState::INACTIVE, true, 0);
}

void test_one_microsecond_before_timeout_stays_active(void)
{
    assertState(
        SystemState::ACTIVE,
        SystemState::ACTIVE,
        false,
        INACTIVITY_TIMEOUT_US - 1
    );
}

void test_motion_keeps_system_active_even_after_long_silence(void)
{
    // Motion always wins over the elapsed-time check.
    assertState(SystemState::ACTIVE, SystemState::ACTIVE, true, 60000000LL);
}

void test_inactive_is_not_changed_by_a_short_elapsed_time(void)
{
    // Only motion wakes an INACTIVE system; time alone never does.
    assertState(SystemState::INACTIVE, SystemState::INACTIVE, false, 0);
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_active_without_timeout_stays_active);
    RUN_TEST(test_active_with_timeout_becomes_inactive);
    RUN_TEST(test_inactive_without_motion_stays_inactive);
    RUN_TEST(test_inactive_with_motion_becomes_active);
    RUN_TEST(test_one_microsecond_before_timeout_stays_active);
    RUN_TEST(test_motion_keeps_system_active_even_after_long_silence);
    RUN_TEST(test_inactive_is_not_changed_by_a_short_elapsed_time);

    return UNITY_END();
}