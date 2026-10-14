#include "unity/unity.h"
#include "../src/pi_controller.h"

void setUp(void) {}
void tearDown(void) {}

void test_output_is_zero_when_error_is_zero(void)
{
    PiController pi;
    pi_init(&pi, 1.0f, 0.1f, 0.0f, 100.0f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pi_step(&pi, 50.0f, 50.0f, 0.01f));
}

void test_proportional_part_only(void)
{
    PiController pi;
    pi_init(&pi, 2.0f, 0.0f, 0.0f, 100.0f);
    /* error = 10 -> output = 2 * 10 = 20 */
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 20.0f, pi_step(&pi, 60.0f, 50.0f, 0.01f));
}

void test_integral_part_accumulates(void)
{
    PiController pi;
    pi_init(&pi, 0.0f, 1.0f, 0.0f, 100.0f);
    /* error = 10, dt = 0.1 -> integral grows by 1.0 per step */
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, pi_step(&pi, 60.0f, 50.0f, 0.1f));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 2.0f, pi_step(&pi, 60.0f, 50.0f, 0.1f));
}

void test_output_is_clamped_to_max(void)
{
    PiController pi;
    pi_init(&pi, 10.0f, 1.0f, 0.0f, 100.0f);
    TEST_ASSERT_EQUAL_FLOAT(100.0f, pi_step(&pi, 200.0f, 0.0f, 0.01f));
}

void test_output_is_clamped_to_min(void)
{
    PiController pi;
    pi_init(&pi, 10.0f, 1.0f, 0.0f, 100.0f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pi_step(&pi, 0.0f, 200.0f, 0.01f));
}

void test_anti_windup_stops_integral_growth(void)
{
    PiController pi;
    int i;
    pi_init(&pi, 10.0f, 1.0f, 0.0f, 100.0f);

    for (i = 0; i < 1000; i++) {
        (void)pi_step(&pi, 200.0f, 0.0f, 0.01f); /* saturated at max */
    }
    /* Integral must not have wound up while the output was saturated. */
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pi.integral);
}

void test_reset_clears_integral(void)
{
    PiController pi;
    pi_init(&pi, 0.0f, 1.0f, 0.0f, 100.0f);
    (void)pi_step(&pi, 60.0f, 50.0f, 0.1f);
    pi_reset(&pi);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pi.integral);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_output_is_zero_when_error_is_zero);
    RUN_TEST(test_proportional_part_only);
    RUN_TEST(test_integral_part_accumulates);
    RUN_TEST(test_output_is_clamped_to_max);
    RUN_TEST(test_output_is_clamped_to_min);
    RUN_TEST(test_anti_windup_stops_integral_growth);
    RUN_TEST(test_reset_clears_integral);
    return UNITY_END();
}
