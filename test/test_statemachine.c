#include "unity/unity.h"
#include "../src/cc_statemachine.h"

static CcContext ctx;
static CcInputs  in;
static CcOutputs out;

#define DT 0.01f

void setUp(void)
{
    cc_init(&ctx);
    in.main_switch = 0U;
    in.set_pressed = 0U;
    in.cancel_pressed = 0U;
    in.brake_pressed = 0U;
    in.speed_valid = 1U;
    in.speed_kmh = 0.0f;
    out.throttle_pct = 0.0f;
}

void tearDown(void) {}

/* Helper: bring the module to ACTIVE at the given speed. */
static void go_active(float speed)
{
    in.speed_kmh = speed;
    in.main_switch = 1U;
    cc_step(&ctx, &in, &out, DT);      /* OFF -> STANDBY */
    in.set_pressed = 1U;
    cc_step(&ctx, &in, &out, DT);      /* STANDBY -> ACTIVE */
    in.set_pressed = 0U;
}

void test_REQ01_starts_in_off(void)
{
    TEST_ASSERT_EQUAL_INT(CC_STATE_OFF, ctx.state);
}

void test_main_switch_moves_off_to_standby(void)
{
    in.main_switch = 1U;
    cc_step(&ctx, &in, &out, DT);
    TEST_ASSERT_EQUAL_INT(CC_STATE_STANDBY, ctx.state);
}

void test_REQ02_set_at_50_kmh_activates(void)
{
    go_active(50.0f);
    TEST_ASSERT_EQUAL_INT(CC_STATE_ACTIVE, ctx.state);
}

void test_REQ02_set_below_30_kmh_stays_standby(void)
{
    in.main_switch = 1U;
    in.speed_kmh = 29.9f;
    cc_step(&ctx, &in, &out, DT);
    in.set_pressed = 1U;
    cc_step(&ctx, &in, &out, DT);
    TEST_ASSERT_EQUAL_INT(CC_STATE_STANDBY, ctx.state);
}

void test_REQ02_set_above_180_kmh_stays_standby(void)
{
    in.main_switch = 1U;
    in.speed_kmh = 180.1f;
    cc_step(&ctx, &in, &out, DT);
    in.set_pressed = 1U;
    cc_step(&ctx, &in, &out, DT);
    TEST_ASSERT_EQUAL_INT(CC_STATE_STANDBY, ctx.state);
}

void test_REQ02_boundaries_30_and_180_are_accepted(void)
{
    go_active(30.0f);
    TEST_ASSERT_EQUAL_INT(CC_STATE_ACTIVE, ctx.state);

    cc_init(&ctx);
    go_active(180.0f);
    TEST_ASSERT_EQUAL_INT(CC_STATE_ACTIVE, ctx.state);
}

void test_REQ02_set_with_brake_pressed_does_not_activate(void)
{
    in.main_switch = 1U;
    in.speed_kmh = 80.0f;
    cc_step(&ctx, &in, &out, DT);
    in.set_pressed = 1U;
    in.brake_pressed = 1U;
    cc_step(&ctx, &in, &out, DT);
    TEST_ASSERT_EQUAL_INT(CC_STATE_STANDBY, ctx.state);
}

void test_REQ03_brake_cancels_active(void)
{
    go_active(80.0f);
    in.brake_pressed = 1U;
    cc_step(&ctx, &in, &out, DT);
    TEST_ASSERT_EQUAL_INT(CC_STATE_OFF, ctx.state);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, out.throttle_pct);
}

void test_REQ04_throttle_is_limited_to_100(void)
{
    go_active(100.0f);                 /* target = 100 */
    in.speed_kmh = 31.0f;              /* large positive error */
    cc_step(&ctx, &in, &out, DT);
    TEST_ASSERT_EQUAL_FLOAT(100.0f, out.throttle_pct);
}

void test_REQ04_throttle_is_limited_to_0(void)
{
    go_active(50.0f);                  /* target = 50 */
    in.speed_kmh = 179.0f;             /* large negative error */
    cc_step(&ctx, &in, &out, DT);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, out.throttle_pct);
}

void test_REQ04_throttle_is_zero_when_not_active(void)
{
    in.main_switch = 1U;
    in.speed_kmh = 80.0f;
    cc_step(&ctx, &in, &out, DT);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, out.throttle_pct);
}

void test_REQ05_invalid_speed_in_off_goes_to_fault(void)
{
    in.speed_valid = 0U;
    cc_step(&ctx, &in, &out, DT);
    TEST_ASSERT_EQUAL_INT(CC_STATE_FAULT, ctx.state);
}

void test_REQ05_invalid_speed_in_active_goes_to_fault(void)
{
    go_active(80.0f);
    in.speed_valid = 0U;
    cc_step(&ctx, &in, &out, DT);
    TEST_ASSERT_EQUAL_INT(CC_STATE_FAULT, ctx.state);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, out.throttle_pct);
}

void test_fault_stays_until_main_switch_is_turned_off(void)
{
    in.speed_valid = 0U;
    cc_step(&ctx, &in, &out, DT);      /* -> FAULT */

    in.speed_valid = 1U;
    in.main_switch = 1U;
    cc_step(&ctx, &in, &out, DT);
    TEST_ASSERT_EQUAL_INT(CC_STATE_FAULT, ctx.state);

    in.main_switch = 0U;
    cc_step(&ctx, &in, &out, DT);
    TEST_ASSERT_EQUAL_INT(CC_STATE_OFF, ctx.state);
}

void test_REQ06_cancel_switches_active_to_off(void)
{
    go_active(80.0f);
    in.cancel_pressed = 1U;
    cc_step(&ctx, &in, &out, DT);
    TEST_ASSERT_EQUAL_INT(CC_STATE_OFF, ctx.state);
}

void test_REQ07_speed_below_30_leaves_active(void)
{
    go_active(35.0f);
    in.speed_kmh = 29.0f;
    cc_step(&ctx, &in, &out, DT);
    TEST_ASSERT_EQUAL_INT(CC_STATE_STANDBY, ctx.state);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, out.throttle_pct);
}

void test_speed_above_180_leaves_active(void)
{
    go_active(175.0f);
    in.speed_kmh = 181.0f;
    cc_step(&ctx, &in, &out, DT);
    TEST_ASSERT_EQUAL_INT(CC_STATE_STANDBY, ctx.state);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_REQ01_starts_in_off);
    RUN_TEST(test_main_switch_moves_off_to_standby);
    RUN_TEST(test_REQ02_set_at_50_kmh_activates);
    RUN_TEST(test_REQ02_set_below_30_kmh_stays_standby);
    RUN_TEST(test_REQ02_set_above_180_kmh_stays_standby);
    RUN_TEST(test_REQ02_boundaries_30_and_180_are_accepted);
    RUN_TEST(test_REQ02_set_with_brake_pressed_does_not_activate);
    RUN_TEST(test_REQ03_brake_cancels_active);
    RUN_TEST(test_REQ04_throttle_is_limited_to_100);
    RUN_TEST(test_REQ04_throttle_is_limited_to_0);
    RUN_TEST(test_REQ04_throttle_is_zero_when_not_active);
    RUN_TEST(test_REQ05_invalid_speed_in_off_goes_to_fault);
    RUN_TEST(test_REQ05_invalid_speed_in_active_goes_to_fault);
    RUN_TEST(test_fault_stays_until_main_switch_is_turned_off);
    RUN_TEST(test_REQ06_cancel_switches_active_to_off);
    RUN_TEST(test_REQ07_speed_below_30_leaves_active);
    RUN_TEST(test_speed_above_180_leaves_active);
    return UNITY_END();
}
