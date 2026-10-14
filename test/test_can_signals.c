#include "unity/unity.h"
#include "../src/can_signals.h"

static uint8_t frame[CAN_FRAME_LEN];

void setUp(void)
{
    uint8_t i;
    for (i = 0U; i < CAN_FRAME_LEN; i++) {
        frame[i] = 0U;
    }
}

void tearDown(void) {}

/* ---------- raw pack / unpack ---------- */

void test_CAN_intel_16bit_byte_aligned(void)
{
    const uint8_t expected[CAN_FRAME_LEN] = {0x34, 0x12, 0, 0, 0, 0, 0, 0};
    TEST_ASSERT_EQUAL_UINT8(1U, can_pack_raw(frame, 0U, 16U, CAN_INTEL, 0x1234U));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, frame, CAN_FRAME_LEN);
}

void test_CAN_intel_8bit_not_byte_aligned(void)
{
    /* bits 4..11: low nibble of 0xAB goes to byte0 high nibble */
    const uint8_t expected[CAN_FRAME_LEN] = {0xB0, 0x0A, 0, 0, 0, 0, 0, 0};
    TEST_ASSERT_EQUAL_UINT8(1U, can_pack_raw(frame, 4U, 8U, CAN_INTEL, 0xABU));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, frame, CAN_FRAME_LEN);
}

void test_CAN_motorola_16bit_byte_aligned(void)
{
    const uint8_t expected[CAN_FRAME_LEN] = {0x12, 0x34, 0, 0, 0, 0, 0, 0};
    TEST_ASSERT_EQUAL_UINT8(1U, can_pack_raw(frame, 7U, 16U, CAN_MOTOROLA, 0x1234U));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, frame, CAN_FRAME_LEN);
}

void test_CAN_motorola_8bit_crossing_a_byte(void)
{
    /* MSB at bit 3: upper nibble to byte0 bits 3..0, lower nibble to byte1 bits 7..4 */
    const uint8_t expected[CAN_FRAME_LEN] = {0x0A, 0xB0, 0, 0, 0, 0, 0, 0};
    TEST_ASSERT_EQUAL_UINT8(1U, can_pack_raw(frame, 3U, 8U, CAN_MOTOROLA, 0xABU));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, frame, CAN_FRAME_LEN);
}

void test_CAN_pack_does_not_touch_neighbour_bits(void)
{
    uint8_t i;
    for (i = 0U; i < CAN_FRAME_LEN; i++) {
        frame[i] = 0xFFU;
    }
    TEST_ASSERT_EQUAL_UINT8(1U, can_pack_raw(frame, 4U, 4U, CAN_INTEL, 0U));
    TEST_ASSERT_EQUAL_HEX8(0x0F, frame[0]);
    TEST_ASSERT_EQUAL_HEX8(0xFF, frame[1]);
}

void test_CAN_pack_unpack_roundtrip_intel_and_motorola(void)
{
    const uint32_t values[] = {0U, 1U, 0x55U, 0xAAU, 0x1234U, 0xFFFFU};
    uint8_t v;
    uint32_t raw = 0U;

    for (v = 0U; v < (uint8_t)(sizeof(values) / sizeof(values[0])); v++) {
        TEST_ASSERT_EQUAL_UINT8(1U, can_pack_raw(frame, 8U, 16U, CAN_INTEL, values[v]));
        TEST_ASSERT_EQUAL_UINT8(1U, can_unpack_raw(frame, 8U, 16U, CAN_INTEL, &raw));
        TEST_ASSERT_EQUAL_UINT32(values[v], raw);

        TEST_ASSERT_EQUAL_UINT8(1U, can_pack_raw(frame, 23U, 16U, CAN_MOTOROLA, values[v]));
        TEST_ASSERT_EQUAL_UINT8(1U, can_unpack_raw(frame, 23U, 16U, CAN_MOTOROLA, &raw));
        TEST_ASSERT_EQUAL_UINT32(values[v], raw);
    }
}

void test_CAN_unpack_ignores_other_bits(void)
{
    uint32_t raw = 0U;
    frame[0] = 0xFFU;
    frame[1] = 0xFFU;
    frame[2] = 0xFFU;
    TEST_ASSERT_EQUAL_UINT8(1U, can_unpack_raw(frame, 4U, 8U, CAN_INTEL, &raw));
    TEST_ASSERT_EQUAL_UINT32(0xFFU, raw);
    frame[0] = 0x0FU;
    frame[1] = 0xF0U;
    TEST_ASSERT_EQUAL_UINT8(1U, can_unpack_raw(frame, 4U, 8U, CAN_INTEL, &raw));
    TEST_ASSERT_EQUAL_UINT32(0U, raw);
}

void test_CAN_invalid_layout_is_rejected_and_frame_unchanged(void)
{
    uint32_t raw = 77U;
    const uint8_t zero[CAN_FRAME_LEN] = {0, 0, 0, 0, 0, 0, 0, 0};

    TEST_ASSERT_EQUAL_UINT8(0U, can_pack_raw(frame, 0U, 0U, CAN_INTEL, 1U));      /* length 0       */
    TEST_ASSERT_EQUAL_UINT8(0U, can_pack_raw(frame, 0U, 33U, CAN_INTEL, 1U));     /* length > 32    */
    TEST_ASSERT_EQUAL_UINT8(0U, can_pack_raw(frame, 64U, 1U, CAN_INTEL, 1U));     /* start > 63     */
    TEST_ASSERT_EQUAL_UINT8(0U, can_pack_raw(frame, 60U, 8U, CAN_INTEL, 1U));     /* runs past end  */
    TEST_ASSERT_EQUAL_UINT8(0U, can_pack_raw(frame, 56U, 16U, CAN_MOTOROLA, 1U)); /* runs past end  */
    TEST_ASSERT_EQUAL_UINT8_ARRAY(zero, frame, CAN_FRAME_LEN);

    TEST_ASSERT_EQUAL_UINT8(0U, can_unpack_raw(frame, 60U, 8U, CAN_INTEL, &raw));
    TEST_ASSERT_EQUAL_UINT32(77U, raw);
}

/* ---------- physical signals ---------- */

void test_CAN_known_frame_is_encoded_byte_by_byte(void)
{
    /* 100.00 km/h -> raw 10000 = 0x2710, brake = 1, throttle 50 % -> raw 100 = 0x64 */
    const uint8_t expected[CAN_FRAME_LEN] = {0x10, 0x27, 0x01, 0x64, 0, 0, 0, 0};

    TEST_ASSERT_EQUAL_UINT8(1U, can_encode(frame, &CAN_SIG_VEHICLE_SPEED, 100.0f));
    TEST_ASSERT_EQUAL_UINT8(1U, can_encode(frame, &CAN_SIG_BRAKE_PEDAL, 1.0f));
    TEST_ASSERT_EQUAL_UINT8(1U, can_encode(frame, &CAN_SIG_THROTTLE_REQUEST, 50.0f));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, frame, CAN_FRAME_LEN);
}

void test_CAN_known_frame_is_decoded(void)
{
    float speed = 0.0f;
    float brake = 0.0f;
    float throttle = 0.0f;

    frame[0] = 0x10U;
    frame[1] = 0x27U;
    frame[2] = 0x01U;
    frame[3] = 0x64U;

    TEST_ASSERT_EQUAL_UINT8(1U, can_decode(frame, &CAN_SIG_VEHICLE_SPEED, &speed));
    TEST_ASSERT_EQUAL_UINT8(1U, can_decode(frame, &CAN_SIG_BRAKE_PEDAL, &brake));
    TEST_ASSERT_EQUAL_UINT8(1U, can_decode(frame, &CAN_SIG_THROTTLE_REQUEST, &throttle));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 100.0f, speed);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.0f, brake);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 50.0f, throttle);
}

void test_CAN_speed_roundtrip_within_resolution(void)
{
    float speed = 0.0f;
    TEST_ASSERT_EQUAL_UINT8(1U, can_encode(frame, &CAN_SIG_VEHICLE_SPEED, 123.45f));
    TEST_ASSERT_EQUAL_UINT8(1U, can_decode(frame, &CAN_SIG_VEHICLE_SPEED, &speed));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 123.45f, speed);
}

void test_CAN_throttle_resolution_is_half_percent(void)
{
    float throttle = 0.0f;
    /* 33.3 % / 0.5 = 66.6 -> rounds to raw 67 -> 33.5 % */
    TEST_ASSERT_EQUAL_UINT8(1U, can_encode(frame, &CAN_SIG_THROTTLE_REQUEST, 33.3f));
    TEST_ASSERT_EQUAL_HEX8(67U, frame[3]);
    TEST_ASSERT_EQUAL_UINT8(1U, can_decode(frame, &CAN_SIG_THROTTLE_REQUEST, &throttle));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 33.5f, throttle);
}

void test_CAN_boundary_values_are_limited_to_raw_range(void)
{
    /* below range -> 0 */
    TEST_ASSERT_EQUAL_UINT8(1U, can_encode(frame, &CAN_SIG_VEHICLE_SPEED, -5.0f));
    TEST_ASSERT_EQUAL_HEX8(0x00, frame[0]);
    TEST_ASSERT_EQUAL_HEX8(0x00, frame[1]);

    /* above range -> 0xFFFF (655.35 km/h) */
    TEST_ASSERT_EQUAL_UINT8(1U, can_encode(frame, &CAN_SIG_VEHICLE_SPEED, 1000.0f));
    TEST_ASSERT_EQUAL_HEX8(0xFF, frame[0]);
    TEST_ASSERT_EQUAL_HEX8(0xFF, frame[1]);

    /* throttle above 127.5 % -> raw 255 */
    TEST_ASSERT_EQUAL_UINT8(1U, can_encode(frame, &CAN_SIG_THROTTLE_REQUEST, 500.0f));
    TEST_ASSERT_EQUAL_HEX8(0xFF, frame[3]);
}

void test_CAN_encode_leaves_other_signals_untouched(void)
{
    TEST_ASSERT_EQUAL_UINT8(1U, can_encode(frame, &CAN_SIG_THROTTLE_REQUEST, 50.0f));
    TEST_ASSERT_EQUAL_UINT8(1U, can_encode(frame, &CAN_SIG_VEHICLE_SPEED, 100.0f));
    TEST_ASSERT_EQUAL_HEX8(0x64, frame[3]);
}

void test_CAN_encode_rejects_invalid_scaling(void)
{
    const CanSignal bad = { 0U, 8U, CAN_INTEL, 0.0f, 0.0f };   /* factor 0 */
    TEST_ASSERT_EQUAL_UINT8(0U, can_encode(frame, &bad, 10.0f));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_CAN_intel_16bit_byte_aligned);
    RUN_TEST(test_CAN_intel_8bit_not_byte_aligned);
    RUN_TEST(test_CAN_motorola_16bit_byte_aligned);
    RUN_TEST(test_CAN_motorola_8bit_crossing_a_byte);
    RUN_TEST(test_CAN_pack_does_not_touch_neighbour_bits);
    RUN_TEST(test_CAN_pack_unpack_roundtrip_intel_and_motorola);
    RUN_TEST(test_CAN_unpack_ignores_other_bits);
    RUN_TEST(test_CAN_invalid_layout_is_rejected_and_frame_unchanged);
    RUN_TEST(test_CAN_known_frame_is_encoded_byte_by_byte);
    RUN_TEST(test_CAN_known_frame_is_decoded);
    RUN_TEST(test_CAN_speed_roundtrip_within_resolution);
    RUN_TEST(test_CAN_throttle_resolution_is_half_percent);
    RUN_TEST(test_CAN_boundary_values_are_limited_to_raw_range);
    RUN_TEST(test_CAN_encode_leaves_other_signals_untouched);
    RUN_TEST(test_CAN_encode_rejects_invalid_scaling);
    return UNITY_END();
}
