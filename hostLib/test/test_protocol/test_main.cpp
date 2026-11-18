// Native tests for the pure codecs and firmware-version rules.

#include <string.h>
#include <unity.h>

#include "bootloader_protocol.h"
#include "fader_buddy_protocol.h"

using namespace faderbuddy;

void setUp() {}
void tearDown() {}

static void test_decode_state_fields() {
  uint32_t raw = ((uint32_t) 1 << STATE_TOUCH_bp) | ((uint32_t) MODE_INPUT_ACTIVE << STATE_MODE_bp) |
                 ((uint32_t) 5 << STATE_ACTIVE_LAYER_bp) | ((uint32_t) 200 << STATE_POSITION_bp) |
                 ((uint32_t) 3 << STATE_POSITION_NONCE_bp) | ((uint32_t) 1234 << STATE_RAW_ADC_bp) |
                 ((uint32_t) 2 << STATE_DOUBLE_TAP_NONCE_bp);
  FaderState s = decode_state(raw);
  TEST_ASSERT_EQUAL_UINT32(raw, s.raw);
  TEST_ASSERT_TRUE(s.touch);
  TEST_ASSERT_EQUAL(MODE_INPUT_ACTIVE, s.mode);
  TEST_ASSERT_EQUAL_UINT8(5, s.active_layer);
  TEST_ASSERT_EQUAL_UINT8(200, s.position);
  TEST_ASSERT_EQUAL_UINT8(3, s.position_nonce);
  TEST_ASSERT_EQUAL_UINT16(1234, s.raw_adc);
  TEST_ASSERT_EQUAL_UINT8(2, s.double_tap_nonce);
}

static void test_decode_state_extremes_do_not_bleed() {
  FaderState s = decode_state((uint32_t) 255 << STATE_POSITION_bp);
  TEST_ASSERT_EQUAL_UINT8(255, s.position);
  TEST_ASSERT_EQUAL_UINT8(0, s.position_nonce);
  TEST_ASSERT_EQUAL_UINT8(0, s.active_layer);
  TEST_ASSERT_FALSE(s.touch);
}

static void test_decode_motor_cal() {
  uint8_t b[12] = {1, 40, 42, 9, 8, 0x01, 0x2C, 0x01, 0x40, 0x00, 0x96, 6};
  MotorCal c = decode_motor_cal(b);
  TEST_ASSERT_TRUE(c.valid);
  TEST_ASSERT_EQUAL_UINT8(40, c.breakaway_rising);
  TEST_ASSERT_EQUAL_UINT8(42, c.breakaway_falling);
  TEST_ASSERT_EQUAL_UINT8(9, c.k_rising);
  TEST_ASSERT_EQUAL_UINT8(8, c.k_falling);
  TEST_ASSERT_EQUAL_UINT16(300, c.v_jump_rising);
  TEST_ASSERT_EQUAL_UINT16(320, c.v_jump_falling);
  TEST_ASSERT_EQUAL_UINT16(150, c.vel_min);
  TEST_ASSERT_EQUAL_UINT8(6, c.deadband);
  b[0] = 0;
  TEST_ASSERT_FALSE(decode_motor_cal(b).valid);
}

static void test_haptic_config_matches_previous_encoding() {
  // The encoding the ESPHome component used before the library existed.
  uint8_t mode = HAPTIC_DETENTS, count = 11, strength = 5;
  uint16_t expected = 0;
  expected |= (mode << HAPTIC_MODE_bp) & HAPTIC_MODE_bm;
  expected |= (count << HAPTIC_DETENT_COUNT_bp) & HAPTIC_DETENT_COUNT_bm;
  expected |= (strength << HAPTIC_DETENT_STRENGTH_bp) & HAPTIC_DETENT_STRENGTH_bm;
  TEST_ASSERT_EQUAL_HEX16(expected, encode_haptic_config(mode, count, strength));
  // Out-of-range fields are masked, not spilled into neighbours.
  TEST_ASSERT_EQUAL_HEX16(HAPTIC_MODE_bm, encode_haptic_config(0xFF, 0, 0));
}

static void test_fw_version_rules() {
  // FW_VERSION_NONE compares above every real version; it must never count.
  TEST_ASSERT_FALSE(fw_supports_move_speed(FW_VERSION_NONE));
  TEST_ASSERT_FALSE(fw_supports_motor_cal(FW_VERSION_NONE));
  TEST_ASSERT_FALSE(fw_supports_bootloader_entry(FW_VERSION_NONE));
  TEST_ASSERT_FALSE(fw_supports_bootloader_entry(0));
  TEST_ASSERT_EQUAL_HEX16(FW_VERSION_NONE, normalize_fw_version(0));
  TEST_ASSERT_EQUAL_HEX16(0x0103, normalize_fw_version(0x0103));

  TEST_ASSERT_FALSE(fw_supports_move_speed(0x0100));
  TEST_ASSERT_TRUE(fw_supports_move_speed(0x0101));
  TEST_ASSERT_FALSE(fw_supports_motor_cal(0x0101));
  TEST_ASSERT_TRUE(fw_supports_motor_cal(0x0102));
  TEST_ASSERT_FALSE(fw_supports_bootloader_entry(0x0102));
  TEST_ASSERT_TRUE(fw_supports_bootloader_entry(0x0103));
  TEST_ASSERT_TRUE(fw_supports_bootloader_entry(0x0200));
}

static void test_format_fw_version() {
  char buf[8];
  format_fw_version(0x0105, buf, sizeof(buf));
  TEST_ASSERT_EQUAL_STRING("1.5", buf);
  format_fw_version(0x020A, buf, sizeof(buf));
  TEST_ASSERT_EQUAL_STRING("2.10", buf);
  format_fw_version(FW_VERSION_NONE, buf, sizeof(buf));
  TEST_ASSERT_EQUAL_STRING("1.0", buf);
}

static void test_image_crc16() {
  // CRC-16/CCITT-FALSE check value.
  const char *check = "123456789";
  TEST_ASSERT_EQUAL_HEX16(0x29B1, image_crc16((const uint8_t *) check, strlen(check)));
  TEST_ASSERT_EQUAL_HEX16(BL_CRC16_INIT, image_crc16(nullptr, 0));
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_decode_state_fields);
  RUN_TEST(test_decode_state_extremes_do_not_bleed);
  RUN_TEST(test_decode_motor_cal);
  RUN_TEST(test_haptic_config_matches_previous_encoding);
  RUN_TEST(test_fw_version_rules);
  RUN_TEST(test_format_fw_version);
  RUN_TEST(test_image_crc16);
  return UNITY_END();
}
