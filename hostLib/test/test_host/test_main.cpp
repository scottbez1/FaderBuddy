// Native tests for the high-level FaderBuddy against the simulated fader.

#include <unity.h>

#include <string>
#include <vector>

#include "../fake_fader.h"
#include "fader_buddy_host.h"

using namespace faderbuddy;
using namespace fbtest;

struct Events : FaderBuddyListener {
  std::vector<std::pair<uint8_t, uint8_t>> manual_moves, raw_moves;
  std::vector<std::pair<bool, uint8_t>> touches;
  std::vector<uint8_t> double_taps;
  std::vector<std::string> stages;
  std::vector<uint8_t> progress;
  std::vector<std::pair<bool, std::string>> results;
  std::string serial;
  int status_changes = 0;

  void on_manual_move(uint8_t p, uint8_t l) override { manual_moves.push_back({p, l}); }
  void on_raw_position_update(uint8_t p, uint8_t l) override { raw_moves.push_back({p, l}); }
  void on_touch_change(bool t, uint8_t l) override { touches.push_back({t, l}); }
  void on_double_tap(uint8_t l) override { double_taps.push_back(l); }
  void on_serial_number(const char *s) override { serial = s; }
  void on_status_changed() override { status_changes++; }
  void on_firmware_update_stage(const char *label) override { stages.push_back(label); }
  void on_firmware_update_progress(const char *, uint8_t pct) override { progress.push_back(pct); }
  void on_firmware_update_result(bool ok, const char *err) override { results.push_back({ok, err}); }
};

static FakePlatform *platform;
static FakeFader *fader;
static FaderBuddy *fb;
static Events *events;

void setUp() {
  platform = new FakePlatform();
  fader = new FakeFader(*platform);
  fb = new FaderBuddy(*fader, *platform);
  events = new Events();
  fb->set_listener(events);
}

void tearDown() {
  delete fb;
  delete events;
  delete fader;
  delete platform;
}

static void run_loop_until_idle() {
  for (int i = 0; i < 100000 && fb->firmware_update_in_progress(); i++) {
    fb->loop();
  }
  TEST_ASSERT_FALSE_MESSAGE(fb->firmware_update_in_progress(), "update never finished");
}

static void test_begin_reads_identity_and_resets_layer() {
  fader->active_layer = 3;
  fb->begin();
  TEST_ASSERT_EQUAL_STRING("A0A1A2A3A4A5A6A7A8A9", fb->get_serial_number());
  TEST_ASSERT_EQUAL_STRING("A0A1A2A3A4A5A6A7A8A9", events->serial.c_str());
  TEST_ASSERT_EQUAL_HEX16(0x0105, fb->get_firmware_version());
  TEST_ASSERT_TRUE(fb->speed_supported());
  TEST_ASSERT_TRUE(fb->bootloader_entry_supported());
  TEST_ASSERT_EQUAL_UINT8(0, fader->active_layer);
  // motor_cal byte 0 is 0: never characterised.
  TEST_ASSERT_EQUAL(FaderBuddy::CALIBRATION_NEEDED, fb->get_calibration_status());
}

static void test_begin_sends_initial_haptic_configs() {
  fb->store_initial_layer_haptic_config(2, HAPTIC_DETENTS, 5, 3);
  fb->begin();
  auto w = fader->last_app_write(REG_LAYER_HAPTIC_CONFIG);
  TEST_ASSERT_EQUAL(4, (int) w.size());
  TEST_ASSERT_EQUAL_UINT8(2, w[1]);
  uint16_t cfg = encode_haptic_config(HAPTIC_DETENTS, 5, 3);
  TEST_ASSERT_EQUAL_UINT8(cfg >> 8, w[2]);
  TEST_ASSERT_EQUAL_UINT8(cfg & 0xFF, w[3]);
}

static void test_move_sends_speed_byte_on_new_firmware() {
  fb->begin();
  fb->remote_move_to(100, 1);  // layer default: full speed
  auto w = fader->last_app_write(REG_LAYER_TARGET);
  TEST_ASSERT_EQUAL(4, (int) w.size());
  TEST_ASSERT_EQUAL_UINT8(LAYER_SPEED_FULL, w[3]);

  fb->set_layer_default_speed(1, 40);
  fb->remote_move_to(100, 1);
  w = fader->last_app_write(REG_LAYER_TARGET);
  TEST_ASSERT_EQUAL_UINT8(40, w[3]);

  fb->remote_move_to(100, 1, 7);
  TEST_ASSERT_EQUAL_UINT8(7, fader->last_app_write(REG_LAYER_TARGET)[3]);
}

static void test_move_falls_back_to_three_bytes_on_old_firmware() {
  fader->fw_version = FW_VERSION_NONE;
  fb->begin();
  TEST_ASSERT_FALSE(fb->speed_supported());
  fb->remote_move_to(100, 0, 7);
  auto w = fader->last_app_write(REG_LAYER_TARGET);
  TEST_ASSERT_EQUAL(3, (int) w.size());
  TEST_ASSERT_TRUE(platform->logged("does not support it"));
}

static void test_invert_applies_both_ways() {
  fb->set_invert(true);
  fb->begin();
  fb->remote_move_to(10, 0);
  TEST_ASSERT_EQUAL_UINT8(245, fader->last_app_write(REG_LAYER_TARGET)[2]);
  TEST_ASSERT_EQUAL_UINT8(10, fb->get_position(0));

  fader->set_state(MODE_INPUT_ACTIVE, true, 0, 200, 1);
  fb->poll();
  TEST_ASSERT_EQUAL_UINT8(55, events->raw_moves.back().first);
}

static void test_events_from_state_changes() {
  fb->begin();
  fader->set_state(MODE_INPUT_ACTIVE, true, 2, 120, 1);
  fb->poll();
  TEST_ASSERT_EQUAL(1, (int) events->touches.size());
  TEST_ASSERT_TRUE(events->touches[0].first);
  TEST_ASSERT_EQUAL_UINT8(2, events->touches[0].second);
  TEST_ASSERT_EQUAL(1, (int) events->manual_moves.size());
  TEST_ASSERT_EQUAL_UINT8(120, events->manual_moves[0].first);
  TEST_ASSERT_EQUAL_UINT8(2, events->manual_moves[0].second);

  // A remote move reports a raw update but not a manual move.
  fader->set_state(MODE_REMOTE_MOVEMENT_IN_PROGRESS, false, 2, 130, 1);
  fb->poll();
  TEST_ASSERT_EQUAL(1, (int) events->manual_moves.size());
  TEST_ASSERT_EQUAL_UINT8(130, events->raw_moves.back().first);

  fader->set_state(MODE_INPUT_IDLE, false, 2, 130, 1, 1);
  fb->poll();
  TEST_ASSERT_EQUAL(1, (int) events->double_taps.size());
  TEST_ASSERT_EQUAL_UINT8(2, events->double_taps[0]);
}

static void test_manual_moves_are_rate_limited_and_the_last_one_delivered() {
  fb->set_layer_value_change_min_interval(0, 500);
  fb->begin();
  fader->set_state(MODE_INPUT_ACTIVE, true, 0, 10, 1);
  fb->poll();
  TEST_ASSERT_EQUAL(1, (int) events->manual_moves.size());
  fader->set_state(MODE_INPUT_ACTIVE, true, 0, 20, 1);
  fb->poll();
  fader->set_state(MODE_INPUT_ACTIVE, true, 0, 30, 1);
  fb->poll();
  TEST_ASSERT_EQUAL(1, (int) events->manual_moves.size());  // throttled
  platform->now += 600;
  fb->poll();
  TEST_ASSERT_EQUAL(2, (int) events->manual_moves.size());
  TEST_ASSERT_EQUAL_UINT8(30, events->manual_moves[1].first);
}

static void test_calibration_status_follows_mode() {
  fb->begin();
  fader->set_state(MODE_SELF_CALIBRATION, false, 0, 0);
  fb->poll();
  TEST_ASSERT_EQUAL(FaderBuddy::CALIBRATION_RUNNING, fb->get_calibration_status());
  fader->motor_cal[0] = 1;
  fader->set_state(MODE_INPUT_IDLE, false, 0, 0);
  fb->poll();
  TEST_ASSERT_EQUAL(FaderBuddy::CALIBRATION_SUCCEEDED, fb->get_calibration_status());

  fader->set_state(MODE_SELF_CALIBRATION, false, 0, 0);
  fb->poll();
  fader->set_state(MODE_ERROR, false, 0, 0);
  fb->poll();
  TEST_ASSERT_EQUAL(FaderBuddy::CALIBRATION_FAILED, fb->get_calibration_status());
}

static void test_absent_fader_is_reprobed_on_backoff_and_initializes() {
  fader->mode = FakeFader::ABSENT;
  fb->begin();
  TEST_ASSERT_TRUE(fb->is_awaiting_device());
  int t = fader->transactions;
  fb->poll();  // before the first backoff expires: no traffic
  TEST_ASSERT_EQUAL(t, fader->transactions);
  platform->now += 1100;
  fb->poll();
  TEST_ASSERT_EQUAL(t + 1, fader->transactions);
  TEST_ASSERT_TRUE(fb->is_awaiting_device());

  fader->mode = FakeFader::APP;
  platform->now += 2100;  // backoff doubled to 2s
  fb->poll();
  TEST_ASSERT_FALSE(fb->is_awaiting_device());
  TEST_ASSERT_EQUAL_HEX16(0x0105, fb->get_firmware_version());
}

static void test_incompatible_protocol() {
  fader->protocol_version = 4;
  fb->begin();
  TEST_ASSERT_TRUE(fb->is_incompatible());
  auto img = make_image(0x0105);
  fb->set_firmware_image(img.data(), img.size(), image_crc16(img.data(), img.size()), 0x0105);
  fb->update_firmware();
  TEST_ASSERT_EQUAL(1, (int) events->results.size());
  TEST_ASSERT_FALSE(events->results[0].first);
  TEST_ASSERT_EQUAL_STRING("UPDI reflash required", events->results[0].second.c_str());
}

static void test_update_firmware_end_to_end() {
  fader->fw_version = 0x0104;
  auto img = make_image(0x0105);
  fb->set_firmware_image(img.data(), img.size(), image_crc16(img.data(), img.size()), 0x0105);
  fb->begin();
  TEST_ASSERT_TRUE(fb->firmware_update_offered());
  TEST_ASSERT_TRUE(fb->firmware_update_available());
  fb->start_firmware_update();
  TEST_ASSERT_TRUE(fb->firmware_update_in_progress());
  run_loop_until_idle();
  TEST_ASSERT_EQUAL(1, (int) events->results.size());
  TEST_ASSERT_TRUE_MESSAGE(events->results[0].first, events->results[0].second.c_str());
  TEST_ASSERT_EQUAL_HEX16(0x0105, fb->get_firmware_version());
  TEST_ASSERT_FALSE(fb->firmware_update_available());
  // Stage labels, in the order the ESPHome component published them.
  std::vector<std::string> expected = {"waiting for fader", "entering bootloader", "erasing", "verifying",
                                       "verifying", "starting app"};
  TEST_ASSERT_EQUAL(expected.size(), events->stages.size());
  for (size_t i = 0; i < expected.size(); i++) {
    TEST_ASSERT_EQUAL_STRING(expected[i].c_str(), events->stages[i].c_str());
  }
  TEST_ASSERT_EQUAL_UINT8(100, events->progress.back());
  for (size_t i = 1; i < events->progress.size(); i++) {
    TEST_ASSERT_TRUE(events->progress[i] > events->progress[i - 1]);
  }
}

static void test_update_recovers_fader_found_in_bootloader() {
  fader->mode = FakeFader::BOOTLOADER;
  auto img = make_image(0x0105);
  fb->set_firmware_image(img.data(), img.size(), image_crc16(img.data(), img.size()), 0x0105);
  fb->begin();
  TEST_ASSERT_TRUE(fb->is_bootloader_resident());
  TEST_ASSERT_EQUAL_STRING("", fb->get_serial_number());
  TEST_ASSERT_TRUE(fb->firmware_update_available());
  fb->update_firmware();
  run_loop_until_idle();
  TEST_ASSERT_TRUE(events->results.back().first);
  TEST_ASSERT_FALSE(fb->is_bootloader_resident());
  // Read now that there is an app.
  TEST_ASSERT_EQUAL_STRING("A0A1A2A3A4A5A6A7A8A9", fb->get_serial_number());
}

static void test_update_when_current_is_a_noop_success() {
  auto img = make_image(0x0105);
  fb->set_firmware_image(img.data(), img.size(), image_crc16(img.data(), img.size()), 0x0105);
  fb->begin();
  // start_firmware_update() rejects it from cached state, with no result...
  fb->start_firmware_update();
  TEST_ASSERT_FALSE(fb->firmware_update_in_progress());
  TEST_ASSERT_EQUAL(0, (int) events->results.size());
  // ...update_firmware() asks the fader, and reports success.
  fb->update_firmware();
  run_loop_until_idle();
  TEST_ASSERT_EQUAL(1, (int) events->results.size());
  TEST_ASSERT_TRUE(events->results[0].first);
  TEST_ASSERT_EQUAL(0, fader->erase_count);
}

static void test_update_waits_for_touch_then_gives_up() {
  fader->fw_version = 0x0104;
  auto img = make_image(0x0105);
  fb->set_firmware_image(img.data(), img.size(), image_crc16(img.data(), img.size()), 0x0105);
  fb->begin();
  fader->set_state(MODE_INPUT_ACTIVE, true, 0, 50);
  fb->update_firmware();
  run_loop_until_idle();
  TEST_ASSERT_EQUAL(1, (int) events->results.size());
  TEST_ASSERT_FALSE(events->results[0].first);
  TEST_ASSERT_EQUAL_STRING("fader in use", events->results[0].second.c_str());
  TEST_ASSERT_EQUAL(0, fader->erase_count);
  TEST_ASSERT_EQUAL_HEX16(0x0104, fb->get_firmware_version());
}

static void test_update_rejected_for_old_firmware() {
  fader->fw_version = 0x0102;
  auto img = make_image(0x0105);
  fb->set_firmware_image(img.data(), img.size(), image_crc16(img.data(), img.size()), 0x0105);
  fb->begin();
  TEST_ASSERT_TRUE(fb->firmware_update_offered());
  TEST_ASSERT_FALSE(fb->firmware_update_possible());
  fb->start_firmware_update();
  TEST_ASSERT_EQUAL_STRING("UPDI reflash required", events->results.back().second.c_str());
  // forced: goes to the wire, which refuses too.
  fb->start_firmware_update(true);
  run_loop_until_idle();
  TEST_ASSERT_EQUAL_STRING("firmware predates bootloader entry", events->results.back().second.c_str());
}

static void test_failed_update_reprobes_and_reports_bootloader() {
  fader->mode = FakeFader::BOOTLOADER;
  fader->corrupt_page = true;
  fader->corrupt_page_addr = BL_APP_START + 3 * BL_PAGE_SIZE;
  auto img = make_image(0x0105);
  fb->set_firmware_image(img.data(), img.size(), image_crc16(img.data(), img.size()), 0x0105);
  fb->begin();
  fb->update_firmware();
  run_loop_until_idle();
  TEST_ASSERT_FALSE(events->results.back().first);
  TEST_ASSERT_TRUE(events->results.back().second.find("CRC mismatch") == 0);
  TEST_ASSERT_TRUE(fb->is_bootloader_resident());
  TEST_ASSERT_TRUE(fb->firmware_update_available());
}

static void test_second_request_while_updating_is_rejected() {
  fader->fw_version = 0x0104;
  auto img = make_image(0x0105);
  fb->set_firmware_image(img.data(), img.size(), image_crc16(img.data(), img.size()), 0x0105);
  fb->begin();
  fb->update_firmware();
  fb->update_firmware();
  TEST_ASSERT_EQUAL(1, (int) events->results.size());
  TEST_ASSERT_EQUAL_STRING("update already in progress", events->results[0].second.c_str());
  run_loop_until_idle();
  TEST_ASSERT_TRUE(events->results.back().first);
}

static void test_poll_is_suppressed_during_update() {
  fader->fw_version = 0x0104;
  auto img = make_image(0x0105);
  fb->set_firmware_image(img.data(), img.size(), image_crc16(img.data(), img.size()), 0x0105);
  fb->begin();
  fb->update_firmware();
  int t = fader->transactions;
  fb->poll();
  TEST_ASSERT_EQUAL(t, fader->transactions);
}

static void test_loop_polls_at_interval() {
  fb->begin();
  fb->set_poll_interval(50);
  fader->set_state(MODE_INPUT_ACTIVE, true, 0, 99, 1);
  fb->loop();
  TEST_ASSERT_EQUAL(1, (int) events->touches.size());
  int t = fader->transactions;
  fb->loop();  // well within 50ms: no poll
  TEST_ASSERT_EQUAL(t, fader->transactions);
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_begin_reads_identity_and_resets_layer);
  RUN_TEST(test_begin_sends_initial_haptic_configs);
  RUN_TEST(test_move_sends_speed_byte_on_new_firmware);
  RUN_TEST(test_move_falls_back_to_three_bytes_on_old_firmware);
  RUN_TEST(test_invert_applies_both_ways);
  RUN_TEST(test_events_from_state_changes);
  RUN_TEST(test_manual_moves_are_rate_limited_and_the_last_one_delivered);
  RUN_TEST(test_calibration_status_follows_mode);
  RUN_TEST(test_absent_fader_is_reprobed_on_backoff_and_initializes);
  RUN_TEST(test_incompatible_protocol);
  RUN_TEST(test_update_firmware_end_to_end);
  RUN_TEST(test_update_recovers_fader_found_in_bootloader);
  RUN_TEST(test_update_when_current_is_a_noop_success);
  RUN_TEST(test_update_waits_for_touch_then_gives_up);
  RUN_TEST(test_update_rejected_for_old_firmware);
  RUN_TEST(test_failed_update_reprobes_and_reports_bootloader);
  RUN_TEST(test_second_request_while_updating_is_rejected);
  RUN_TEST(test_poll_is_suppressed_during_update);
  RUN_TEST(test_loop_polls_at_interval);
  return UNITY_END();
}
