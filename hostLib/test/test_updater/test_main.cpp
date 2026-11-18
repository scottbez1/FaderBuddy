// Native tests for FirmwareUpdater against the simulated bootloader.

#include <unity.h>

#include "../fake_fader.h"
#include "fader_buddy_updater.h"

using namespace faderbuddy;
using namespace fbtest;

static FakePlatform *platform;
static FakeFader *fader;
static FaderBuddyDevice *device;

void setUp() {
  platform = new FakePlatform();
  fader = new FakeFader(*platform);
  device = new FaderBuddyDevice(*fader, *platform);
}

void tearDown() {
  delete device;
  delete fader;
  delete platform;
}

struct Recorder : UpdateObserver {
  std::vector<UpdateStage> stages;
  uint32_t last_written = 0, last_total = 0;
  void on_update_stage(UpdateStage s) override { stages.push_back(s); }
  void on_update_pages_written(uint32_t w, uint32_t t) override {
    last_written = w;
    last_total = t;
  }
};

static FirmwareImage image_of(const std::vector<uint8_t> &img) {
  return FirmwareImage{img.data(), (uint32_t) img.size(), image_crc16(img.data(), img.size()),
                       (uint16_t) ((img[img.size() - 2] << 8) | img[img.size() - 1])};
}

// Runs to completion; fails the test if it doesn't finish in a sane number of
// ticks (a stuck state machine).
static void run(FirmwareUpdater &u) {
  for (int i = 0; i < 100000 && u.tick(); i++) {
  }
  TEST_ASSERT_FALSE_MESSAGE(u.running(), "updater never finished");
}

static void assert_flash_matches(const std::vector<uint8_t> &img) {
  TEST_ASSERT_EQUAL_MEMORY(img.data(), fader->flash + BL_APP_START, img.size());
}

static void test_install_from_resident_bootloader() {
  fader->mode = FakeFader::BOOTLOADER;
  auto img = make_image(0x0106);
  Recorder rec;
  FirmwareUpdater u(*device, &rec);
  TEST_ASSERT_TRUE(u.start(image_of(img)));
  run(u);
  TEST_ASSERT_TRUE_MESSAGE(u.succeeded(), u.error_message());
  TEST_ASSERT_FALSE(u.was_noop());
  assert_flash_matches(img);
  TEST_ASSERT_EQUAL(FakeFader::APP, fader->mode);
  TEST_ASSERT_EQUAL_HEX16(0x0106, fader->fw_version);
  TEST_ASSERT_EQUAL_UINT32(BL_APP_SIZE / BL_PAGE_SIZE, u.pages_written());
  TEST_ASSERT_EQUAL_UINT32(u.pages_total(), rec.last_written);
  // No ENTER_BOOTLOADER was needed.
  TEST_ASSERT_EQUAL(0, (int) fader->last_app_write(REG_ENTER_BOOTLOADER).size());
}

static void test_page_zero_is_written_last_and_only_once() {
  fader->mode = FakeFader::BOOTLOADER;
  auto img = make_image(0x0106);
  FirmwareUpdater u(*device);
  u.start(image_of(img));
  run(u);
  TEST_ASSERT_TRUE(u.succeeded());
  TEST_ASSERT_EQUAL_UINT32(BL_APP_SIZE / BL_PAGE_SIZE, fader->page_write_order.size());
  TEST_ASSERT_EQUAL_HEX16(BL_APP_START, fader->page_write_order.back());
  for (size_t i = 0; i + 1 < fader->page_write_order.size(); i++) {
    TEST_ASSERT_NOT_EQUAL(BL_APP_START, fader->page_write_order[i]);
  }
}

static void test_install_from_running_app_enters_bootloader() {
  fader->mode = FakeFader::APP;
  fader->fw_version = 0x0103;
  auto img = make_image(0x0105);
  Recorder rec;
  FirmwareUpdater u(*device, &rec);
  u.start(image_of(img));
  run(u);
  TEST_ASSERT_TRUE_MESSAGE(u.succeeded(), u.error_message());
  TEST_ASSERT_EQUAL(5, (int) fader->last_app_write(REG_ENTER_BOOTLOADER).size());
  TEST_ASSERT_EQUAL_HEX16(0x0105, fader->fw_version);
  // Stage sequence, de-duplicated, as a host would show it.
  std::vector<UpdateStage> expected = {UpdateStage::PROBE,       UpdateStage::WAIT_MARKER,   UpdateStage::ERASE,
                                       UpdateStage::ERASE_WAIT,  UpdateStage::WRITE,         UpdateStage::BODY_CHECK,
                                       UpdateStage::BODY_CRC_WAIT, UpdateStage::WRITE_CHECK, UpdateStage::CRC_WAIT,
                                       UpdateStage::WAIT_APP,    UpdateStage::CONFIRM,       UpdateStage::DONE};
  TEST_ASSERT_EQUAL(expected.size(), rec.stages.size());
  for (size_t i = 0; i < expected.size(); i++) {
    TEST_ASSERT_EQUAL((int) expected[i], (int) rec.stages[i]);
  }
}

static void test_refuses_firmware_predating_bootloader_entry() {
  fader->fw_version = 0x0102;
  auto img = make_image(0x0105);
  FirmwareUpdater u(*device);
  u.start(image_of(img));
  run(u);
  TEST_ASSERT_EQUAL(UpdateError::PREDATES_BOOTLOADER_ENTRY, u.error());
  TEST_ASSERT_EQUAL(0, (int) fader->last_app_write(REG_ENTER_BOOTLOADER).size());
  TEST_ASSERT_EQUAL(0, fader->erase_count);
}

static void test_refuses_unversioned_firmware() {
  fader->fw_version = FW_VERSION_NONE;  // reads back 0xFFFF, numerically "newest"
  auto img = make_image(0x0105);
  FirmwareUpdater u(*device);
  u.start(image_of(img));
  run(u);
  TEST_ASSERT_EQUAL(UpdateError::PREDATES_BOOTLOADER_ENTRY, u.error());
}

// The production jig's "old" app is FW_VERSION 0 but does support entry.
static void test_unversioned_app_can_be_updated_when_gate_disabled() {
  fader->fw_version = 0x0000;
  fader->reject_enter = false;
  auto img = make_image(0x0105);
  FirmwareUpdater::Options opt;
  opt.require_bootloader_entry_version = false;
  FirmwareUpdater u(*device);
  u.start(image_of(img), opt);
  run(u);
  TEST_ASSERT_TRUE_MESSAGE(u.succeeded(), u.error_message());
  TEST_ASSERT_EQUAL_HEX16(0x0105, fader->fw_version);
}

// ...and installing it confirms against a raw version of 0, not "unknown".
static void test_installs_fw_version_zero_image() {
  fader->mode = FakeFader::BOOTLOADER;
  auto img = make_image(0x0000);
  FirmwareUpdater u(*device);
  u.start(image_of(img));
  run(u);
  TEST_ASSERT_TRUE_MESSAGE(u.succeeded(), u.error_message());
}

static void test_skip_if_current_is_a_noop_success() {
  fader->fw_version = 0x0105;
  auto img = make_image(0x0105);
  FirmwareUpdater::Options opt;
  opt.skip_if_current = true;
  FirmwareUpdater u(*device);
  u.start(image_of(img), opt);
  run(u);
  TEST_ASSERT_TRUE(u.succeeded());
  TEST_ASSERT_TRUE(u.was_noop());
  TEST_ASSERT_EQUAL(0, fader->erase_count);
  TEST_ASSERT_EQUAL(FakeFader::APP, fader->mode);
}

static void test_reinstalls_same_version_without_skip() {
  fader->fw_version = 0x0105;
  auto img = make_image(0x0105);
  FirmwareUpdater u(*device);
  u.start(image_of(img));
  run(u);
  TEST_ASSERT_TRUE_MESSAGE(u.succeeded(), u.error_message());
  TEST_ASSERT_FALSE(u.was_noop());
  TEST_ASSERT_EQUAL(1, fader->erase_count);
}

static void test_body_crc_mismatch_leaves_reset_vector_blank() {
  fader->mode = FakeFader::BOOTLOADER;
  fader->corrupt_page = true;
  fader->corrupt_page_addr = BL_APP_START + 5 * BL_PAGE_SIZE;
  auto img = make_image(0x0106);
  FirmwareUpdater u(*device);
  u.start(image_of(img));
  run(u);
  TEST_ASSERT_EQUAL(UpdateError::CRC_MISMATCH, u.error());
  TEST_ASSERT_EQUAL((int) UpdateStage::BODY_CRC_WAIT, (int) u.failed_stage());
  TEST_ASSERT_TRUE(fader->reset_vector_blank());
  TEST_ASSERT_EQUAL(FakeFader::BOOTLOADER, fader->mode);
}

static void test_whole_image_crc_mismatch_uncommits_page_zero() {
  fader->mode = FakeFader::BOOTLOADER;
  fader->corrupt_page = true;
  fader->corrupt_page_addr = BL_APP_START;  // page 0 itself goes bad
  auto img = make_image(0x0106);
  FirmwareUpdater u(*device);
  u.start(image_of(img));
  run(u);
  TEST_ASSERT_EQUAL(UpdateError::CRC_MISMATCH, u.error());
  TEST_ASSERT_EQUAL((int) UpdateStage::CRC_WAIT, (int) u.failed_stage());
  // Re-blanked: the fader stays in its bootloader across resets.
  TEST_ASSERT_EQUAL(2, fader->erase_count);
  TEST_ASSERT_TRUE(fader->reset_vector_blank());
}

static void test_wrong_version_after_install_fails() {
  fader->mode = FakeFader::BOOTLOADER;
  fader->app_reports_wrong_version = true;
  auto img = make_image(0x0106);
  FirmwareUpdater u(*device);
  u.start(image_of(img));
  run(u);
  TEST_ASSERT_EQUAL(UpdateError::FW_VERSION_MISMATCH, u.error());
}

static void test_absent_device() {
  fader->mode = FakeFader::ABSENT;
  auto img = make_image(0x0106);
  FirmwareUpdater u(*device);
  u.start(image_of(img));
  run(u);
  TEST_ASSERT_EQUAL(UpdateError::NOT_RESPONDING, u.error());
  TEST_ASSERT_EQUAL_STRING("device not responding", u.error_message());
}

static void test_enter_ignored_times_out_waiting_for_marker() {
  fader->fw_version = 0x0105;
  fader->reject_enter = true;
  auto img = make_image(0x0106);
  FirmwareUpdater u(*device);
  u.start(image_of(img));
  run(u);
  TEST_ASSERT_EQUAL(UpdateError::NO_MARKER, u.error());
  TEST_ASSERT_EQUAL(0, fader->erase_count);
}

static void test_bad_image_length() {
  auto img = make_image(0x0106);
  FirmwareImage bad = image_of(img);
  bad.length = 100;
  FirmwareUpdater u(*device);
  TEST_ASSERT_FALSE(u.start(bad));
  TEST_ASSERT_FALSE(u.running());
  TEST_ASSERT_EQUAL(UpdateError::BAD_IMAGE, u.error());
  TEST_ASSERT_EQUAL(0, fader->transactions);
}

static void test_cancel_stops_bus_traffic() {
  fader->mode = FakeFader::BOOTLOADER;
  auto img = make_image(0x0106);
  FirmwareUpdater u(*device);
  u.start(image_of(img));
  while (u.stage() != UpdateStage::WRITE) {
    TEST_ASSERT_TRUE(u.tick());
  }
  u.tick();
  u.cancel();
  int before = fader->transactions;
  TEST_ASSERT_FALSE(u.tick());
  TEST_ASSERT_EQUAL(before, fader->transactions);
  TEST_ASSERT_EQUAL(UpdateError::CANCELLED, u.error());
  TEST_ASSERT_TRUE(fader->reset_vector_blank());
}

static void test_write_stage_yields_within_budget() {
  fader->mode = FakeFader::BOOTLOADER;
  auto img = make_image(0x0106);
  FirmwareUpdater::Options opt;
  opt.write_budget_ms = 20;
  FirmwareUpdater u(*device);
  u.start(image_of(img), opt);
  while (u.stage() != UpdateStage::WRITE) {
    u.tick();
  }
  uint32_t before = u.pages_written();
  u.tick();
  uint32_t per_tick = u.pages_written() - before;
  TEST_ASSERT_TRUE(per_tick >= 1);
  TEST_ASSERT_TRUE_MESSAGE(per_tick < u.pages_total() - 1, "one tick wrote the whole image");
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_install_from_resident_bootloader);
  RUN_TEST(test_page_zero_is_written_last_and_only_once);
  RUN_TEST(test_install_from_running_app_enters_bootloader);
  RUN_TEST(test_refuses_firmware_predating_bootloader_entry);
  RUN_TEST(test_refuses_unversioned_firmware);
  RUN_TEST(test_unversioned_app_can_be_updated_when_gate_disabled);
  RUN_TEST(test_installs_fw_version_zero_image);
  RUN_TEST(test_skip_if_current_is_a_noop_success);
  RUN_TEST(test_reinstalls_same_version_without_skip);
  RUN_TEST(test_body_crc_mismatch_leaves_reset_vector_blank);
  RUN_TEST(test_whole_image_crc_mismatch_uncommits_page_zero);
  RUN_TEST(test_wrong_version_after_install_fails);
  RUN_TEST(test_absent_device);
  RUN_TEST(test_enter_ignored_times_out_waiting_for_marker);
  RUN_TEST(test_bad_image_length);
  RUN_TEST(test_cancel_stops_bus_traffic);
  RUN_TEST(test_write_stage_yields_within_budget);
  return UNITY_END();
}
