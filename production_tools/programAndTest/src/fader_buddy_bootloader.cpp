/*
 * Copyright 2026 Scott Bezek
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "fader_buddy_bootloader.h"

FaderBuddyBootloader::FaderBuddyBootloader(uint8_t address)
    : _address(address), _wire(nullptr) {}

void FaderBuddyBootloader::begin(TwoWire* wire) {
  _wire = wire;
}

bool FaderBuddyBootloader::writeBytesRetry(const uint8_t* buf, size_t n,
                                           uint8_t attempts, uint32_t retryDelayMs) {
  if (_wire == nullptr) return false;
  for (uint8_t a = 0; a < attempts; a++) {
    _wire->beginTransmission(_address);
    _wire->write(buf, n);
    if (_wire->endTransmission() == 0) {
      return true;
    }
    delay(retryDelayMs);  // bootloader may be stalled in a flash erase/write
  }
  return false;
}

bool FaderBuddyBootloader::readRetry(const uint8_t* regbuf, size_t regn, uint8_t* out,
                                     size_t rn, uint8_t attempts, uint32_t retryDelayMs) {
  if (_wire == nullptr) return false;
  for (uint8_t a = 0; a < attempts; a++) {
    _wire->beginTransmission(_address);
    _wire->write(regbuf, regn);
    if (_wire->endTransmission(false) != 0) {  // repeated start
      delay(retryDelayMs);
      continue;
    }
    if (_wire->requestFrom(_address, rn) != rn) {
      delay(retryDelayMs);
      continue;
    }
    for (size_t i = 0; i < rn; i++) {
      out[i] = _wire->read();
    }
    return true;
  }
  return false;
}

bool FaderBuddyBootloader::readBytesRetry(uint8_t* out, size_t rn, uint8_t attempts,
                                          uint32_t retryDelayMs) {
  if (_wire == nullptr) return false;
  for (uint8_t a = 0; a < attempts; a++) {
    if (_wire->requestFrom(_address, rn) == rn) {
      for (size_t i = 0; i < rn; i++) {
        out[i] = _wire->read();
      }
      return true;
    }
    delay(retryDelayMs);  // target still busy (e.g. computing a CRC) -> NAK
  }
  return false;
}

bool FaderBuddyBootloader::readVersionByte(uint8_t& version) {
  uint8_t reg = 0x00;
  return readRetry(&reg, 1, &version, 1, 1, 0);  // single-shot probe, no retry
}

bool FaderBuddyBootloader::enterBootloader() {
  uint8_t buf[5] = {
      REG_ENTER_BOOTLOADER,
      (uint8_t)(ENTER_BOOTLOADER_MAGIC >> 24),
      (uint8_t)(ENTER_BOOTLOADER_MAGIC >> 16),
      (uint8_t)(ENTER_BOOTLOADER_MAGIC >> 8),
      (uint8_t)(ENTER_BOOTLOADER_MAGIC),
  };
  return writeBytesRetry(buf, sizeof(buf), 3, 5);
}

bool FaderBuddyBootloader::waitForMarker(uint32_t timeout_ms) {
  uint32_t start = millis();
  while (millis() - start < timeout_ms) {
    uint8_t v;
    if (readVersionByte(v) && v == BL_VERSION_MARKER) {
      return true;
    }
    delay(5);
  }
  return false;
}

bool FaderBuddyBootloader::waitForApp(uint32_t timeout_ms, uint8_t& version) {
  uint32_t start = millis();
  while (millis() - start < timeout_ms) {
    uint8_t v;
    if (readVersionByte(v) && v != BL_VERSION_MARKER && v != 0xFF && v != 0x00) {
      version = v;
      return true;
    }
    delay(5);
  }
  return false;
}

bool FaderBuddyBootloader::getStatus(uint8_t& blVersion, uint8_t& status,
                                     uint8_t& lastError) {
  uint8_t reg = BL_CMD_GET_STATUS;
  uint8_t out[3];
  if (!readRetry(&reg, 1, out, 3)) return false;
  blVersion = out[0];
  status = out[1];
  lastError = out[2];
  return true;
}

bool FaderBuddyBootloader::eraseApp() {
  uint8_t reg = BL_CMD_ERASE_APP;
  if (!writeBytesRetry(&reg, 1, 3, 5)) return false;
  // The erase stalls the target CPU (~hundreds of ms); wait for it to answer,
  // then confirm no NVM error was recorded.
  uint8_t v, s, e;
  for (uint8_t a = 0; a < 200; a++) {
    if (getStatus(v, s, e)) {
      return e == BL_ERR_NONE;
    }
    delay(10);
  }
  return false;
}

bool FaderBuddyBootloader::setPageAddr(uint16_t addr) {
  uint8_t buf[3] = {BL_CMD_SET_PAGE_ADDR, (uint8_t)(addr >> 8), (uint8_t)(addr & 0xFF)};
  return writeBytesRetry(buf, sizeof(buf));
}

bool FaderBuddyBootloader::sendFrame(const uint8_t* data16) {
  uint8_t buf[1 + BL_FRAME_DATA_LEN + 2];
  buf[0] = BL_CMD_SEND_FRAME;
  uint16_t crc = BL_CRC16_INIT;
  for (uint8_t i = 0; i < BL_FRAME_DATA_LEN; i++) {
    buf[1 + i] = data16[i];
    crc = bl_crc16_update(crc, data16[i]);
  }
  buf[1 + BL_FRAME_DATA_LEN] = (uint8_t)(crc >> 8);
  buf[1 + BL_FRAME_DATA_LEN + 1] = (uint8_t)(crc & 0xFF);
  return writeBytesRetry(buf, sizeof(buf));
}

bool FaderBuddyBootloader::getImageCrc16(uint16_t addr, uint16_t len, uint16_t& crc) {
  uint8_t buf[5] = {BL_CMD_GET_VERSION_CRC16, (uint8_t)(addr >> 8), (uint8_t)(addr & 0xFF),
                    (uint8_t)(len >> 8), (uint8_t)(len & 0xFF)};
  // Write the command terminated by a STOP. The bootloader computes the CRC over
  // the whole range *after* releasing the bus -- it cannot clock-stretch for the
  // entire computation because the ESP32 I2C hardware caps SCL stretching at
  // ~13ms. It then holds the 3-byte result and NAKs reads until the compute
  // finishes, so we poll with a separate retrying read.
  if (!writeBytesRetry(buf, sizeof(buf), 3, 5)) return false;
  delay(40);  // typical whole-image compute time; the read retries cover the rest
  uint8_t out[3];
  if (!readBytesRetry(out, 3, 80, 5)) return false;
  crc = ((uint16_t)out[1] << 8) | out[2];
  return true;
}

bool FaderBuddyBootloader::runApp() {
  uint8_t reg = BL_CMD_RUN_APP;
  return writeBytesRetry(&reg, 1, 3, 5);
}

bool FaderBuddyBootloader::readFwVersion(uint16_t& version) {
  uint8_t reg = REG_FW_VERSION;
  uint8_t out[2];
  if (!readRetry(&reg, 1, out, 2)) return false;
  version = ((uint16_t)out[0] << 8) | out[1];
  return true;
}

bool FaderBuddyBootloader::updateFirmware(const uint8_t* image, uint32_t length,
                                          uint16_t flashStart, uint16_t expectedImageCrc,
                                          uint16_t expectedFwVersion, char* err,
                                          size_t errLen, ProgressFn progress) {
#define BL_FAIL(msg)                            \
  do {                                          \
    snprintf(err, errLen, "%s", (msg));         \
    return false;                               \
  } while (0)

  if (length == 0 || (length % BL_PAGE_SIZE) != 0) BL_FAIL("bad image length");

  // 1) Make sure the bootloader is resident (drop out of the app if needed).
  uint8_t ver;
  if (!readVersionByte(ver)) BL_FAIL("no I2C response");
  if (ver != BL_VERSION_MARKER) {
    if (progress) progress("entering bootloader", 0, 0);
    if (!enterBootloader()) BL_FAIL("enter cmd failed");
    if (!waitForMarker(2000)) BL_FAIL("no bootloader marker");
  }

  // 2) Sanity-check the bootloader is answering commands.
  uint8_t blVer, st, lastErr;
  if (!getStatus(blVer, st, lastErr)) BL_FAIL("no status");

  // 3) Erase the application section.
  if (progress) progress("erasing", 0, 0);
  if (!eraseApp()) BL_FAIL("erase failed");

  // 4) Stream the image, one 64-byte page (4 frames) at a time.
  uint32_t pages = length / BL_PAGE_SIZE;
  for (uint32_t p = 0; p < pages; p++) {
    uint16_t pageAddr = flashStart + (uint16_t)(p * BL_PAGE_SIZE);
    if (!setPageAddr(pageAddr)) BL_FAIL("set page addr failed");
    for (uint8_t f = 0; f < BL_FRAMES_PER_PAGE; f++) {
      const uint8_t* chunk = image + (p * BL_PAGE_SIZE) + (f * BL_FRAME_DATA_LEN);
      if (!sendFrame(chunk)) BL_FAIL("send frame failed");
    }
    if (progress) progress("writing", p + 1, pages);
  }

  // 4b) Surface any NVM write error the bootloader recorded during streaming.
  {
    uint8_t bv, stt, le;
    if (getStatus(bv, stt, le) && le != BL_ERR_NONE) {
      snprintf(err, errLen, "nvm err=%u after write", le);
      return false;
    }
  }

  // 5) Verify the whole image via the target's own CRC over the written range.
  if (progress) progress("verifying", 0, 0);
  uint16_t crc;
  if (!getImageCrc16(flashStart, (uint16_t)length, crc)) BL_FAIL("crc read failed");
  if (crc != expectedImageCrc) {
    snprintf(err, errLen, "CRC got=0x%04X exp=0x%04X", crc, expectedImageCrc);
    return false;
  }

  // 6) Run the new application and confirm it comes up with the expected version.
  if (progress) progress("starting app", 0, 0);
  if (!runApp()) BL_FAIL("run app cmd failed");
  uint8_t appVer;
  if (!waitForApp(2000, appVer)) BL_FAIL("app did not start");

  uint16_t fw;
  if (!readFwVersion(fw)) BL_FAIL("no fw version");
  if (fw != expectedFwVersion) BL_FAIL("fw version mismatch");

  return true;
#undef BL_FAIL
}
