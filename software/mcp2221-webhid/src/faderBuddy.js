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

/**
 * FaderBuddy I2C Protocol Implementation
 * Protocol version: 3
 * All multi-byte values are big-endian (MSB first)
 */

export class FaderBuddy {
    constructor(i2cBus, address = 0x20) {
        this.bus = i2cBus;
        this.address = address;

        // Register addresses
        this.REG_VERSION = 0x00;
        this.REG_STATE = 0x01;
        this.REG_TARGET = 0x02;
        this.REG_UPTIME = 0x03;
        this.REG_CAL_TOUCH = 0x04;
        this.REG_CLEAR_ERROR = 0x05;
        this.REG_TOUCH_RAW = 0x06;
        this.REG_SELF_CAL = 0x07;
        this.REG_SERIAL = 0x08;
        this.REG_TOUCH_DELTA = 0x09;
        this.REG_TOUCH_REF = 0x0A;
        this.REG_TOUCH_RECAL = 0x0B;
        this.REG_HAPTIC_CONFIG = 0x0C;

        // Mode states
        this.MODE_REMOTE_MOVEMENT = 0;
        this.MODE_INPUT_ACTIVE = 1;
        this.MODE_INPUT_IDLE = 2;
        this.MODE_ERROR = 3;
        this.MODE_SELF_CALIBRATION = 4;

        // Haptic modes
        this.HAPTIC_NO_HAPTICS = 0;
        this.HAPTIC_SMOOTH_WITH_MAGNET_ENDS = 1;
        this.HAPTIC_DETENTS = 2;
    }

    /**
     * Read a register using the I2C bus
     * @param {number} reg - Register address
     * @param {number} length - Number of bytes to read
     * @returns {Uint8Array} - Register data
     */
    async readRegister(reg, length) {
        const result = await this.bus.readI2cBlock(this.address, reg, length);
        return new Uint8Array(result.buffer);
    }

    /**
     * Write to a register
     * @param {number} reg - Register address
     * @param {Uint8Array} data - Data to write (optional for command registers)
     */
    async writeRegister(reg, data = new Uint8Array(0)) {
        await this.bus.writeI2cBlock(this.address, reg, data.length, data);
    }

    /**
     * Convert big-endian bytes to uint16
     */
    bytesToUint16BE(bytes, offset = 0) {
        return (bytes[offset] << 8) | bytes[offset + 1];
    }

    /**
     * Convert big-endian bytes to int16 (signed)
     */
    bytesToInt16BE(bytes, offset = 0) {
        const val = (bytes[offset] << 8) | bytes[offset + 1];
        // Convert to signed
        return val > 32767 ? val - 65536 : val;
    }

    /**
     * Convert big-endian bytes to uint32
     */
    bytesToUint32BE(bytes, offset = 0) {
        return (bytes[offset] << 24) |
               (bytes[offset + 1] << 16) |
               (bytes[offset + 2] << 8) |
               bytes[offset + 3];
    }

    /**
     * Read protocol version
     * @returns {number} - Protocol version
     */
    async readVersion() {
        const data = await this.readRegister(this.REG_VERSION, 1);
        return data[0];
    }

    /**
     * Read and parse state register (REG_STATE)
     * Returns an object with all state fields extracted from the packed uint32
     * @returns {Object} - State object with fields: touchDetected, mode, settingsNonce, position, positionNonce, rawAdc, singleTapNonce, doubleTapNonce
     */
    async readState() {
        const data = await this.readRegister(this.REG_STATE, 4);
        const state = this.bytesToUint32BE(data);

        // Extract bitfields
        // Bit 0: Touch detected
        const touchDetected = (state & 0x01) !== 0;

        // Bits 1-3: Mode (3 bits)
        const mode = (state >> 1) & 0x07;

        // Bits 4-5: Settings nonce (2 bits)
        const settingsNonce = (state >> 4) & 0x03;

        // Bits 6-13: Position (8 bits)
        const position = (state >> 6) & 0xFF;

        // Bits 14-15: Position nonce (2 bits)
        const positionNonce = (state >> 14) & 0x03;

        // Bits 16-26: Raw ADC (11 bits)
        const rawAdc = (state >> 16) & 0x7FF;

        // Bits 27-28: Single tap nonce (2 bits)
        const singleTapNonce = (state >> 27) & 0x03;

        // Bits 29-30: Double tap nonce (2 bits)
        const doubleTapNonce = (state >> 29) & 0x03;

        return {
            touchDetected,
            mode,
            settingsNonce,
            position,
            positionNonce,
            rawAdc,
            singleTapNonce,
            doubleTapNonce
        };
    }

    /**
     * Read target position
     * @returns {number} - Target position (0-255)
     */
    async readTarget() {
        const data = await this.readRegister(this.REG_TARGET, 1);
        return data[0];
    }

    /**
     * Set target position
     * @param {number} target - Target position (0-255)
     */
    async setTarget(target) {
        const data = new Uint8Array([target & 0xFF]);
        await this.writeRegister(this.REG_TARGET, data);
    }

    /**
     * Read uptime in milliseconds
     * @returns {number} - Uptime in milliseconds
     */
    async readUptime() {
        const data = await this.readRegister(this.REG_UPTIME, 4);
        return this.bytesToUint32BE(data);
    }

    /**
     * Trigger touch sensor calibration
     */
    async calibrateTouch() {
        await this.writeRegister(this.REG_CAL_TOUCH);
    }

    /**
     * Clear error state
     */
    async clearError() {
        await this.writeRegister(this.REG_CLEAR_ERROR);
    }

    /**
     * Read raw touch sensor value
     * @returns {number} - Raw touch value (0-65535)
     */
    async readTouchRaw() {
        const data = await this.readRegister(this.REG_TOUCH_RAW, 2);
        return this.bytesToUint16BE(data);
    }

    /**
     * Start self-calibration sequence
     */
    async selfCalibrate() {
        await this.writeRegister(this.REG_SELF_CAL);
    }

    /**
     * Read device serial number
     * @returns {Uint8Array} - 10-byte serial number
     */
    async readSerial() {
        return await this.readRegister(this.REG_SERIAL, 10);
    }

    /**
     * Read touch delta (signed)
     * @returns {number} - Touch delta value
     */
    async readTouchDelta() {
        const data = await this.readRegister(this.REG_TOUCH_DELTA, 2);
        return this.bytesToInt16BE(data);
    }

    /**
     * Read touch reference baseline
     * @returns {number} - Touch reference value
     */
    async readTouchRef() {
        const data = await this.readRegister(this.REG_TOUCH_REF, 2);
        return this.bytesToUint16BE(data);
    }

    /**
     * Read touch recalibration event counter
     * @returns {number} - Recalibration count
     */
    async readTouchRecal() {
        const data = await this.readRegister(this.REG_TOUCH_RECAL, 2);
        return this.bytesToUint16BE(data);
    }

    /**
     * Set haptic configuration
     * @param {number} nonce - 2-bit nonce (0-3)
     * @param {number} mode - Haptic mode (0=NO_HAPTICS, 1=SMOOTH_WITH_MAGNET_ENDS, 2=DETENTS)
     * @param {number} detentCount - 4-bit detent count (0-15)
     * @param {number} detentStrength - 3-bit detent strength (0-7)
     * @param {number} targetPosition - 8-bit target position (0-255)
     */
    async setHapticConfig(nonce, mode, detentCount, detentStrength, targetPosition) {
        // Pack the 32-bit value according to the bit layout:
        // Bits 0-1: Nonce (2 bits)
        // Bits 2-4: Mode (3 bits)
        // Bits 5-8: Detent count (4 bits)
        // Bits 9-11: Detent strength (3 bits)
        // Bits 12-19: Target position (8 bits)
        const config =
            ((nonce & 0x03) << 0) |
            ((mode & 0x07) << 2) |
            ((detentCount & 0x0F) << 5) |
            ((detentStrength & 0x07) << 9) |
            ((targetPosition & 0xFF) << 12);

        // Convert to big-endian bytes
        const data = new Uint8Array([
            (config >> 24) & 0xFF,
            (config >> 16) & 0xFF,
            (config >> 8) & 0xFF,
            config & 0xFF
        ]);

        await this.writeRegister(this.REG_HAPTIC_CONFIG, data);
    }
}
