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
 * Protocol version: 5
 * All multi-byte values are big-endian (MSB first)
 *
 * v5 Changes from v3:
 * - REG_TARGET (0x02) removed - use layer-addressed REG_LAYER_TARGET (0x0E)
 * - REG_HAPTIC_CONFIG (0x0C) deprecated - use layer-addressed REG_LAYER_HAPTIC_CONFIG (0x0F)
 * - STATE register bitfields changed (active_layer replaces haptic_config_nonce, positions shifted)
 * - Haptic config is now 16-bit (removed nonce and target position)
 * - New layer-addressed protocol for per-layer configuration
 */

export class FaderBuddy {
    constructor(i2cBus, address = 0x20) {
        this.bus = i2cBus;
        this.address = address;

        // Register addresses (Protocol v5)
        this.REG_VERSION = 0x00;
        this.REG_STATE = 0x01;
        // 0x02 removed in v5
        this.REG_UPTIME = 0x03;
        this.REG_CAL_TOUCH = 0x04;
        this.REG_CLEAR_ERROR = 0x05;
        this.REG_TOUCH_RAW = 0x06;
        this.REG_SELF_CAL = 0x07;
        this.REG_SERIAL = 0x08;
        this.REG_TOUCH_DELTA = 0x09;
        this.REG_TOUCH_REF = 0x0A;
        this.REG_TOUCH_RECAL = 0x0B;
        // 0x0C deprecated in v5
        this.REG_ACTIVE_LAYER = 0x0D;
        this.REG_LAYER_TARGET = 0x0E;         // Layer-addressed
        this.REG_LAYER_HAPTIC_CONFIG = 0x0F;  // Layer-addressed

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

        // Number of layers
        this.NUM_LAYERS = 8;
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
     * Read a layer-addressed register
     * Protocol: Write [register, layer], then read N bytes
     * @param {number} reg - Register address
     * @param {number} layer - Layer index (0-7)
     * @param {number} length - Number of bytes to read
     * @returns {Uint8Array} - Register data
     */
    async readLayerRegister(reg, layer, length) {
        // Write register address and layer index
        await this.bus.writeI2cBlock(this.address, reg, 1, new Uint8Array([layer]));
        // Read the data (use raw read without register address)
        const result = await this.bus.i2cRead(this.address, length);
        return new Uint8Array(result.buffer);
    }

    /**
     * Write to a layer-addressed register
     * Protocol: Write [register, layer, ...data]
     * @param {number} reg - Register address
     * @param {number} layer - Layer index (0-7)
     * @param {Uint8Array} data - Data to write
     */
    async writeLayerRegister(reg, layer, data) {
        const payload = new Uint8Array([layer, ...data]);
        await this.bus.writeI2cBlock(this.address, reg, payload.length, payload);
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
     * Convert uint16 to big-endian bytes
     */
    uint16ToBytesBE(val) {
        return new Uint8Array([(val >> 8) & 0xFF, val & 0xFF]);
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
     * Read and parse state register (REG_STATE) - Protocol v5 format
     * Returns an object with all state fields extracted from the packed uint32
     *
     * v5 STATE register bitfields:
     * - Bit 0: Touch detected (1 bit)
     * - Bits 1-3: Mode (3 bits)
     * - Bits 4-6: Active layer (3 bits) - NEW in v5
     * - Bits 7-14: Position (8 bits)
     * - Bits 15-16: Position nonce (2 bits)
     * - Bits 17-27: Raw ADC (11 bits)
     * - Bits 28-29: Double tap nonce (2 bits)
     *
     * @returns {Object} - State object
     */
    async readState() {
        const data = await this.readRegister(this.REG_STATE, 4);
        const state = this.bytesToUint32BE(data);

        // Extract bitfields (Protocol v5 layout)
        // Bit 0: Touch detected
        const touchDetected = (state & 0x01) !== 0;

        // Bits 1-3: Mode (3 bits)
        const mode = (state >> 1) & 0x07;

        // Bits 4-6: Active layer (3 bits) - NEW in v5
        const activeLayer = (state >> 4) & 0x07;

        // Bits 7-14: Position (8 bits)
        const position = (state >> 7) & 0xFF;

        // Bits 15-16: Position nonce (2 bits)
        const positionNonce = (state >> 15) & 0x03;

        // Bits 17-27: Raw ADC (11 bits)
        const rawAdc = (state >> 17) & 0x7FF;

        // Bits 28-29: Double tap nonce (2 bits)
        const doubleTapNonce = (state >> 28) & 0x03;

        return {
            touchDetected,
            mode,
            activeLayer,
            position,
            positionNonce,
            rawAdc,
            doubleTapNonce
        };
    }

    /**
     * Read active layer index
     * @returns {number} - Active layer (0-7)
     */
    async readActiveLayer() {
        const data = await this.readRegister(this.REG_ACTIVE_LAYER, 1);
        return data[0];
    }

    /**
     * Set active layer index
     * @param {number} layer - Layer index (0-7)
     */
    async setActiveLayer(layer) {
        const data = new Uint8Array([layer & 0x07]);
        await this.writeRegister(this.REG_ACTIVE_LAYER, data);
    }

    /**
     * Read target position for a specific layer
     * @param {number} layer - Layer index (0-7)
     * @returns {number} - Target position (0-255)
     */
    async readLayerTarget(layer) {
        const data = await this.readLayerRegister(this.REG_LAYER_TARGET, layer, 1);
        return data[0];
    }

    /**
     * Set target position for a specific layer
     * @param {number} layer - Layer index (0-7)
     * @param {number} target - Target position (0-255)
     */
    async setLayerTarget(layer, target) {
        const data = new Uint8Array([target & 0xFF]);
        await this.writeLayerRegister(this.REG_LAYER_TARGET, layer, data);
    }

    /**
     * Read haptic configuration for a specific layer (16-bit, Protocol v5)
     *
     * Haptic config bitfields (16-bit):
     * - Bits 0-2: Mode (3 bits)
     * - Bits 3-6: Detent count (4 bits)
     * - Bits 7-9: Detent strength (3 bits)
     * - Bits 10-15: Reserved
     *
     * @param {number} layer - Layer index (0-7)
     * @returns {Object} - Haptic config object
     */
    async readLayerHapticConfig(layer) {
        const data = await this.readLayerRegister(this.REG_LAYER_HAPTIC_CONFIG, layer, 2);
        const config = this.bytesToUint16BE(data);

        // Extract bitfields
        const mode = config & 0x07;
        const detentCount = (config >> 3) & 0x0F;
        const detentStrength = (config >> 7) & 0x07;

        return {
            mode,
            detentCount,
            detentStrength
        };
    }

    /**
     * Set haptic configuration for a specific layer (16-bit, Protocol v5)
     * @param {number} layer - Layer index (0-7)
     * @param {number} mode - Haptic mode (0=NO_HAPTICS, 1=SMOOTH_WITH_MAGNET_ENDS, 2=DETENTS)
     * @param {number} detentCount - 4-bit detent count (0-15)
     * @param {number} detentStrength - 3-bit detent strength (0-7)
     */
    async setLayerHapticConfig(layer, mode, detentCount, detentStrength) {
        // Pack the 16-bit value
        const config =
            ((mode & 0x07) << 0) |
            ((detentCount & 0x0F) << 3) |
            ((detentStrength & 0x07) << 7);

        const data = this.uint16ToBytesBE(config);
        await this.writeLayerRegister(this.REG_LAYER_HAPTIC_CONFIG, layer, data);
    }

    /**
     * Set target position for the active layer (convenience method)
     * This sets the target on the currently active layer
     * @param {number} target - Target position (0-255)
     */
    async setTarget(target) {
        const activeLayer = await this.readActiveLayer();
        await this.setLayerTarget(activeLayer, target);
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
     * Get haptic mode name
     * @param {number} mode - Haptic mode value
     * @returns {string} - Mode name
     */
    getHapticModeName(mode) {
        switch (mode) {
            case this.HAPTIC_NO_HAPTICS: return 'None';
            case this.HAPTIC_SMOOTH_WITH_MAGNET_ENDS: return 'Smooth+Ends';
            case this.HAPTIC_DETENTS: return 'Detents';
            default: return 'Unknown';
        }
    }
}
