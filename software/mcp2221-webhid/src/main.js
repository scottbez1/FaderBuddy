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

import { MCP2221 } from '@johntalton/mcp2221';
import { I2CBusMCP2221 } from '@johntalton/i2c-bus-mcp2221';
import { FaderBuddy } from './faderBuddy.js';

// UI State
let fader = null;
let currentActiveLayer = 0;

// I2C bus mutex for serializing operations
let i2cBusy = false;
const i2cWaiters = [];

/**
 * Process queued I2C operations sequentially
 */
async function processQueue() {
    // If already processing, return (mutual exclusion)
    if (i2cBusy) return;

    i2cBusy = true;

    // Process all queued operations
    while (i2cWaiters.length > 0) {
        const item = i2cWaiters.shift();
        try {
            const result = await item.fn();
            item.resolve(result);
        } catch (error) {
            log('I2C operation error: ' + error.message);
            item.reject(error);
        }
    }

    i2cBusy = false;
}

/**
 * Run an async function with exclusive I2C bus access
 * Queues operations and processes them FIFO
 * @param {Function} fn - Async function to execute
 * @returns {Promise} - Result of the function
 */
async function runWithI2cBus(fn) {
    return new Promise((resolve, reject) => {
        i2cWaiters.push({ fn, resolve, reject });
        void processQueue();  // Don't await - let it run independently
    });
}

// Mode names
const MODE_NAMES = [
    'REMOTE_MOVEMENT',
    'INPUT_ACTIVE',
    'INPUT_IDLE',
    'ERROR',
    'SELF_CALIBRATION'
];

// Logging
function log(message) {
    const logDiv = document.getElementById('log');
    const timestamp = new Date().toLocaleTimeString();
    logDiv.innerHTML += `[${timestamp}] ${message}<br>`;
    logDiv.scrollTop = logDiv.scrollHeight;
}

/**
 * Enable or disable all elements that require a connection
 */
function setConnectionControlsEnabled(enabled) {
    document.querySelectorAll('.requires-connection').forEach(el => {
        el.disabled = !enabled;
    });
    if (enabled) {
        updateLayerTableHighlight();
    }
}

// Update UI with state
function updateUI(state) {
    if (!state) return;

    document.getElementById('currentPosition').textContent = state.position;
    document.getElementById('currentPositionSlider').value = state.position;
    document.getElementById('mode').textContent = MODE_NAMES[state.mode] || 'UNKNOWN';
    document.getElementById('touchStatus').textContent = state.touchDetected ? 'YES' : 'NO';
    document.getElementById('activeLayer').textContent = state.activeLayer;
    document.getElementById('posNonce').textContent = state.positionNonce;
    document.getElementById('doubleTapNonce').textContent = state.doubleTapNonce;
    document.getElementById('rawAdc').textContent = state.rawAdc;

    // Update active layer highlighting in table
    if (state.activeLayer !== currentActiveLayer) {
        currentActiveLayer = state.activeLayer;
        updateLayerTableHighlight();
    }
}

/**
 * Update the active layer highlighting in the table
 */
function updateLayerTableHighlight() {
    const rows = document.querySelectorAll('#layerTableBody tr');
    rows.forEach((row, index) => {
        if (index === currentActiveLayer) {
            row.classList.add('active-layer');
            row.querySelector('.activate-btn').disabled = true;
        } else {
            row.classList.remove('active-layer');
            row.querySelector('.activate-btn').disabled = false;
        }
    });
}

/**
 * Create the layer table rows
 */
function createLayerTable() {
    const tbody = document.getElementById('layerTableBody');
    tbody.innerHTML = '';

    for (let i = 0; i < 8; i++) {
        const row = document.createElement('tr');
        row.innerHTML = `
            <td>${i}</td>
            <td>
                <select id="layerMode${i}" class="requires-connection" disabled>
                    <option value="0">None</option>
                    <option value="1">Smooth+Ends</option>
                    <option value="2">Detents</option>
                </select>
            </td>
            <td>
                <input type="number" id="layerDetents${i}" class="requires-connection" min="0" max="15" value="0" disabled>
            </td>
            <td>
                <input type="number" id="layerStrength${i}" class="requires-connection" min="0" max="7" value="7" disabled>
            </td>
            <td>
                <button class="activate-btn requires-connection" id="activateLayer${i}" disabled>Activate</button>
            </td>
        `;
        tbody.appendChild(row);

        // Add event listeners for auto-apply on change
        const modeSelect = row.querySelector(`#layerMode${i}`);
        const detentsInput = row.querySelector(`#layerDetents${i}`);
        const strengthInput = row.querySelector(`#layerStrength${i}`);
        const activateBtn = row.querySelector(`#activateLayer${i}`);

        const layerIndex = i;

        modeSelect.addEventListener('change', () => applyLayerConfig(layerIndex));
        detentsInput.addEventListener('change', () => applyLayerConfig(layerIndex));
        strengthInput.addEventListener('change', () => applyLayerConfig(layerIndex));

        activateBtn.addEventListener('click', async () => {
            await runWithI2cBus(async () => {
                await fader.setActiveLayer(layerIndex);
                log(`Activated layer ${layerIndex}`);
            });
        });
    }
}

/**
 * Apply haptic config for a specific layer
 */
async function applyLayerConfig(layer) {
    if (!fader) return;

    const mode = parseInt(document.getElementById(`layerMode${layer}`).value);
    const detentCount = parseInt(document.getElementById(`layerDetents${layer}`).value);
    const detentStrength = parseInt(document.getElementById(`layerStrength${layer}`).value);

    await runWithI2cBus(async () => {
        await fader.setLayerHapticConfig(layer, mode, detentCount, detentStrength);
        log(`Layer ${layer}: mode=${fader.getHapticModeName(mode)}, detents=${detentCount}, strength=${detentStrength}`);
    });
}

/**
 * Read and populate all layer configurations
 */
async function refreshAllLayers() {
    if (!fader) return;

    await runWithI2cBus(async () => {
        for (let i = 0; i < 8; i++) {
            const config = await fader.readLayerHapticConfig(i);

            document.getElementById(`layerMode${i}`).value = config.mode;
            document.getElementById(`layerDetents${i}`).value = config.detentCount;
            document.getElementById(`layerStrength${i}`).value = config.detentStrength;
        }
        log('Refreshed all layer configurations');
    });
}


// Polling function with serialized I2C operations
let pollCounter = 0;
let pollTimeout = null;

async function poll() {
    if (!fader) {
        return;
    }

    await runWithI2cBus(async () => {
        // Always read state
        const state = await fader.readState();
        updateUI(state);

        // Read diagnostics every 10th poll (~1 second with 100ms delay)
        if (pollCounter % 10 === 0) {
            const uptime = await fader.readUptime();
            document.getElementById('uptime').textContent = (uptime / 1000).toFixed(1) + 's';

            const touchRaw = await fader.readTouchRaw();
            document.getElementById('touchRaw').textContent = touchRaw;

            const touchDelta = await fader.readTouchDelta();
            document.getElementById('touchDelta').textContent = touchDelta;

            const touchRef = await fader.readTouchRef();
            document.getElementById('touchRef').textContent = touchRef;

            const touchRecal = await fader.readTouchRecal();
            document.getElementById('touchRecal').textContent = touchRecal;
        }

        pollCounter++;
    });

    // Schedule next poll after this one completes
    if (fader) {
        pollTimeout = setTimeout(poll, 100);
    }
}

// Store device reference for cleanup
let hidDevice = null;

// Initialize layer table on page load
createLayerTable();

// Connect button
document.getElementById('connectBtn').addEventListener('click', async () => {
    try {
        log('Requesting MCP2221 device...');

        const devices = await navigator.hid.requestDevice({
            filters: [{ vendorId: 0x04D8, productId: 0x00DD }]
        });

        if (devices.length === 0) {
            throw new Error('No device selected');
        }

        hidDevice = devices[0];
        await hidDevice.open();

        // Create readable/writable streams for the HID device
        let pendingRead = null;

        const readable = new ReadableStream({
            type: 'bytes',
            start(controller) {
                hidDevice.addEventListener('inputreport', (event) => {
                    const data = new Uint8Array(event.data.buffer);

                    if (pendingRead) {
                        const { view, resolve } = pendingRead;
                        const bytesToCopy = Math.min(data.length, view.byteLength);
                        new Uint8Array(view.buffer, view.byteOffset, bytesToCopy).set(data.subarray(0, bytesToCopy));
                        resolve();
                        pendingRead = null;
                        controller.byobRequest?.respond(bytesToCopy);
                    } else {
                        controller.enqueue(data);
                    }
                });
            },
            pull(controller) {
                // BYOB read request
                if (controller.byobRequest) {
                    return new Promise((resolve) => {
                        pendingRead = {
                            view: controller.byobRequest.view,
                            resolve
                        };
                    });
                }
            }
        });

        const writable = new WritableStream({
            async write(chunk) {
                await hidDevice.sendReport(0, chunk);
            }
        });

        // Create binding object with readable/writable streams
        const binding = { readable, writable };

        // Create MCP2221 device instance
        const device = new MCP2221(binding);

        // Create I2C bus from the MCP2221 device
        const bus = I2CBusMCP2221.from(device);

        log('Connected to MCP2221');

        const addrStr = document.getElementById('i2cAddress').value;
        const addr = parseInt(addrStr);
        fader = new FaderBuddy(bus, addr);

        // Read version
        const version = await fader.readVersion();
        log(`Protocol version: ${version}`);
        document.getElementById('version').textContent = version;

        // Enable UI
        document.getElementById('connectBtn').disabled = true;
        document.getElementById('disconnectBtn').disabled = false;
        document.getElementById('connectionStatus').textContent = 'Connected';
        document.getElementById('connectionStatus').className = 'status connected';
        setConnectionControlsEnabled(true);

        // Refresh layer data
        await refreshAllLayers();

        // Start polling (will reschedule itself after each completion)
        poll();

    } catch (e) {
        log('Connection failed: ' + e.message);
    }
});

// Disconnect button
document.getElementById('disconnectBtn').addEventListener('click', async () => {
    // Stop polling by clearing fader (poll() checks for this)
    fader = null;

    if (pollTimeout) {
        clearTimeout(pollTimeout);
        pollTimeout = null;
    }

    if (hidDevice && hidDevice.opened) {
        await hidDevice.close();
    }
    hidDevice = null;

    document.getElementById('connectBtn').disabled = false;
    document.getElementById('disconnectBtn').disabled = true;
    document.getElementById('connectionStatus').textContent = 'Not connected';
    document.getElementById('connectionStatus').className = 'status disconnected';
    setConnectionControlsEnabled(false);

    log('Disconnected');
});

// Target slider
document.getElementById('targetSlider').addEventListener('input', (e) => {
    document.getElementById('targetValue').textContent = e.target.value;
});

// Set target button
document.getElementById('setTargetBtn').addEventListener('click', async () => {
    const target = parseInt(document.getElementById('targetSlider').value);
    await runWithI2cBus(async () => {
        await fader.setLayerTarget(currentActiveLayer, target);
        log(`Set target position to ${target} on layer ${currentActiveLayer}`);
    });
});

// Calibrate touch button
document.getElementById('calTouchBtn').addEventListener('click', async () => {
    await runWithI2cBus(async () => {
        await fader.calibrateTouch();
        log('Touch calibration triggered');
    });
});

// Clear error button
document.getElementById('clearErrorBtn').addEventListener('click', async () => {
    await runWithI2cBus(async () => {
        await fader.clearError();
        log('Error cleared');
    });
});

// Self calibration button
document.getElementById('selfCalBtn').addEventListener('click', async () => {
    await runWithI2cBus(async () => {
        await fader.selfCalibrate();
        log('Self calibration started');
    });
});

// Refresh layers button
document.getElementById('refreshLayersBtn').addEventListener('click', async () => {
    await refreshAllLayers();
});

// Clear log button
document.getElementById('clearLogBtn').addEventListener('click', () => {
    document.getElementById('log').innerHTML = '';
});

log('FaderBuddy WebHID Demo loaded. Click "Connect MCP2221" to begin.');
