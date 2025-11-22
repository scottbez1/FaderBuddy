import { MCP2221 } from '@johntalton/mcp2221';
import { I2CBusMCP2221 } from '@johntalton/i2c-bus-mcp2221';
import { MotorFader } from './motorFader.js';

// UI State
let chip = null;
let fader = null;

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

// Update UI with state
function updateUI(state) {
    if (!state) return;

    document.getElementById('currentPosition').textContent = state.position;
    document.getElementById('mode').textContent = MODE_NAMES[state.mode] || 'UNKNOWN';
    document.getElementById('touchStatus').textContent = state.touchDetected ? 'YES' : 'NO';
    document.getElementById('posNonce').textContent = state.positionNonce;
    document.getElementById('rawAdc').textContent = state.rawAdc;
}

// Polling function with serialized I2C operations
let polling = false;
let pollCounter = 0;
let pollTimeout = null;

async function poll() {
    if (!fader || polling) {
        // Reschedule if still active
        if (fader) {
            pollTimeout = setTimeout(poll, 100);
        }
        return;
    }

    polling = true;
    try {
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
    } catch (e) {
        log('Error polling: ' + e.message);
    } finally {
        polling = false;

        // Schedule next poll after this one completes
        if (fader) {
            pollTimeout = setTimeout(poll, 100);
        }
    }
}

// Store device reference for cleanup
let hidDevice = null;

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
        fader = new MotorFader(bus, addr);

        // Read version
        const version = await fader.readVersion();
        log(`Protocol version: ${version}`);
        document.getElementById('version').textContent = version;

        // Enable UI
        document.getElementById('connectBtn').disabled = true;
        document.getElementById('disconnectBtn').disabled = false;
        document.getElementById('targetSlider').disabled = false;
        document.getElementById('setTargetBtn').disabled = false;
        document.getElementById('calTouchBtn').disabled = false;
        document.getElementById('clearErrorBtn').disabled = false;
        document.getElementById('selfCalBtn').disabled = false;
        document.getElementById('connectionStatus').textContent = 'Connected';
        document.getElementById('connectionStatus').className = 'status connected';

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
    chip = null;

    document.getElementById('connectBtn').disabled = false;
    document.getElementById('disconnectBtn').disabled = true;
    document.getElementById('targetSlider').disabled = true;
    document.getElementById('setTargetBtn').disabled = true;
    document.getElementById('calTouchBtn').disabled = true;
    document.getElementById('clearErrorBtn').disabled = true;
    document.getElementById('selfCalBtn').disabled = true;
    document.getElementById('connectionStatus').textContent = 'Not connected';
    document.getElementById('connectionStatus').className = 'status disconnected';

    log('Disconnected');
});

// Target slider
document.getElementById('targetSlider').addEventListener('input', (e) => {
    document.getElementById('targetValue').textContent = e.target.value;
});

// Set target button
document.getElementById('setTargetBtn').addEventListener('click', async () => {
    if (polling) return;
    polling = true;

    const target = parseInt(document.getElementById('targetSlider').value);
    try {
        await fader.setTarget(target);
        log(`Set target position to ${target}`);
    } catch (e) {
        log('Error setting target: ' + e.message);
    } finally {
        polling = false;
    }
});

// Calibrate touch button
document.getElementById('calTouchBtn').addEventListener('click', async () => {
    if (polling) return;
    polling = true;

    try {
        await fader.calibrateTouch();
        log('Touch calibration triggered');
    } catch (e) {
        log('Error calibrating touch: ' + e.message);
    } finally {
        polling = false;
    }
});

// Clear error button
document.getElementById('clearErrorBtn').addEventListener('click', async () => {
    if (polling) return;
    polling = true;

    try {
        await fader.clearError();
        log('Error cleared');
    } catch (e) {
        log('Error clearing error: ' + e.message);
    } finally {
        polling = false;
    }
});

// Self calibration button
document.getElementById('selfCalBtn').addEventListener('click', async () => {
    if (polling) return;
    polling = true;

    try {
        await fader.selfCalibrate();
        log('Self calibration started');
    } catch (e) {
        log('Error starting self calibration: ' + e.message);
    } finally {
        polling = false;
    }
});

// Clear log button
document.getElementById('clearLogBtn').addEventListener('click', () => {
    document.getElementById('log').innerHTML = '';
});

log('motorFader WebHID Demo loaded. Click "Connect MCP2221" to begin.');
