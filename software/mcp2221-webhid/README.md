# motorFader WebHID Demo

Web-based interface for controlling motorFader devices via MCP2221 USB-to-I2C bridge using WebHID.

## Setup

1. Install dependencies:
```bash
npm install
```

2. Build the bundle:
```bash
npm run build
```

3. Open `index.html` in a WebHID-compatible browser (Chrome, Edge, or other Chromium-based browsers)

## Development

To automatically rebuild on file changes:
```bash
npm run watch
```

## Usage

1. Connect your MCP2221 USB device to your computer
2. Ensure the motorFader is connected to the MCP2221's I2C pins
3. Open `index.html` in your browser
4. Click "Connect MCP2221" and select the device
5. Verify the I2C address matches your fader (default: 0x20)
6. Control and monitor your motorFader in real-time

## Features

- Real-time position monitoring (10Hz polling)
- Bidirectional control (set target position)
- Mode state visualization
- Touch detection monitoring
- Touch diagnostics (raw/delta/reference values)
- Error handling and recovery
- Self-calibration trigger

## Requirements

- WebHID-compatible browser (Chrome 89+, Edge 89+)
- MCP2221/MCP2221A USB-to-I2C bridge
- motorFader hardware with I2C interface
