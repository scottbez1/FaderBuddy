## Firmware Flashing

If you purchase a FaderBuddy PCB assembly from Bezek Labs, it will come with firmware already uploaded and tested, so you won't need to flash any firmware to get started. You can use these instructions if you'd like to update the firmware later.

If you assemble your own FaderBuddy PCBs or purchase an assembled PCB from JLCPCB, you'll have a blank ATtiny1616 microcontroller, so you'll need to flash the firmware yourself using a UPDI programmer before you can use it. This only needs to be done once per board (unless you want to update the firmware later).

The [Adafruit UPDI Friend](https://www.adafruit.com/product/5879) is a recommended UPDI programmer, but you can also build an UPDI programmer with a USB->Serial adapter and a few other components (see a comprehensive overview from SpenceKonde [here](https://github.com/SpenceKonde/AVR-Guidance/blob/master/UPDI/jtag2updi.md#a-note-on-breakout-boards)).

### Connecting the UPDI Friend

On the back of the FaderBuddy PCB, there are three through-hole test points labeled for UPDI programming, laid out at 0.1" spacing. You can solder on standard pin headers for repeated use, or simply hold jumper wires against the pads for a one-time flash.

Connect the Adafruit UPDI Friend to the test points as follows:

| UPDI Friend wire | FaderBuddy pad |
|------------------|----------------|
| Black (GND)      | VIO **-**          |
| Red (VCC)        | VIO **+**          |
| White (UPDI)     | **U**          |

<img width="654" height="660" alt="FaderBuddy wiring(6)" src="https://github.com/user-attachments/assets/c0eb65ca-65b3-4f0f-8f84-f7e84c4d8df0" />

### Flashing Steps

1. **Install PlatformIO:**

   If you don't already have PlatformIO installed, the easiest way is via VS Code: install the [PlatformIO IDE extension](https://platformio.org/install/ide?install=vscode).

2. **Connect the UPDI Friend** to the FaderBuddy board's UPDI and GND pads, and plug it into your computer via USB.

3. **Build and upload the firmware:**
   Use the "Upload" option in the PlatformIO extension. PlatformIO will automatically download the required toolchain, compile the firmware, and upload it to the ATtiny1616 via the UPDI Friend.

4. **Verify** the board is working by connecting it to an I2C bus -- it should appear at its configured address (default 0x20). You can verify with the ESPHome I2C scan (`scan: true`) or the WebHID debug tool if you have an MCP2221.

### Troubleshooting Firmware Upload

- **"No device found" or upload fails:** Check your UPDI and GND connections. UPDI requires only a single data line plus ground.
- **Wrong serial port:** PlatformIO should auto-detect, but you can set the upload port explicitly in `platformio.ini` or via command line: `pio run --target upload --upload-port /dev/ttyUSB0`
- **Permission denied (Linux):** You may need to add your user to the `dialout` group: `sudo usermod -a -G dialout $USER` (then log out and back in).

