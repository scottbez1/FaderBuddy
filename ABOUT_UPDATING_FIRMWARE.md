## Firmware Flashing

If you purchase a [FaderBuddy PCB assembly from Bezek Labs](bezeklabs.etsy.com/listing/4506790932), it will come with firmware already uploaded and tested, so you won't need to flash any firmware to get started. The firmware can be updated over I2C (built into the ESPHome component and Home Assistant integration). You can use these instructions to update over UPDI, but the I2C updates are usually easier as long as the bootloader is already flashed.

If you [assemble your own](ABOUT_PCB_FABRICATION.md) FaderBuddy PCBs or purchase an assembled PCB from JLCPCB, you'll have a blank ATtiny1616 microcontroller, so you MUST flash the firmware yourself using a UPDI programmer before you can use it. This only needs to be done once per board (unless you want to update the firmware later).

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

3. **Upload the bootloader, then the application.** The firmware is two images: the [I2C bootloader](ABOUT_I2C_BOOTLOADER.md) and the application that runs after it. Upload them in this order, from the repo root:

   ```bash
   pio run -e fb_bootloader_only -t upload   # erases the chip, installs the bootloader and sets its fuses
   pio run -e fb_app_only -t upload          # installs the application on top
   ```

   Order matters: the bootloader upload erases the whole chip, so uploading it second would wipe the application. PlatformIO automatically downloads the required toolchain the first time.

   In the PlatformIO extension, the equivalent is the "Upload" task under `fb_bootloader_only` and then the one under `fb_app_only`. The plain "Upload" for the default environment runs both, in that order.

   Once the bootloader is installed, later updates need only the second command (or no UPDI at all; see [ABOUT_I2C_BOOTLOADER.md](ABOUT_I2C_BOOTLOADER.md)).

4. **Verify** the board is working by connecting it to an I2C bus -- it should appear at its configured address (default 0x20). You can verify with the ESPHome I2C scan (`scan: true`) or the WebHID debug tool if you have an MCP2221.

### Troubleshooting Firmware Upload

- **"No device found" or upload fails:** Check your UPDI and GND connections. UPDI requires only a single data line plus ground.
- **Wrong serial port:** PlatformIO should auto-detect, but you can set the upload port explicitly in `platformio.ini` or via command line: `pio run --target upload --upload-port /dev/ttyUSB0`
- **Permission denied (Linux):** You may need to add your user to the `dialout` group: `sudo usermod -a -G dialout $USER` (then log out and back in).

