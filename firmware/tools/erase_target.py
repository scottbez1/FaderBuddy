Import("env")

# Adds `pio run --target erase`, which performs a full UPDI chip erase
# (flash + EEPROM) without programming anything. Uses the same device/port/baud
# flags as the normal upload (see `upload_flags` in platformio.ini).

env.AddCustomTarget(
    name="erase",
    dependencies=None,
    actions=["pyupdi $UPLOAD_FLAGS -e"],
    title="Erase Chip",
    description="Full UPDI chip erase (flash + EEPROM, fuses untouched)",
    always_build=True,
)
