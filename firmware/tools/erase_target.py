Import("env")

# Adds a custom `erase` target (`pio run -e <env> -t erase`) that does a full
# chip erase over UPDI via pymcuprog, without building or flashing anything.
# Wipes flash (app + bootloader) but does not touch fuses (BOOTEND/APPEND
# survive, so a subsequent bootloader/app flash still lands at the right
# offsets).


def erase_chip(*args, **kwargs):
    env.Execute("$PYTHONEXE -m pip install --quiet pymcuprog")
    port = env.subst("$UPLOAD_PORT")
    env.Execute(
        '"$PYTHONEXE" "$PROJECT_DIR/firmware/tools/flash_with_fuses.py" '
        '--erase-only --port "%s"' % port
    )


env.AddCustomTarget(
    name="erase",
    dependencies=None,
    actions=[erase_chip],
    title="Full chip erase",
    description="Full flash erase over UPDI via pymcuprog",
)
