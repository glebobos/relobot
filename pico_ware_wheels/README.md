# Wheels RP2040 CircuitPython firmware

This directory contains the CircuitPython code for the wheel controllers.

## Files
- `code.py`: Main control logic.
- `boot.py`: USB and storage configuration.

## Uploading Code
To update the Python logic, simply copy the files to the mounted `WHEELS` drive:
```bash
cp code.py /media/admin/WHEELS/
```

## Updating Firmware (CircuitPython)
If you need to update the CircuitPython firmware itself:

1. **Put the board in BOOTSEL mode**:
   ```bash
   stty -F /dev/ttyACM0 1200
   ```
2. **Copy the CircuitPython UF2**:
   ```bash
   cp circuitpython_9.x.x.uf2 /media/admin/RPI-RP2/
   ```

For more details, see [FLASHING.md](../../FLASHING.md).
