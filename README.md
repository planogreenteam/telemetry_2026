# Sending Data Between Devices

## LoRa
1. Find your serial devices:
```bash
ls /dev/tty.*
```

2. Start the generic telemetry receiver:
```bash
python3 telemetry_receiver.py --transport lora --port /dev/tty.usbserial-0001
```

3. Start the generic telemetry sender for the BMV device:
```bash
python3 telemetry_sender.py --device bmv --transport lora --bmv-port /dev/tty.usbserial-VE7ALZXZ --lora-port /dev/tty.usbserial-0001
```

## Dry Run
Build BMV telemetry packets without transmitting over LoRa:
```bash
python3 telemetry_sender.py --device bmv --transport lora --dry-run
```

## Build Executables
Build standalone binaries for `telemetry_receiver.py` and `telemetry_sender.py` with PyInstaller:

1. Install PyInstaller for the Python interpreter you want to use:
```bash
python3 -m pip install --user pyinstaller
```

2. Run the build script:
```bash
cd packaging/pyinstaller
./build.sh
```

You can also build with `make`:
```bash
cd packaging/pyinstaller
make
```

The generated executables are written to `dist/` at the repository root.

If you need to use a different Python interpreter:
```bash
cd packaging/pyinstaller
PYTHON_BIN=/path/to/python3 ./build.sh
```

## Notes
- `telemetry_sender.py` is the main sender entrypoint.
- `telemetry_receiver.py` is the main receiver entrypoint.
- Sender and receiver radio settings (`--freq`, `--bw`, `--sf`, ...) must
  match on both ends or there is no link. The defaults match each other.
