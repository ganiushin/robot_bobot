# ESP32 MicroPython BLE + Web Bluetooth Loader

This project lets you connect to an ESP32 running MicroPython from a browser via Web Bluetooth (BLE), run code, list/remove files, and upload files to the board.

## Structure

- `firmware/main.py` — MicroPython firmware script that exposes a Nordic UART BLE service and a simple JSON protocol.
- `web/index.html` — Web Bluetooth client that can connect, run code, list files, remove files, and upload files.
- `tools/https_dev_server.py` — Minimal HTTPS server for local testing (required by Web Bluetooth).

## Flash firmware script to ESP32

1) Connect ESP32 over USB and copy `firmware/main.py` to the device (e.g., using mpremote or Thonny).
2) Reboot ESP32. It will advertise as `MPY-ESP32-BLE` and include the Nordic UART service UUID in scan response.

## Run web client (Chrome/Edge)

Web Bluetooth requires HTTPS or localhost. Use the provided dev server and a self-signed cert.

1) Generate a self-signed certificate (one-time):

```bash
cd tools
openssl req -x509 -newkey rsa:2048 -nodes -sha256 -days 365 \
  -keyout dev.key.pem -out dev.cert.pem -subj '/CN=localhost'
```

2) Start HTTPS server (serves the `web` folder):

```bash
python3 tools/https_dev_server.py 8443 web
```

3) Open `https://localhost:8443/` in Chrome/Edge. Accept the self-signed certificate warning if prompted.

4) Click "Connect", pick your ESP32, then use:
   - Run code: enter Python code (e.g., `print("hello")`) and press "Run code".
   - Upload file: choose a file and target path (e.g., `/main.py`) and click "Upload file".
   - List files: enter a path (e.g., `/`) and click "List".
   - Remove file: enter a path and click "Remove".

## Protocol

- `run`: `{op:"run", code_b64:"..."}` — code is UTF-8 base64; output arrives as `{op:"out", s:"..."}`; errors as `{op:"err", s:"..."}`; completion `{op:"ack", phase:"run", ok:true}`.
- `put`/`data`/`end`: upload pipeline with base64 chunks.
- `ls`, `rm` — filesystem helpers.

## Camera Support

The project includes support for OV2640 camera on ESP32-S3 with pin configuration for CAMERA_MODEL_ESP32S3_CAR.

**Important**: The `camera` module may not be available in standard MicroPython firmware. See `CAMERA_SETUP.md` for instructions on how to enable camera support.

### Camera Features
- Initialize camera with configurable resolution
- Capture images and receive via BLE
- Collect dataset for machine learning
- Train models directly in browser using TensorFlow.js
- Classify images with trained models

## Notes

- BLE bandwidth is limited; large files are chunked at 1 KiB per JSON message; each JSON is split into ~180B ATT writes.
- If Chrome cannot see the device, ensure HTTPS, permissions, and that ESP32 advertises the service UUID (this firmware does).
- On first connect attempt after reboot, if you only see pings, try running code again; pings are sent every 5s to keep the link active.
- Camera module requires MicroPython firmware with camera support enabled. See `CAMERA_SETUP.md` for details.


