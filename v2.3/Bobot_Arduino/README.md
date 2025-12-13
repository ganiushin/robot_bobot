# Bobot v2.3 Firmware

## Features
- **ESP32-S3** based robot firmware.
- **Web Interface** with BLE communication.
- **Camera Streaming** (160x120 Grayscale/JPEG).
- **On-Device AI Inference** (MLP Neural Network).
- **Dynamic Model Architecture**:
  - Input: 160x120 (19200 features).
  - Hidden Layer: Configurable (16, 32, 64 units).
  - Output: Dynamic classes (based on training).
- **OTA Model Upload**:
  - Int8 Quantized weights for fast transfer.
  - Protocol: `CONFIG_MODEL` -> `UL_START` -> Binary Chunks.
- **Blockly Integration**: Visual programming for robot logic.

## Hardware
- ESP32-S3 (Freenove or similar).
- OV3660 Camera.
- 2x DC Motors (DRV8833 or L298N).

## Usage
1. Upload `Bobot_Arduino.ino` to ESP32-S3 (Partition Scheme: Huge APP + PSRAM Enabled).
2. Open `web/index.html` in a BLE-supported browser (Chrome/Edge).
3. Connect, Train, and Upload Model.

