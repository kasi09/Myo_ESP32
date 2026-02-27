# Myo_ESP32

ESP32 Arduino library for connecting to a **Myo armband** via Bluetooth Low Energy (BLE).
Enables real-time streaming of EMG (electromyography) and IMU sensor data.

## Features

- BLE connection to Myo armband
- EMG data streaming (8 channels, 2 samples per notification packet)
- IMU data notifications (accelerometer / gyroscope)
- Battery level monitoring
- Configurable BLE device address via constructor

## Hardware Requirements

- ESP32 development board
- Myo armband (Thalmic Labs)

## Dependencies

Install via Arduino IDE Board Manager / Library Manager:

- [ESP32 Arduino Core](https://github.com/espressif/arduino-esp32)
- BLEDevice, BLEClient, BLEUtils, BLEScan (included in ESP32 core)

## Usage

```cpp
#include "myo.h"

// Use default address or pass your Myo's BLE address
myo myo("f9:64:aa:5e:d8:ef");

void setup() {
  Serial.begin(115200);
}

void loop() {
  myo.connect();
  myo.getAllData();   // start sensor streaming
  myo.EMGNotify();   // enable EMG notifications
  myo.BATTNotify();  // enable battery notifications
  delay(1000);
}
```

The EMG callback outputs 2 samples × 8 channels per packet:

```
EMG[0]: 12, -3, 0, 8, 5, -1, 22, -7
EMG[1]:  9,  2, 1, 6, 3,  4, 18, -4
```

## API

| Method | Description |
|---|---|
| `myo(address)` | Constructor — optional BLE MAC address string |
| `connect()` | Scan and connect to the Myo armband |
| `getAllData()` | Send command to start all sensor streaming |
| `EMGNotify()` | Enable EMG data notifications |
| `IMUNotify()` | Enable IMU data notifications |
| `BATTNotify()` | Enable battery level notifications |
| `getFirmwareVersion()` | Read firmware version into `fw_major/minor/patch` |
| `getMyoInfo()` | Read device info into `fw_serial_number`, `fw_sku`, etc. |

## BLE UUIDs

| Service | UUID |
|---|---|
| Control / Info | `d5060001-a904-deb9-4748-2c7f4a124842` |
| IMU | `d5060002-a904-deb9-4748-2c7f4a124842` |
| EMG | `d5060005-a904-deb9-4748-2c7f4a124842` |
| Battery (standard) | `0000180f-0000-1000-8000-00805f9b34fb` |

## Changelog

### 2024 – Bug fixes & improvements
- Fixed buffer overflow in `fw_serial_number` (serial number is 6 bytes, not 7)
- Fixed `fw_reserved` array size (8 entries)
- Fixed off-by-one in EMG raw-byte debug loop (`<` instead of `<=`)
- Added null-checks for all BLE service/characteristic lookups
- `connect()` now checks return value and only sets `connected = true` on success
- BLE device address is now configurable via constructor (no recompile needed)
- EMG notification callback implemented with correct 8-channel signed-int parsing
- Corrected `getMyoInfo()` field byte offsets to match Myo protocol
