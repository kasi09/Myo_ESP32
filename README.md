# Myo_ESP32

ESP32 Arduino library for connecting to a **Myo armband** via Bluetooth Low Energy (BLE).
Enables real-time streaming of EMG (electromyography) and IMU sensor data.

## Features

- BLE connection to Myo armband with **auto-reconnect**
- **EMG** data streaming (8 channels, 2 samples per notification packet)
- **IMU** data parsing — Quaternion, Accelerometer, Gyroscope (int16 raw + scale factors)
- **Pose / Gesture detection** — Rest, Fist, WaveIn, WaveOut, FingersSpread, DoubleTap
- **Vibration** control (short / medium / long)
- Battery level monitoring
- **CSV output for AI/ML training** — labeled samples via Serial command
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
  myo.connect();           // auto-reconnect if disconnected
  myo.getAllData();        // start EMG + IMU + classifier streaming
  myo.EMGNotify();
  myo.IMUNotify();
  myo.BATTNotify();
  myo.PoseNotify();

  // Serial commands: 0-9 = record labeled CSV sample, h = header, s/m/l = vibrate
  if(Serial.available()) {
    char cmd = Serial.read();
    if(cmd >= '0' && cmd <= '9') myo.printCSVLine(cmd - '0');
    else if(cmd == 'h')          myo.printCSVHeader();
    else if(cmd == 's')          myo.vibrate(1);
    else if(cmd == 'm')          myo.vibrate(2);
    else if(cmd == 'l')          myo.vibrate(3);
  }
  delay(10);
}
```

### IMU scale factors

| Data | Raw unit | Scale | Physical unit |
|---|---|---|---|
| Quaternion (w,x,y,z) | int16 | / 16384.0 | unit quaternion |
| Accelerometer (x,y,z) | int16 | / 2048.0 | g (9.81 m/s²) |
| Gyroscope (x,y,z) | int16 | / 16.0 | deg/s |

### AI training data collection

Send the CSV header once, then type `0`–`9` over Serial Monitor to record labeled samples:

```
h          → label,e0s0,...,e7s1,qw,qx,qy,qz,ax,ay,az,gx,gy,gz,pose
0          → 0,12,-3,0,8,5,-1,22,-7,9,2,1,6,3,4,18,-4,16100,...,0
1          → 1,45,30,...
```

Copy the output into a `.csv` file for training with scikit-learn, TensorFlow, etc.

## API

### Setup & connection

| Method | Description |
|---|---|
| `myo(address)` | Constructor — optional BLE MAC address string |
| `connect()` | Connect to Myo; auto-reconnect after disconnect |
| `getAllData()` | Set mode: EMG stream + IMU all + classifier enabled |

### Notifications (call once after connect)

| Method | Description |
|---|---|
| `EMGNotify()` | Enable EMG notifications (4 characteristics) |
| `IMUNotify()` | Enable IMU data notifications |
| `BATTNotify()` | Enable battery level notifications |
| `PoseNotify()` | Enable classifier / pose event notifications |

### Data parsing (call from BLE callbacks)

| Method | Description |
|---|---|
| `parseEMG(data, len)` | Parse 16-byte EMG packet → `emg[2][8]` |
| `parseIMU(data, len)` | Parse 20-byte IMU packet → `imu_quat/accel/gyro` |
| `parsePose(data, len)` | Parse classifier event → `currentPose`, prints to Serial |

### Commands

| Method | Description |
|---|---|
| `vibrate(type)` | Vibrate: 1=short, 2=medium, 3=long |
| `printCSVHeader()` | Print CSV column names to Serial |
| `printCSVLine(label)` | Print one labeled training sample to Serial |

### Device info

| Method | Description |
|---|---|
| `getFirmwareVersion()` | Read into `fw_major`, `fw_minor`, `fw_patch` |
| `getMyoInfo()` | Read into `fw_serial_number`, `fw_sku`, etc. |

### Sensor data (public members, updated by parse* methods)

```cpp
int8_t  emg[2][8];    // 2 samples x 8 channels
int16_t imu_quat[4];  // w, x, y, z
int16_t imu_accel[3]; // x, y, z
int16_t imu_gyro[3];  // x, y, z
MyoPose currentPose;  // REST, FIST, WAVE_IN, WAVE_OUT, FINGERS_SPREAD, DOUBLE_TAP
```

## BLE UUIDs

| Service | UUID |
|---|---|
| Control / Info / Command | `d5060001-a904-deb9-4748-2c7f4a124842` |
| IMU | `d5060002-a904-deb9-4748-2c7f4a124842` |
| Classifier / Pose | `d5060003-a904-deb9-4748-2c7f4a124842` |
| EMG | `d5060005-a904-deb9-4748-2c7f4a124842` |
| Battery (standard BLE) | `0000180f-0000-1000-8000-00805f9b34fb` |

## Changelog

### 2024 – Extensions
- IMU data fully parsed: Quaternion, Accelerometer, Gyroscope (int16 raw, documented scale factors)
- Pose / gesture detection via classifier events (Fist, WaveIn/Out, FingersSpread, DoubleTap, Rest)
- Vibration command: `vibrate(1/2/3)` for short / medium / long
- Auto-reconnect on BLE disconnect via `BLEClientCallbacks`
- CSV output for AI/ML training data collection (`printCSVHeader()` / `printCSVLine(label)`)
- Fixed `getAllData()` mode command: IMU mode was 0 (off), corrected to 3 (send all)
- All 4 EMG characteristics now registered with callbacks
- Serial command interface for vibration and CSV recording

### 2024 – Bug fixes
- Fixed buffer overflow in `fw_serial_number` (serial number is 6 bytes, not 7)
- Fixed `fw_reserved` array size (8 entries)
- Fixed off-by-one in EMG raw-byte debug loop (`<` instead of `<=`)
- Added null-checks for all BLE service/characteristic lookups
- `connect()` now checks return value and only sets `connected = true` on success
- BLE device address is now configurable via constructor (no recompile needed)
- EMG notification callback implemented with correct 8-channel signed-int parsing
- Corrected `getMyoInfo()` field byte offsets to match Myo protocol
