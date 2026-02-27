#include "myo.h"
#include <BLEDevice.h>
#include <BLEClient.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <stdint.h>
#include <BLEValue.h>

// ── Change this to your Myo's BLE address ────────────────────────────────────
myo myo("f9:64:aa:5e:d8:ef");

bool notifyRegistered = false;

// ── BLE Callbacks ─────────────────────────────────────────────────────────────

void batteryCallback(BLERemoteCharacteristic*, uint8_t* pData, size_t, bool) {
  Serial.print("Battery: ");
  Serial.print(pData[0]);
  Serial.println("%");
}

// EMG: 16 bytes = 2 samples x 8 channels (int8)
void emgCallback(BLERemoteCharacteristic*, uint8_t* pData, size_t length, bool) {
  myo.parseEMG(pData, length);
  // Uncomment to print live EMG:
  // for(int s = 0; s < 2; s++) {
  //   Serial.print("EMG["); Serial.print(s); Serial.print("]: ");
  //   for(int c = 0; c < 8; c++) {
  //     Serial.print(myo.emg[s][c]);
  //     if(c < 7) Serial.print(", ");
  //   }
  //   Serial.println();
  // }
}

// IMU: 20 bytes – quaternion, accelerometer, gyroscope (all int16 little-endian)
void imuCallback(BLERemoteCharacteristic*, uint8_t* pData, size_t length, bool) {
  myo.parseIMU(pData, length);
  // Uncomment to print live IMU (scale factors: quat/16384, accel/2048g, gyro/16 deg/s):
  // Serial.printf("Q: %d %d %d %d  A: %d %d %d  G: %d %d %d\n",
  //   myo.imu_quat[0], myo.imu_quat[1], myo.imu_quat[2], myo.imu_quat[3],
  //   myo.imu_accel[0], myo.imu_accel[1], myo.imu_accel[2],
  //   myo.imu_gyro[0],  myo.imu_gyro[1],  myo.imu_gyro[2]);
}

// Classifier events: pose detection
void poseCallback(BLERemoteCharacteristic*, uint8_t* pData, size_t length, bool) {
  myo.parsePose(pData, length);
}

// ── Register all callbacks once after connection ──────────────────────────────
void registerCallbacks() {
  BLERemoteService* pSvc;
  BLERemoteCharacteristic* pChr;

  // Battery
  pSvc = myo.pClient->getService(BLEUUID("0000180f-0000-1000-8000-00805f9b34fb"));
  if(pSvc) {
    pChr = pSvc->getCharacteristic(BLEUUID("00002a19-0000-1000-8000-00805f9b34fb"));
    if(pChr) pChr->registerForNotify(batteryCallback);
  }

  // EMG (4 characteristics, we use the first one; all carry the same interleaved stream)
  pSvc = myo.pClient->getService(BLEUUID("d5060005-a904-deb9-4748-2c7f4a124842"));
  if(pSvc) {
    const char* emgUUIDs[4] = {
      "d5060105-a904-deb9-4748-2c7f4a124842",
      "d5060205-a904-deb9-4748-2c7f4a124842",
      "d5060305-a904-deb9-4748-2c7f4a124842",
      "d5060405-a904-deb9-4748-2c7f4a124842"
    };
    for(int i = 0; i < 4; i++) {
      pChr = pSvc->getCharacteristic(BLEUUID(emgUUIDs[i]));
      if(pChr) pChr->registerForNotify(emgCallback);
    }
  }

  // IMU
  pSvc = myo.pClient->getService(BLEUUID("d5060002-a904-deb9-4748-2c7f4a124842"));
  if(pSvc) {
    pChr = pSvc->getCharacteristic(BLEUUID("d5060402-a904-deb9-4748-2c7f4a124842"));
    if(pChr) pChr->registerForNotify(imuCallback);
  }

  // Pose / Classifier events
  pSvc = myo.pClient->getService(BLEUUID("d5060003-a904-deb9-4748-2c7f4a124842"));
  if(pSvc) {
    pChr = pSvc->getCharacteristic(BLEUUID("d5060103-a904-deb9-4748-2c7f4a124842"));
    if(pChr) pChr->registerForNotify(poseCallback);
  }

  notifyRegistered = true;
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial.println("Myo ESP32 ready.");
  Serial.println("Serial commands:");
  Serial.println("  h      – print CSV header");
  Serial.println("  0-9    – record labeled CSV sample");
  Serial.println("  s/m/l  – vibrate short/medium/long");
}

// ── Loop ──────────────────────────────────────────────────────────────────────
void loop() {
  myo.connect();

  if(myo.connected) {
    myo.getAllData();

    if(!notifyRegistered) {
      registerCallbacks();
    }

    myo.EMGNotify();
    myo.IMUNotify();
    myo.BATTNotify();
    myo.PoseNotify();

    // Serial command handling
    if(Serial.available()) {
      char cmd = Serial.read();
      if(cmd >= '0' && cmd <= '9') {
        myo.printCSVLine(cmd - '0');
      } else if(cmd == 'h') {
        myo.printCSVHeader();
      } else if(cmd == 's') {
        myo.vibrate(1);  // short
      } else if(cmd == 'm') {
        myo.vibrate(2);  // medium
      } else if(cmd == 'l') {
        myo.vibrate(3);  // long
      }
    }
  }

  delay(10);
}
