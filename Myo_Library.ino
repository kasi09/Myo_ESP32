#include "myo.h"
#include <BLEDevice.h>
#include <BLEClient.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <stdint.h>
#include <BLEValue.h>

// ── Choose connection mode ────────────────────────────────────────────────────
// Option A – auto-scan: finds any Myo by BLE name (no address needed)
myo myo;
// Option B – fixed address: faster reconnect if you know the MAC
// myo myo("f9:64:aa:5e:d8:ef");

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
  // Uncomment to print live Euler angles (roll/pitch/yaw in degrees):
  // float roll, pitch, yaw;
  // myo.getEuler(roll, pitch, yaw);
  // Serial.printf("Roll: %.1f  Pitch: %.1f  Yaw: %.1f\n", roll, pitch, yaw);
}

// Classifier events: pose detection, arm sync, lock/unlock
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

  // EMG (4 characteristics, each carries 2 samples of 8 channels)
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
  Serial.println("  u      – unlock (hold)");
}

// ── Loop ──────────────────────────────────────────────────────────────────────
void loop() {
  myo.connect();

  if(myo.connected) {
    myo.getAllData();
    myo.unlock();          // hold-unlock: prevents auto-lock after inactivity

    if(!notifyRegistered) {
      registerCallbacks();
      notifyRegistered = true;
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
      } else if(cmd == 'u') {
        myo.unlockDone = false;
        myo.unlock();
      }
    }
  }

  delay(10);
}
