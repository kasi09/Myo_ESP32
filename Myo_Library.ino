#include "myo.h"
#include <BLEDevice.h>
#include <BLEClient.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <stdint.h>
#include <BLEValue.h>

myo myo;

bool FirstTime = false;

void notifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
  Serial.print("Battery: ");
  Serial.println(pData[0]);
}

// EMG data: 16 bytes = 2 samples x 8 channels (signed int8)
void notifyCallbackEMG(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
  if(length < 16) return;
  for(int sample = 0; sample < 2; sample++) {
    Serial.print("EMG["); Serial.print(sample); Serial.print("]: ");
    for(int ch = 0; ch < 8; ch++) {
      Serial.print((int8_t)pData[sample * 8 + ch]);
      if(ch < 7) Serial.print(", ");
    }
    Serial.println();
  }
}

void setup()
{
  Serial.begin(115200);
}

void loop()
{
  myo.connect();
  myo.getAllData();
  if(myo.connected && !FirstTime) {
    BLERemoteService* pBattSvc = myo.pClient->getService(BLEUUID("0000180f-0000-1000-8000-00805f9b34fb"));
    if(pBattSvc) {
      BLERemoteCharacteristic* pBattChr = pBattSvc->getCharacteristic(BLEUUID("00002a19-0000-1000-8000-00805f9b34fb"));
      if(pBattChr) pBattChr->registerForNotify(notifyCallback);
    }

    BLERemoteService* pEmgSvc = myo.pClient->getService(BLEUUID("d5060005-a904-deb9-4748-2c7f4a124842"));
    if(pEmgSvc) {
      BLERemoteCharacteristic* pEmgChr = pEmgSvc->getCharacteristic(BLEUUID("d5060105-a904-deb9-4748-2c7f4a124842"));
      if(pEmgChr) pEmgChr->registerForNotify(notifyCallbackEMG);
    }

    FirstTime = true;
  }
  myo.EMGNotify();
  myo.BATTNotify();
  /*
  if(myo.connected && myo.initDo) {
    myo.getFirmwareVersion();

    Serial.println(myo.fw_major);
    Serial.println(myo.fw_minor);
    Serial.println(myo.fw_patch);
    Serial.println(myo.fw_hardware_rev);

    myo.initDo = false;
  }
  */
  /*
  if(myo.connected && myo.initDo) {
    myo.getMyoInfo();

    for(int i = 0; i < 6; i++) Serial.print(myo.fw_serial_number[i]);
    Serial.println();
    Serial.println(myo.fw_unlock_pose);
    Serial.println(myo.fw_active_classifier_type);
    Serial.println(myo.fw_active_classifier_index);
    Serial.println(myo.fw_has_custom_classifier);
    Serial.println(myo.fw_stream_indicating);
    Serial.println(myo.fw_sku);

    myo.initDo = false;
  }
  */
  delay(1000);
}
