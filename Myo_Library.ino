#include "myo.h"
#include <BLEDevice.h>
#include <BLEClient.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <stdint.h>
#include <BLEValue.h>

myo myo;

bool FirstTime = 0;

void notifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
  Serial.print("Battery: ");
  Serial.println(pData[0]);
}

void notifyCallback0(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
  int k;
  for(k=0; k<=length; k++) {
    Serial.print("0Value ");
    Serial.print(k);
    Serial.print(" : ");
    Serial.println(pData[k]);
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
  if(!FirstTime) {
    myo.pClient->getService(BLEUUID("0000180f-0000-1000-8000-00805f9b34fb"))->getCharacteristic(BLEUUID("00002a19-0000-1000-8000-00805f9b34fb"))->registerForNotify(notifyCallback); 
    //myo.pClient->getService(BLEUUID("d5060005-a904-deb9-4748-2c7f4a124842"))->getCharacteristic(BLEUUID("d5060105-a904-deb9-4748-2c7f4a124842"))->registerForNotify(notifyCallback0); 
    FirstTime = 1;     
  }  
  //myo.EMGNotify();
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

    Serial.print(myo.fw_serial_number[0]);
    Serial.print(myo.fw_serial_number[1]);
    Serial.print(myo.fw_serial_number[2]);
    Serial.print(myo.fw_serial_number[3]);
    Serial.print(myo.fw_serial_number[4]);
    Serial.print(myo.fw_serial_number[5]);
    Serial.println(myo.fw_serial_number[6]);
    Serial.println(myo.fw_unlock_pose);
    Serial.println(myo.fw_active_classifier_type);
    Serial.println(myo.fw_active_classifier_index);
    Serial.println(myo.fw_has_custom_classifier);
    Serial.println(myo.fw_stream_indicating);
    Serial.println(myo.fw_sku);
    Serial.print(myo.fw_reserved[0]);
    Serial.print(myo.fw_reserved[1]);
    Serial.print(myo.fw_reserved[2]);
    Serial.print(myo.fw_reserved[3]);
    Serial.print(myo.fw_reserved[4]);
    Serial.print(myo.fw_reserved[5]);
    Serial.print(myo.fw_reserved[6]);
    Serial.println(myo.fw_reserved[7]);
    
    myo.initDo = false;
  }
  */
  //Serial.println("...");
  delay(1000);
}
