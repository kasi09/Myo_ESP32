
#include "Arduino.h"
#include "myo.h"

myo::myo(const char* address) : pAddress(address) {}

void myo::getMyoInfo() {
  if(myo::connected && !myo::getMyoInfoDone) {
    BLEUUID tservice = BLEUUID("d5060001-a904-deb9-4748-2c7f4a124842");
    BLEUUID tcharacteristic = BLEUUID("d5060101-a904-deb9-4748-2c7f4a124842");

    BLERemoteService* pSvc = myo::pClient->getService(tservice);
    if(!pSvc) { Serial.println("getMyoInfo: Service not found"); return; }
    BLERemoteCharacteristic* pChr = pSvc->getCharacteristic(tcharacteristic);
    if(!pChr) { Serial.println("getMyoInfo: Characteristic not found"); return; }

    std::string stringt = pChr->readValue();
    // Serial number is 6 bytes (indices 0-5)
    myo::fw_serial_number[0]        = stringt[0];
    myo::fw_serial_number[1]        = stringt[1];
    myo::fw_serial_number[2]        = stringt[2];
    myo::fw_serial_number[3]        = stringt[3];
    myo::fw_serial_number[4]        = stringt[4];
    myo::fw_serial_number[5]        = stringt[5];
    myo::fw_unlock_pose             = (byte)stringt[7] * 256 + (byte)stringt[6];
    myo::fw_active_classifier_type  = stringt[8];
    myo::fw_active_classifier_index = stringt[9];
    myo::fw_has_custom_classifier   = stringt[10];
    myo::fw_stream_indicating       = stringt[11];
    myo::fw_sku                     = stringt[12];
    myo::fw_reserved[0]             = stringt[13];
    myo::fw_reserved[1]             = stringt[14];
    myo::fw_reserved[2]             = stringt[15];
    myo::fw_reserved[3]             = stringt[16];
    myo::fw_reserved[4]             = stringt[17];
    myo::fw_reserved[5]             = stringt[18];
    myo::fw_reserved[6]             = stringt[19];
    myo::fw_reserved[7]             = stringt[20];
    myo::getMyoInfoDone             = true;
  }
}

void myo::getFirmwareVersion() {
  if(myo::connected && !myo::getFirmwareVersionDone) {
    BLEUUID tservice = BLEUUID("d5060001-a904-deb9-4748-2c7f4a124842");
    BLEUUID tcharacteristic = BLEUUID("d5060201-a904-deb9-4748-2c7f4a124842");

    BLERemoteService* pSvc = myo::pClient->getService(tservice);
    if(!pSvc) { Serial.println("getFirmwareVersion: Service not found"); return; }
    BLERemoteCharacteristic* pChr = pSvc->getCharacteristic(tcharacteristic);
    if(!pChr) { Serial.println("getFirmwareVersion: Characteristic not found"); return; }

    std::string stringt = pChr->readValue();
    myo::fw_major               = (byte)stringt[1] * 256 + (byte)stringt[0];
    myo::fw_minor               = (byte)stringt[3] * 256 + (byte)stringt[2];
    myo::fw_patch               = (byte)stringt[5] * 256 + (byte)stringt[4];
    myo::fw_hardware_rev        = (byte)stringt[7] * 256 + (byte)stringt[6];
    myo::getFirmwareVersionDone = true;
  }
}

void myo::connect() {
  if(!myo::connected) {
    BLEDevice::init("");
    BLEScan* pBLEScan = BLEDevice::getScan();
    pBLEScan->setActiveScan(true);
    pBLEScan->start(10);
    myo::pClient = BLEDevice::createClient();
    if(!myo::pClient->connect(myo::pAddress)) {
      Serial.println("BLE connect failed");
      return;
    }
    myo::connected = true;
    Serial.println("BLE connected");
  }
}

void myo::EMGNotify() {
  if(myo::connected && !myo::EMGNotifyDone) {
    BLEUUID tservice = BLEUUID("d5060005-a904-deb9-4748-2c7f4a124842");
    BLERemoteService* pSvc = myo::pClient->getService(tservice);
    if(!pSvc) { Serial.println("EMGNotify: Service not found"); return; }

    BLEUUID uuids[4] = {
      BLEUUID("d5060105-a904-deb9-4748-2c7f4a124842"),
      BLEUUID("d5060205-a904-deb9-4748-2c7f4a124842"),
      BLEUUID("d5060305-a904-deb9-4748-2c7f4a124842"),
      BLEUUID("d5060405-a904-deb9-4748-2c7f4a124842")
    };
    uint8_t NotifyOn[] = {0x01};
    for(int i = 0; i < 4; i++) {
      BLERemoteCharacteristic* pChr = pSvc->getCharacteristic(uuids[i]);
      if(!pChr) { Serial.print("EMGNotify: Characteristic "); Serial.print(i); Serial.println(" not found"); return; }
      pChr->getDescriptor((uint16_t)0x2902)->writeValue(NotifyOn, sizeof(NotifyOn));
    }
    myo::EMGNotifyDone = true;
  }
}

void myo::IMUNotify() {
  if(myo::connected && !myo::IMUNotifyDone) {
    BLEUUID tservice = BLEUUID("d5060002-a904-deb9-4748-2c7f4a124842");
    BLEUUID tcharacteristic = BLEUUID("d5060402-a904-deb9-4748-2c7f4a124842");

    BLERemoteService* pSvc = myo::pClient->getService(tservice);
    if(!pSvc) { Serial.println("IMUNotify: Service not found"); return; }
    BLERemoteCharacteristic* pChr = pSvc->getCharacteristic(tcharacteristic);
    if(!pChr) { Serial.println("IMUNotify: Characteristic not found"); return; }

    uint8_t NotifyOn[] = {0x01};
    pChr->getDescriptor((uint16_t)0x2902)->writeValue(NotifyOn, sizeof(NotifyOn));
    myo::IMUNotifyDone = true;
  }
}

void myo::BATTNotify() {
  if(myo::connected && !myo::BATTNotifyDone) {
    BLEUUID tservice = BLEUUID("0000180f-0000-1000-8000-00805f9b34fb");
    BLEUUID tcharacteristic = BLEUUID("00002a19-0000-1000-8000-00805f9b34fb");

    BLERemoteService* pSvc = myo::pClient->getService(tservice);
    if(!pSvc) { Serial.println("BATTNotify: Service not found"); return; }
    BLERemoteCharacteristic* pChr = pSvc->getCharacteristic(tcharacteristic);
    if(!pChr) { Serial.println("BATTNotify: Characteristic not found"); return; }

    uint8_t NotifyOn[] = {0x01};
    pChr->getDescriptor((uint16_t)0x2902)->writeValue(NotifyOn, sizeof(NotifyOn));
    myo::BATTNotifyDone = true;
  }
}

void myo::getAllData() {
  if(myo::connected && !myo::getAllDataDone) {
    BLEUUID tservice = BLEUUID("d5060001-a904-deb9-4748-2c7f4a124842");
    BLEUUID tcharacteristic = BLEUUID("d5060401-a904-deb9-4748-2c7f4a124842");

    BLERemoteService* pSvc = myo::pClient->getService(tservice);
    if(!pSvc) { Serial.println("getAllData: Service not found"); return; }
    BLERemoteCharacteristic* pChr = pSvc->getCharacteristic(tcharacteristic);
    if(!pChr) { Serial.println("getAllData: Characteristic not found"); return; }

    uint8_t writeVal[] = {0x01, 0x03, 0x02, 0x00, 0x01};
    pChr->writeValue(writeVal, sizeof(writeVal));
    myo::getAllDataDone = true;
  }
}
