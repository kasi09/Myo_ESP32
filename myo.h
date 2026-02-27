
#ifndef myo_h
#define myo_h

#include "Arduino.h"
#include <BLEDevice.h>
#include <BLEClient.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <stdint.h>
#include <BLEValue.h>
#include <BLEAddress.h>

class myo
{
  public:
    myo(const char* address = "f9:64:aa:5e:d8:ef");
    void getMyoInfo();
    void getFirmwareVersion();
    void connect();
    void EMGNotify();
    void IMUNotify();
    void BATTNotify();
    void getAllData();

    BLEAddress pAddress;
    BLEClient* pClient;
    
    int temp; 

    bool initDo                 = true;
    bool getMyoInfoDone         = false;
    bool getFirmwareVersionDone = false;
    bool connected              = false;
    bool EMGNotifyDone          = false;
    bool IMUNotifyDone          = false;
    bool BATTNotifyDone         = false;
    bool getAllDataDone         = false;
    
    uint16_t fw_major;
    uint16_t fw_minor;
    uint16_t fw_patch;
    uint16_t fw_hardware_rev;
    
    uint8_t fw_serial_number[6];
    uint16_t fw_unlock_pose;
    uint8_t fw_active_classifier_type;
    uint8_t fw_active_classifier_index;
    uint8_t fw_has_custom_classifier;
    uint8_t fw_stream_indicating;
    uint8_t fw_sku;
    uint8_t fw_reserved[8];

    
  private:
};

#endif
