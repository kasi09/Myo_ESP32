
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

enum MyoPose {
  POSE_REST           = 0,
  POSE_FIST           = 1,
  POSE_WAVE_IN        = 2,
  POSE_WAVE_OUT       = 3,
  POSE_FINGERS_SPREAD = 4,
  POSE_DOUBLE_TAP     = 5,
  POSE_UNKNOWN        = 255
};

class myo
{
  public:
    myo();                              // auto-scan: find Myo by BLE name
    myo(const char* address);           // connect to fixed MAC address

    // BLE setup
    void connect();
    void getAllData();     // start all sensor streaming

    // Notifications (call once after connect)
    void EMGNotify();
    void IMUNotify();
    void BATTNotify();
    void PoseNotify();

    // Device info
    void getMyoInfo();
    void getFirmwareVersion();

    // Commands
    void vibrate(uint8_t type);   // 1=short, 2=medium, 3=long
    void unlock();                // hold-unlock (prevents auto-lock)
    void lock();

    // Data parsing (call from BLE callbacks in .ino)
    void parseEMG(uint8_t* data, size_t len);
    void parseIMU(uint8_t* data, size_t len);
    void parsePose(uint8_t* data, size_t len);

    // Computed orientation
    void getEuler(float &roll, float &pitch, float &yaw);

    // AI training data output
    void printCSVHeader();
    void printCSVLine(uint8_t label);

    // BLE handles
    BLEAddress pAddress;
    BLEClient* pClient;

    // State flags
    bool connected              = false;
    bool autoScan               = false;
    bool initDo                 = true;
    bool getMyoInfoDone         = false;
    bool getFirmwareVersionDone = false;
    bool EMGNotifyDone          = false;
    bool IMUNotifyDone          = false;
    bool BATTNotifyDone         = false;
    bool PoseNotifyDone         = false;
    bool getAllDataDone          = false;
    bool unlockDone             = false;

    // Arm state (from classifier sync event)
    bool    armSynced = false;
    uint8_t arm       = 0;   // 0 = right, 1 = left

    // Firmware info
    uint16_t fw_major;
    uint16_t fw_minor;
    uint16_t fw_patch;
    uint16_t fw_hardware_rev;
    uint8_t  fw_serial_number[6];
    uint16_t fw_unlock_pose;
    uint8_t  fw_active_classifier_type;
    uint8_t  fw_active_classifier_index;
    uint8_t  fw_has_custom_classifier;
    uint8_t  fw_stream_indicating;
    uint8_t  fw_sku;
    uint8_t  fw_reserved[8];

    // Sensor data (updated by parse* methods)
    int8_t   emg[2][8];    // 2 samples x 8 channels
    int16_t  imu_quat[4];  // w, x, y, z  (scale: /16384.0 -> unit quaternion)
    int16_t  imu_accel[3]; // x, y, z     (scale: /2048.0  -> g)
    int16_t  imu_gyro[3];  // x, y, z     (scale: /16.0    -> deg/s)
    MyoPose  currentPose   = POSE_UNKNOWN;

    int temp;

  private:
};

#endif
