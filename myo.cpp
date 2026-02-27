
#include "Arduino.h"
#include "myo.h"

// ── Auto-reconnect: BLE client callbacks ─────────────────────────────────────
class MyoClientCallbacks : public BLEClientCallbacks {
  myo* m;
public:
  MyoClientCallbacks(myo* instance) : m(instance) {}
  void onConnect(BLEClient*) {}
  void onDisconnect(BLEClient*) {
    m->connected       = false;
    m->EMGNotifyDone   = false;
    m->IMUNotifyDone   = false;
    m->BATTNotifyDone  = false;
    m->PoseNotifyDone  = false;
    m->getAllDataDone   = false;
    Serial.println("BLE disconnected – reconnecting...");
  }
};

// ── Constructor ───────────────────────────────────────────────────────────────
myo::myo(const char* address) : pAddress(address) {
  memset(emg, 0, sizeof(emg));
  memset(imu_quat,  0, sizeof(imu_quat));
  memset(imu_accel, 0, sizeof(imu_accel));
  memset(imu_gyro,  0, sizeof(imu_gyro));
}

// ── connect ───────────────────────────────────────────────────────────────────
void myo::connect() {
  if(!myo::connected) {
    BLEDevice::init("");
    BLEScan* pBLEScan = BLEDevice::getScan();
    pBLEScan->setActiveScan(true);
    pBLEScan->start(10);
    myo::pClient = BLEDevice::createClient();
    myo::pClient->setClientCallbacks(new MyoClientCallbacks(this));
    if(!myo::pClient->connect(myo::pAddress)) {
      Serial.println("BLE connect failed");
      return;
    }
    myo::connected = true;
    Serial.println("BLE connected");
  }
}

// ── getAllData: enable EMG (mode 2) + IMU (mode 3) + classifier (mode 1) ─────
void myo::getAllData() {
  if(myo::connected && !myo::getAllDataDone) {
    BLEUUID tservice = BLEUUID("d5060001-a904-deb9-4748-2c7f4a124842");
    BLEUUID tcharacteristic = BLEUUID("d5060401-a904-deb9-4748-2c7f4a124842");

    BLERemoteService* pSvc = myo::pClient->getService(tservice);
    if(!pSvc) { Serial.println("getAllData: Service not found"); return; }
    BLERemoteCharacteristic* pChr = pSvc->getCharacteristic(tcharacteristic);
    if(!pChr) { Serial.println("getAllData: Characteristic not found"); return; }

    // cmd=0x01, len=3, emg=2 (send), imu=3 (send all), classifier=1 (enabled)
    uint8_t writeVal[] = {0x01, 0x03, 0x02, 0x03, 0x01};
    pChr->writeValue(writeVal, sizeof(writeVal));
    myo::getAllDataDone = true;
  }
}

// ── EMGNotify ─────────────────────────────────────────────────────────────────
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

// ── IMUNotify ─────────────────────────────────────────────────────────────────
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

// ── BATTNotify ────────────────────────────────────────────────────────────────
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

// ── PoseNotify ────────────────────────────────────────────────────────────────
void myo::PoseNotify() {
  if(myo::connected && !myo::PoseNotifyDone) {
    BLEUUID tservice = BLEUUID("d5060003-a904-deb9-4748-2c7f4a124842");
    BLEUUID tcharacteristic = BLEUUID("d5060103-a904-deb9-4748-2c7f4a124842");

    BLERemoteService* pSvc = myo::pClient->getService(tservice);
    if(!pSvc) { Serial.println("PoseNotify: Service not found"); return; }
    BLERemoteCharacteristic* pChr = pSvc->getCharacteristic(tcharacteristic);
    if(!pChr) { Serial.println("PoseNotify: Characteristic not found"); return; }

    uint8_t NotifyOn[] = {0x01};
    pChr->getDescriptor((uint16_t)0x2902)->writeValue(NotifyOn, sizeof(NotifyOn));
    myo::PoseNotifyDone = true;
  }
}

// ── vibrate ───────────────────────────────────────────────────────────────────
void myo::vibrate(uint8_t type) {
  if(!myo::connected) return;
  BLEUUID tservice = BLEUUID("d5060001-a904-deb9-4748-2c7f4a124842");
  BLEUUID tcharacteristic = BLEUUID("d5060401-a904-deb9-4748-2c7f4a124842");

  BLERemoteService* pSvc = myo::pClient->getService(tservice);
  if(!pSvc) { Serial.println("vibrate: Service not found"); return; }
  BLERemoteCharacteristic* pChr = pSvc->getCharacteristic(tcharacteristic);
  if(!pChr) { Serial.println("vibrate: Characteristic not found"); return; }

  uint8_t cmd[] = {0x03, 0x01, type};
  pChr->writeValue(cmd, sizeof(cmd));
}

// ── parseEMG ──────────────────────────────────────────────────────────────────
// Packet: 16 bytes = 2 samples x 8 channels (int8)
void myo::parseEMG(uint8_t* data, size_t len) {
  if(len < 16) return;
  for(int s = 0; s < 2; s++)
    for(int c = 0; c < 8; c++)
      emg[s][c] = (int8_t)data[s * 8 + c];
}

// ── parseIMU ──────────────────────────────────────────────────────────────────
// Packet: 20 bytes, all int16 little-endian
//   [0-7]   quaternion w,x,y,z  (divide by 16384.0 for unit quaternion)
//   [8-13]  accelerometer x,y,z (divide by 2048.0 for g)
//   [14-19] gyroscope x,y,z     (divide by 16.0 for deg/s)
void myo::parseIMU(uint8_t* data, size_t len) {
  if(len < 20) return;
  auto r16 = [&](int i) -> int16_t {
    return (int16_t)((uint16_t)data[i] | ((uint16_t)data[i + 1] << 8));
  };
  imu_quat[0]  = r16(0);
  imu_quat[1]  = r16(2);
  imu_quat[2]  = r16(4);
  imu_quat[3]  = r16(6);
  imu_accel[0] = r16(8);
  imu_accel[1] = r16(10);
  imu_accel[2] = r16(12);
  imu_gyro[0]  = r16(14);
  imu_gyro[1]  = r16(16);
  imu_gyro[2]  = r16(18);
}

// ── parsePose ─────────────────────────────────────────────────────────────────
// Classifier event packet:
//   byte[0] = event type: 0x03 = pose event
//   byte[1] = pose id: 0=Rest, 1=Fist, 2=WaveIn, 3=WaveOut, 4=FingersSpread, 5=DoubleTap
void myo::parsePose(uint8_t* data, size_t len) {
  if(len < 2) return;
  if(data[0] != 0x03) return;  // only handle pose events
  uint8_t id = data[1];
  currentPose = (id <= 5) ? (MyoPose)id : POSE_UNKNOWN;
  const char* names[] = {"Rest", "Fist", "WaveIn", "WaveOut", "FingersSpread", "DoubleTap"};
  Serial.print("Pose: ");
  Serial.println(id <= 5 ? names[id] : "Unknown");
}

// ── printCSVHeader ────────────────────────────────────────────────────────────
void myo::printCSVHeader() {
  Serial.println("label,"
    "e0s0,e1s0,e2s0,e3s0,e4s0,e5s0,e6s0,e7s0,"
    "e0s1,e1s1,e2s1,e3s1,e4s1,e5s1,e6s1,e7s1,"
    "qw,qx,qy,qz,ax,ay,az,gx,gy,gz,pose");
}

// ── printCSVLine ──────────────────────────────────────────────────────────────
// Call with label 0-9 to record a labeled training sample
void myo::printCSVLine(uint8_t label) {
  Serial.print(label);
  for(int s = 0; s < 2; s++)
    for(int c = 0; c < 8; c++) {
      Serial.print(','); Serial.print(emg[s][c]);
    }
  for(int i = 0; i < 4; i++) { Serial.print(','); Serial.print(imu_quat[i]); }
  for(int i = 0; i < 3; i++) { Serial.print(','); Serial.print(imu_accel[i]); }
  for(int i = 0; i < 3; i++) { Serial.print(','); Serial.print(imu_gyro[i]); }
  Serial.print(','); Serial.println((uint8_t)currentPose);
}

// ── getMyoInfo ────────────────────────────────────────────────────────────────
void myo::getMyoInfo() {
  if(myo::connected && !myo::getMyoInfoDone) {
    BLEUUID tservice = BLEUUID("d5060001-a904-deb9-4748-2c7f4a124842");
    BLEUUID tcharacteristic = BLEUUID("d5060101-a904-deb9-4748-2c7f4a124842");

    BLERemoteService* pSvc = myo::pClient->getService(tservice);
    if(!pSvc) { Serial.println("getMyoInfo: Service not found"); return; }
    BLERemoteCharacteristic* pChr = pSvc->getCharacteristic(tcharacteristic);
    if(!pChr) { Serial.println("getMyoInfo: Characteristic not found"); return; }

    std::string s = pChr->readValue();
    for(int i = 0; i < 6; i++) fw_serial_number[i] = s[i];
    fw_unlock_pose             = (byte)s[7] * 256 + (byte)s[6];
    fw_active_classifier_type  = s[8];
    fw_active_classifier_index = s[9];
    fw_has_custom_classifier   = s[10];
    fw_stream_indicating       = s[11];
    fw_sku                     = s[12];
    for(int i = 0; i < 8; i++) fw_reserved[i] = s[13 + i];
    myo::getMyoInfoDone = true;
  }
}

// ── getFirmwareVersion ────────────────────────────────────────────────────────
void myo::getFirmwareVersion() {
  if(myo::connected && !myo::getFirmwareVersionDone) {
    BLEUUID tservice = BLEUUID("d5060001-a904-deb9-4748-2c7f4a124842");
    BLEUUID tcharacteristic = BLEUUID("d5060201-a904-deb9-4748-2c7f4a124842");

    BLERemoteService* pSvc = myo::pClient->getService(tservice);
    if(!pSvc) { Serial.println("getFirmwareVersion: Service not found"); return; }
    BLERemoteCharacteristic* pChr = pSvc->getCharacteristic(tcharacteristic);
    if(!pChr) { Serial.println("getFirmwareVersion: Characteristic not found"); return; }

    std::string s = pChr->readValue();
    fw_major        = (byte)s[1] * 256 + (byte)s[0];
    fw_minor        = (byte)s[3] * 256 + (byte)s[2];
    fw_patch        = (byte)s[5] * 256 + (byte)s[4];
    fw_hardware_rev = (byte)s[7] * 256 + (byte)s[6];
    myo::getFirmwareVersionDone = true;
  }
}
