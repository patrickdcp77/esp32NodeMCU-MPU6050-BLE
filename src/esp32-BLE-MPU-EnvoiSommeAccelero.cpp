#include <Arduino.h>
#include <Wire.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLEAdvertising.h>
#include <BLE2902.h>
#include <math.h>

// DEBUG = 0: no serial output
// DEBUG = 1: serial logs enabled
#define DEBUG 0

#if DEBUG
#define DBG_BEGIN(baud) Serial.begin(baud)
#define DBG_PRINTLN(x) Serial.println(x)
#else
#define DBG_BEGIN(baud) ((void)0)
#define DBG_PRINTLN(x) ((void)0)
#endif

static const char *DEVICE_NAME = "ESP32-BLE-MPU-SUM";
static const char *SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e";
static const char *CHAR_TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e";

static const uint8_t MPU6050_ADDR_DEFAULT = 0x68;
static const uint8_t MPU6050_ADDR_ALT = 0x69;
static const uint8_t MPU6050_PWR_MGMT_1 = 0x6B;
static const uint8_t MPU6050_SMPLRT_DIV = 0x19;
static const uint8_t MPU6050_CONFIG = 0x1A;
static const uint8_t MPU6050_ACCEL_CONFIG = 0x1C;
static const uint8_t MPU6050_ACCEL_XOUT_H = 0x3B;

// 500 Hz sampling. BLE notify speed will be the practical upper bound.
static const unsigned long SAMPLE_INTERVAL_US = 2000;

BLECharacteristic *txCharacteristic = nullptr;
BLE2902 *txCccd = nullptr;
bool deviceConnected = false;
bool mpuAvailable = false;
uint8_t mpuAddress = MPU6050_ADDR_DEFAULT;
unsigned long lastSampleUs = 0;

struct MpuAccelRaw {
  int16_t ax;
  int16_t ay;
  int16_t az;
};

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) override {
    (void)pServer;
    deviceConnected = true;
    DBG_PRINTLN("BLE: client connecte");
  }

  void onDisconnect(BLEServer *pServer) override {
    deviceConnected = false;
    DBG_PRINTLN("BLE: client deconnecte, reprise advertising");
    pServer->startAdvertising();
  }
};

bool isNotifyEnabled() {
  if (!deviceConnected || txCharacteristic == nullptr || txCccd == nullptr) {
    return false;
  }
  return txCccd->getNotifications();
}

bool i2cProbe(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

bool detectMpuAddress() {
  if (i2cProbe(MPU6050_ADDR_DEFAULT)) {
    mpuAddress = MPU6050_ADDR_DEFAULT;
    return true;
  }
  if (i2cProbe(MPU6050_ADDR_ALT)) {
    mpuAddress = MPU6050_ADDR_ALT;
    return true;
  }
  return false;
}

bool mpuWriteRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(mpuAddress);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool mpuReadAccel(MpuAccelRaw &out) {
  Wire.beginTransmission(mpuAddress);
  Wire.write(MPU6050_ACCEL_XOUT_H);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  const uint8_t kReadLen = 6;
  if (Wire.requestFrom(mpuAddress, kReadLen) != kReadLen) {
    return false;
  }

  uint8_t raw[kReadLen];
  for (uint8_t i = 0; i < kReadLen; ++i) {
    raw[i] = Wire.read();
  }

  out.ax = (static_cast<int16_t>(raw[0]) << 8) | raw[1];
  out.ay = (static_cast<int16_t>(raw[2]) << 8) | raw[3];
  out.az = (static_cast<int16_t>(raw[4]) << 8) | raw[5];
  return true;
}

bool setupMpu6050() {
  if (!mpuWriteRegister(MPU6050_PWR_MGMT_1, 0x00)) {
    return false;
  }
  delay(50);

  if (!mpuWriteRegister(MPU6050_SMPLRT_DIV, 0x00)) {
    return false;
  }
  if (!mpuWriteRegister(MPU6050_CONFIG, 0x01)) {
    return false;
  }

  // +/-16g range to avoid clipping on strong impacts.
  if (!mpuWriteRegister(MPU6050_ACCEL_CONFIG, 0x18)) {
    return false;
  }

  return true;
}

void setupBle() {
  BLEDevice::init(DEVICE_NAME);
  BLEDevice::setMTU(247);

  BLEServer *server = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());

  BLEService *service = server->createService(SERVICE_UUID);
  txCharacteristic = service->createCharacteristic(
      CHAR_TX_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  txCccd = new BLE2902();
  txCharacteristic->addDescriptor(txCccd);

  service->start();

  BLEAdvertising *advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(SERVICE_UUID);
  advertising->setScanResponse(true);
  advertising->start();
}

void setup() {
  DBG_BEGIN(115200);
  delay(300);
  DBG_PRINTLN("Demarrage BLE + MPU6050 (somme accel uniquement)");

  Wire.begin(21, 22);
  Wire.setClock(400000);

  mpuAvailable = detectMpuAddress() && setupMpu6050();
  if (!mpuAvailable) {
    DBG_PRINTLN("MPU6050 absent/non initialise");
  }

  setupBle();
  DBG_PRINTLN("BLE advertising actif");
}

void loop() {
  if (!deviceConnected || txCharacteristic == nullptr || !mpuAvailable) {
    delay(10);
    return;
  }

  unsigned long nowUs = micros();
  if (static_cast<unsigned long>(nowUs - lastSampleUs) < SAMPLE_INTERVAL_US) {
    return;
  }
  lastSampleUs = nowUs;

  MpuAccelRaw data;
  if (!mpuReadAccel(data)) {
    return;
  }

  // +/-16g scale: 2048 LSB/g
  const float ax = data.ax / 2048.0f;
  const float ay = data.ay / 2048.0f;
  const float az = data.az / 2048.0f;
  const float accMagnitudeG = sqrtf((ax * ax) + (ay * ay) + (az * az));

  // Compact text payload: milli-g as integer (example: 981 for ~0.981g).
  const uint16_t accMilliG = static_cast<uint16_t>(accMagnitudeG * 1000.0f);
  char payload[8];
  snprintf(payload, sizeof(payload), "%u", static_cast<unsigned>(accMilliG));

  txCharacteristic->setValue(payload);
  if (isNotifyEnabled()) {
    txCharacteristic->notify();
  }
}
