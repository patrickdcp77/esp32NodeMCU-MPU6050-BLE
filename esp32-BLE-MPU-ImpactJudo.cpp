#include <Arduino.h>
#include <Wire.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLEAdvertising.h>
#include <BLE2902.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>

// DEBUG = 0: no serial output (max performance)
// DEBUG = 1: structured serial logs
#define DEBUG 0

#if DEBUG
#define DBG_BEGIN(baud) Serial.begin(baud)
#define DBG_PRINT(x) Serial.print(x)
#define DBG_PRINTLN(x) Serial.println(x)
#define DBG_PRINTF(...) Serial.printf(__VA_ARGS__)
#else
#define DBG_BEGIN(baud) ((void)0)
#define DBG_PRINT(x) ((void)0)
#define DBG_PRINTLN(x) ((void)0)
#define DBG_PRINTF(...) ((void)0)
#endif

// BLE Nordic UART-like UUIDs (compatibles avec nRF Connect et apps UART BLE)
static const char *DEVICE_NAME = "ESP32-BLE-MPU6050";
static const char *SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e";
static const char *CHAR_TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e";
static const char *CHAR_RX_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e";

static const uint8_t MPU6050_ADDR_DEFAULT = 0x68;
static const uint8_t MPU6050_ADDR_ALT = 0x69;
static const uint8_t MPU6050_PWR_MGMT_1 = 0x6B;
static const uint8_t MPU6050_SMPLRT_DIV = 0x19;
static const uint8_t MPU6050_CONFIG = 0x1A;
static const uint8_t MPU6050_ACCEL_CONFIG = 0x1C;
static const uint8_t MPU6050_GYRO_CONFIG = 0x1B;
static const uint8_t MPU6050_ACCEL_XOUT_H = 0x3B;

BLECharacteristic *txCharacteristic = nullptr;
BLE2902 *txCccd = nullptr;
bool deviceConnected = false;
bool mpuAvailable = false;
uint8_t mpuAddress = MPU6050_ADDR_DEFAULT;
unsigned long lastSendMs = 0;
unsigned long lastDiagMs = 0;
UBaseType_t minLoopStackWatermarkWords = UINT32_MAX;
unsigned long lastSampleUs = 0;

static const unsigned long SAMPLE_INTERVAL_US = 2000; // 500 Hz capture
static const float IMPACT_THRESHOLD_G = 2.5f;
static const unsigned long EVENT_WINDOW_MS = 250;
bool impactEventActive = false;
unsigned long eventStartMs = 0;
float peakAccG = 0.0f;
float peakGyroDps = 0.0f;
float eventAx = 0.0f;
float eventAy = 0.0f;
float eventAz = 0.0f;

struct MpuRawData {
  int16_t ax;
  int16_t ay;
  int16_t az;
  int16_t temp;
  int16_t gx;
  int16_t gy;
  int16_t gz;
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

class RxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) override {
    std::string value = pCharacteristic->getValue();
    if (!value.empty()) {
      DBG_PRINT("BLE RX: ");
      DBG_PRINTLN(value.c_str());
    }
  }
};

void debugLogLive(unsigned long nowMs,
                  float ax,
                  float ay,
                  float az,
                  float accMagnitudeG,
                  float gyroMagnitudeDps,
                  float tempC) {
#if DEBUG
  static unsigned long lastDebugLiveMs = 0;
  if (nowMs - lastDebugLiveMs < 1000) {
    return;
  }
  lastDebugLiveMs = nowMs;

  DBG_PRINTF("[LIVE] t=%lums a=(%.2f,%.2f,%.2f) |a|=%.2fg |gyro|=%.1fdps temp=%.1fC\n",
             nowMs,
             ax,
             ay,
             az,
             accMagnitudeG,
             gyroMagnitudeDps,
             tempC);
#else
  (void)nowMs;
  (void)ax;
  (void)ay;
  (void)az;
  (void)accMagnitudeG;
  (void)gyroMagnitudeDps;
  (void)tempC;
#endif
}

bool mpuWriteRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(mpuAddress);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool mpuReadBlock(uint8_t startReg, uint8_t *buffer, size_t length) {
  Wire.beginTransmission(mpuAddress);
  Wire.write(startReg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  size_t received = Wire.requestFrom(mpuAddress, static_cast<uint8_t>(length));
  if (received != length) {
    return false;
  }

  for (size_t i = 0; i < length; ++i) {
    buffer[i] = Wire.read();
  }
  return true;
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

bool setupMpu6050() {
  if (!mpuWriteRegister(MPU6050_PWR_MGMT_1, 0x00)) {
    return false;
  }
  delay(50);

  // High-rate mode for impact capture: 1 kHz internal sample, low-latency filter.
  if (!mpuWriteRegister(MPU6050_SMPLRT_DIV, 0x00)) {
    return false;
  }
  if (!mpuWriteRegister(MPU6050_CONFIG, 0x01)) {
    return false;
  }

  // Wider ranges prevent clipping during violent impacts.
  if (!mpuWriteRegister(MPU6050_ACCEL_CONFIG, 0x18)) { // +/-16g
    return false;
  }
  if (!mpuWriteRegister(MPU6050_GYRO_CONFIG, 0x18)) { // +/-2000 deg/s
    return false;
  }
  return true;
}

bool readMpu6050(MpuRawData &out) {
  uint8_t raw[14] = {0};
  if (!mpuReadBlock(MPU6050_ACCEL_XOUT_H, raw, sizeof(raw))) {
    return false;
  }

  out.ax = (static_cast<int16_t>(raw[0]) << 8) | raw[1];
  out.ay = (static_cast<int16_t>(raw[2]) << 8) | raw[3];
  out.az = (static_cast<int16_t>(raw[4]) << 8) | raw[5];
  out.temp = (static_cast<int16_t>(raw[6]) << 8) | raw[7];
  out.gx = (static_cast<int16_t>(raw[8]) << 8) | raw[9];
  out.gy = (static_cast<int16_t>(raw[10]) << 8) | raw[11];
  out.gz = (static_cast<int16_t>(raw[12]) << 8) | raw[13];
  return true;
}

void setupBle() {
  BLEDevice::init(DEVICE_NAME);
  BLEServer *server = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());

  BLEService *service = server->createService(SERVICE_UUID);

  txCharacteristic = service->createCharacteristic(
      CHAR_TX_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  txCccd = new BLE2902();
  txCharacteristic->addDescriptor(txCccd);

  BLECharacteristic *rxCharacteristic = service->createCharacteristic(
      CHAR_RX_UUID,
      BLECharacteristic::PROPERTY_WRITE);
  rxCharacteristic->setCallbacks(new RxCallbacks());

  service->start();

  BLEAdvertising *advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(SERVICE_UUID);
  advertising->setScanResponse(true);
  advertising->start();
}

bool isNotifyEnabled() {
  if (!deviceConnected || txCharacteristic == nullptr || txCccd == nullptr) {
    return false;
  }
  return txCccd->getNotifications();
}

void setup() {
  DBG_BEGIN(115200);
  delay(500);
  DBG_PRINTLN("Demarrage BLE + MPU6050");

  Wire.begin(21, 22);
  Wire.setClock(100000);

  mpuAvailable = detectMpuAddress();

  if (!mpuAvailable) {
    DBG_PRINTLN("MPU6050 absent (adresse 0x68/0x69 introuvable)");
  } else if (!setupMpu6050()) {
    DBG_PRINTLN("Erreur MPU6050: verifie cablage (VCC, GND, SDA, SCL)");
  } else {
    mpuAvailable = true;
    DBG_PRINT("MPU6050 initialise sur adresse 0x");
    DBG_PRINTLN(String(mpuAddress, HEX));
  }

  if (!mpuAvailable) {
    DBG_PRINTLN("Mode BLE seul actif (lectures MPU desactivees)");
  } else {
    DBG_PRINTLN("MPU6050 initialise");
  }

  setupBle();
  DBG_PRINTLN("BLE advertising actif");

  // Baseline memoire pour suivre une eventuelle derive/overflow runtime.
  UBaseType_t watermark = uxTaskGetStackHighWaterMark(nullptr);
  minLoopStackWatermarkWords = watermark;
  DBG_PRINTF("[BOOT] freeHeap=%uB loopStackMin=%uB\n",
             ESP.getFreeHeap(),
             static_cast<unsigned>(minLoopStackWatermarkWords * sizeof(StackType_t)));
}

void sendImpactEvent(unsigned long nowMs,
                     float accG,
                     float ax,
                     float ay,
                     float az,
                     float gyroDps) {
  char payload[96];
  snprintf(payload, sizeof(payload),
           "{\"evt\":\"impact\",\"t\":%lu,\"g\":%.2f,\"gx\":%.0f,\"ax\":%.2f,\"ay\":%.2f,\"az\":%.2f}",
           nowMs, accG, gyroDps, ax, ay, az);

  txCharacteristic->setValue(payload);
  if (isNotifyEnabled()) {
    txCharacteristic->notify();
  }
  DBG_PRINT("[IMPACT] ");
  DBG_PRINTLN(payload);
}

void loop() {
  if (!deviceConnected || txCharacteristic == nullptr) {
    delay(100);
    return;
  }

  unsigned long now = millis();
  unsigned long nowUs = micros();
  if (static_cast<unsigned long>(nowUs - lastSampleUs) < SAMPLE_INTERVAL_US) {
    return;
  }
  lastSampleUs = nowUs;

  MpuRawData data;
  if (!mpuAvailable) {
    return;
  }

  if (!readMpu6050(data)) {
    DBG_PRINTLN("Erreur lecture MPU6050 (bus I2C)");
    mpuAvailable = detectMpuAddress();
    if (!mpuAvailable) {
      DBG_PRINTLN("MPU6050 perdu: lectures suspendues");
    }
    return;
  }

  const float ax = data.ax / 2048.0f;  // +/-16g => 2048 LSB/g
  const float ay = data.ay / 2048.0f;
  const float az = data.az / 2048.0f;
  const float gx = data.gx / 16.4f;    // +/-2000 dps => 16.4 LSB/(deg/s)
  const float gy = data.gy / 16.4f;
  const float gz = data.gz / 16.4f;
  const float tempC = (data.temp / 340.0f) + 36.53f;

  const float accMagnitudeG = sqrtf((ax * ax) + (ay * ay) + (az * az));
  const float gyroMagnitudeDps = sqrtf((gx * gx) + (gy * gy) + (gz * gz));

  if (accMagnitudeG > IMPACT_THRESHOLD_G) {
    if (!impactEventActive) {
      impactEventActive = true;
      eventStartMs = now;
      peakAccG = accMagnitudeG;
      peakGyroDps = gyroMagnitudeDps;
      eventAx = ax;
      eventAy = ay;
      eventAz = az;
    } else if (accMagnitudeG > peakAccG) {
      peakAccG = accMagnitudeG;
      peakGyroDps = gyroMagnitudeDps;
      eventAx = ax;
      eventAy = ay;
      eventAz = az;
    }
  }

  if (impactEventActive && (now - eventStartMs >= EVENT_WINDOW_MS)) {
    sendImpactEvent(now, peakAccG, eventAx, eventAy, eventAz, peakGyroDps);
    impactEventActive = false;
  }

  if (now - lastSendMs >= 200) {
    char livePayload[120];
    snprintf(livePayload, sizeof(livePayload),
             "{\"a\":%.2f,%.2f,%.2f,\"g\":%.1f,\"t\":%.1f}",
             ax, ay, az, gyroMagnitudeDps, tempC);
    txCharacteristic->setValue(livePayload);
    if (isNotifyEnabled()) {
      txCharacteristic->notify();
    }
    debugLogLive(now, ax, ay, az, accMagnitudeG, gyroMagnitudeDps, tempC);
    lastSendMs = now;
  }

  if (now - lastDiagMs >= 5000) {
    UBaseType_t watermark = uxTaskGetStackHighWaterMark(nullptr);
    if (watermark < minLoopStackWatermarkWords) {
      minLoopStackWatermarkWords = watermark;
    }

    DBG_PRINTF("[DIAG] freeHeap=%uB loopStackMin=%uB\n",
               ESP.getFreeHeap(),
               static_cast<unsigned>(minLoopStackWatermarkWords * sizeof(StackType_t)));
    lastDiagMs = now;
  }
}