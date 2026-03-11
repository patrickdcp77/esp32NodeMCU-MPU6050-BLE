#include <Arduino.h>
#include <Wire.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLEAdvertising.h>
#include <BLE2902.h>
#include <DHT.h>
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

// GPIO MAP (grouped in one place)
// - I2C MPU6050: SDA=GPIO21, SCL=GPIO22
// - DHT22 data: GPIO4
// - Hardware ID bits: GPIO25, GPIO26, GPIO32, GPIO33
//
// NOTE: 3 GPIO only provide 2^3 = 8 unique binary combinations.
// To address 10 systems (ID 1..10) reliably, we use 4 GPIO (2^4 = 16 combinations).
//
// Wiring rule per ID pin: use ESP32 internal pull-up only (INPUT_PULLUP).
// No external 3.3V pull-up resistor is required.
// - pin left open (pull-up active) => bit 0
// - pin strapped to GND (jumper or switch) => bit 1
//
// Selected ID pins (safe and free on NodeMCU-32S):
// - ID_BIT0: GPIO25 (LSB)
// - ID_BIT1: GPIO26
// - ID_BIT2: GPIO32
// - ID_BIT3: GPIO33 (MSB)
//
// Table (system ID -> resistor straps to GND):
//  ID1  -> 0000 : none
//  ID2  -> 0001 : GPIO25
//  ID3  -> 0010 : GPIO26
//  ID4  -> 0011 : GPIO26, GPIO25
//  ID5  -> 0100 : GPIO32
//  ID6  -> 0101 : GPIO32, GPIO25
//  ID7  -> 0110 : GPIO32, GPIO26
//  ID8  -> 0111 : GPIO32, GPIO26, GPIO25
//  ID9  -> 1000 : GPIO33
//  ID10 -> 1001 : GPIO33, GPIO25

static const char *DEVICE_NAME_PREFIX = "ESP32-BLE-MPU-SUM";
static const char *SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e";
static const char *CHAR_TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e";

static const uint8_t ID_BIT0_GPIO = 25;
static const uint8_t ID_BIT1_GPIO = 26;
static const uint8_t ID_BIT2_GPIO = 32;
static const uint8_t ID_BIT3_GPIO = 33;
static const uint8_t SYSTEM_ID_MIN = 1;
static const uint8_t SYSTEM_ID_MAX = 10;

static const uint8_t MPU6050_ADDR_DEFAULT = 0x68;
static const uint8_t MPU6050_ADDR_ALT = 0x69;
static const uint8_t MPU6050_PWR_MGMT_1 = 0x6B;
static const uint8_t MPU6050_SMPLRT_DIV = 0x19;
static const uint8_t MPU6050_CONFIG = 0x1A;
static const uint8_t MPU6050_ACCEL_CONFIG = 0x1C;
static const uint8_t MPU6050_ACCEL_XOUT_H = 0x3B;

static const uint8_t DHT_PIN = 4;
static const uint8_t DHT_TYPE = DHT22;
static const unsigned long DHT_READ_INTERVAL_MS = 5000;

// Target 1 kHz sampling for MPU reads (max practical target for this setup).
static const unsigned long SAMPLE_INTERVAL_US = 1000;

BLECharacteristic *txCharacteristic = nullptr;
BLE2902 *txCccd = nullptr;
bool deviceConnected = false;
bool mpuAvailable = false;
uint8_t mpuAddress = MPU6050_ADDR_DEFAULT;
uint8_t systemId = SYSTEM_ID_MIN;
char deviceName[32] = {0};
unsigned long lastSampleUs = 0;
unsigned long lastDhtReadMs = 0;
float dhtTemperatureC = NAN;

DHT dht(DHT_PIN, DHT_TYPE);

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

uint8_t readSystemIdFromGpio() {
  pinMode(ID_BIT0_GPIO, INPUT_PULLUP);
  pinMode(ID_BIT1_GPIO, INPUT_PULLUP);
  pinMode(ID_BIT2_GPIO, INPUT_PULLUP);
  pinMode(ID_BIT3_GPIO, INPUT_PULLUP);

  // Active low because of INPUT_PULLUP: grounded pin means bit = 1.
  const uint8_t b0 = (digitalRead(ID_BIT0_GPIO) == LOW) ? 1 : 0;
  const uint8_t b1 = (digitalRead(ID_BIT1_GPIO) == LOW) ? 1 : 0;
  const uint8_t b2 = (digitalRead(ID_BIT2_GPIO) == LOW) ? 1 : 0;
  const uint8_t b3 = (digitalRead(ID_BIT3_GPIO) == LOW) ? 1 : 0;

  const uint8_t rawCode = static_cast<uint8_t>((b3 << 3) | (b2 << 2) | (b1 << 1) | b0);
  const uint8_t id = static_cast<uint8_t>(rawCode + 1);

  if (id < SYSTEM_ID_MIN || id > SYSTEM_ID_MAX) {
    return SYSTEM_ID_MIN;
  }
  return id;
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
  BLEDevice::init(deviceName);
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
  systemId = readSystemIdFromGpio();
  snprintf(deviceName, sizeof(deviceName), "%s-%02u", DEVICE_NAME_PREFIX, systemId);

  DBG_PRINTLN("Demarrage BLE + MPU6050 (somme accel uniquement)");
  DBG_PRINTLN(deviceName);

  Wire.begin(21, 22);
  Wire.setClock(400000);

  mpuAvailable = detectMpuAddress() && setupMpu6050();
  if (!mpuAvailable) {
    DBG_PRINTLN("MPU6050 absent/non initialise");
  }

  setupBle();
  dht.begin();

  // DHT22 needs a short startup time before first valid reading.
  delay(2000);
  const float t0 = dht.readTemperature();
  if (!isnan(t0)) {
    dhtTemperatureC = t0;
  }
  lastDhtReadMs = millis();

  DBG_PRINTLN("BLE advertising actif");
}

void loop() {
  if (!deviceConnected || txCharacteristic == nullptr || !mpuAvailable) {
    delay(10);
    return;
  }

  unsigned long nowUs = micros();
  unsigned long nowMs = millis();
  if (static_cast<unsigned long>(nowUs - lastSampleUs) < SAMPLE_INTERVAL_US) {
    return;
  }
  lastSampleUs = nowUs;

  if (nowMs - lastDhtReadMs >= DHT_READ_INTERVAL_MS) {
    const float t = dht.readTemperature();
    if (!isnan(t)) {
      dhtTemperatureC = t;
    }
    lastDhtReadMs = nowMs;
  }

  MpuAccelRaw data;
  if (!mpuReadAccel(data)) {
    return;
  }

  // +/-16g scale: 2048 LSB/g
  const float ax = data.ax / 2048.0f;
  const float ay = data.ay / 2048.0f;
  const float az = data.az / 2048.0f;
  const float accMagnitudeG = sqrtf((ax * ax) + (ay * ay) + (az * az));
  // Remove static gravity component so an immobile sensor is close to 0.
  const float dynamicAccG = fabsf(accMagnitudeG - 1.0f);

  // Compact payload for default MTU: "id,mg,temp".
  // temp is absolute value in plain degC, e.g. 24.7
  const uint16_t accMilliG = static_cast<uint16_t>(dynamicAccG * 1000.0f);
  char payload[24];
  const float tempAbsC = isnan(dhtTemperatureC) ? 0.0f : fabsf(dhtTemperatureC);

  if (!isnan(dhtTemperatureC)) {
    snprintf(payload,
             sizeof(payload),
             "%u,%u,%.1f",
             static_cast<unsigned>(systemId),
             static_cast<unsigned>(accMilliG),
             tempAbsC);
  } else {
    snprintf(payload,
             sizeof(payload),
             "%u,%u,0.0",
             static_cast<unsigned>(systemId),
             static_cast<unsigned>(accMilliG));
  }

  txCharacteristic->setValue(payload);
  if (isNotifyEnabled()) {
    txCharacteristic->notify();
  }
}
