#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLEAdvertising.h>
#include <BLE2902.h>

// BLE Nordic UART-like UUIDs (compatibles avec nRF Connect et apps UART BLE)
static const char *DEVICE_NAME = "ESP32-BLE-5DIGITS";
static const char *SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e";
static const char *CHAR_TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e";
static const char *CHAR_RX_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e";

static const char *FRAME_5_DIGITS = "salut tout le monde";
static const unsigned long SEND_INTERVAL_MS = 500;

BLECharacteristic *txCharacteristic = nullptr;
bool deviceConnected = false;
unsigned long lastSendMs = 0;

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) override {
    (void)pServer;
    deviceConnected = true;
    Serial.println("BLE: client connecte");
  }

  void onDisconnect(BLEServer *pServer) override {
    deviceConnected = false;
    Serial.println("BLE: client deconnecte, reprise advertising");
    pServer->startAdvertising();
  }
};

class RxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) override {
    std::string value = pCharacteristic->getValue();
    if (!value.empty()) {
      Serial.print("BLE RX: ");
      Serial.println(value.c_str());
    }
  }
};

void setupBle() {
  BLEDevice::init(DEVICE_NAME);
  BLEServer *server = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());

  BLEService *service = server->createService(SERVICE_UUID);

  txCharacteristic = service->createCharacteristic(
      CHAR_TX_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  txCharacteristic->addDescriptor(new BLE2902());

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

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("Demarrage BLE (trame fixe 5 chiffres)");

  setupBle();
  Serial.println("BLE advertising actif");
}

void loop() {
  if (!deviceConnected || txCharacteristic == nullptr) {
    delay(100);
    return;
  }

  unsigned long now = millis();
  if (now - lastSendMs < SEND_INTERVAL_MS) {
    return;
  }
  lastSendMs = now;

  txCharacteristic->setValue(FRAME_5_DIGITS);
  txCharacteristic->notify();

  Serial.print("BLE TX: ");
  Serial.println(FRAME_5_DIGITS);
}