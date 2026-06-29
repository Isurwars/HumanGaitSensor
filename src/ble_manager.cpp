#include "ble_manager.h"

void BleManager::Begin(const char* deviceName) {
  // Initialize the BLE Device
  BLEDevice::init(deviceName);

  // Create the BLE Server
  pServer_ = BLEDevice::createServer();
  pServer_->setCallbacks(this);

  // Create the BLE Service
  BLEService *pService = pServer_->createService(SERVICE_UUID);

  // Create the BLE Tx Characteristic for sensor data streaming
  pTxCharacteristic_ = pService->createCharacteristic(
      CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );

  // Add the Client Characteristic Configuration Descriptor (CCCD)
  // This is required for clients (like phones) to enable notifications.
  pTxCharacteristic_->addDescriptor(new BLE2902());

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  
  // These settings help with connection latency/stability on mobile platforms
  pAdvertising->setMinPreferred(0x06);  // set value to 0x00 to not advertise this parameter
  pAdvertising->setMinPreferred(0x12);
  
  BLEDevice::startAdvertising();
  Serial.println("BLE service initialized and advertising started!");
}

void BleManager::UpdateSensorData(float ax, float ay, float az, float gx, float gy, float gz, float temp) {
  if (device_connected_) {
    // Format the payload as a clean CSV string
    // This allows easy parsing on the receiver and readable raw data on BLE terminals
    char payload[64];
    snprintf(payload, sizeof(payload), "%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.1f",
             ax, ay, az, gx, gy, gz, temp);

    pTxCharacteristic_->setValue(payload);
    pTxCharacteristic_->notify();
  }
}

void BleManager::HandleConnectionHousekeeping() {
  // Handle disconnection
  if (!device_connected_ && old_device_connected_) {
    delay(500); // Give the BLE stack a moment to settle
    pServer_->startAdvertising(); // Restart advertising
    Serial.println("BLE Client Disconnected. Restarted advertising.");
    old_device_connected_ = device_connected_;
  }
  
  // Handle connection
  if (device_connected_ && !old_device_connected_) {
    Serial.println("BLE Client Connected.");
    old_device_connected_ = device_connected_;
  }
}

void BleManager::onConnect(BLEServer* pServer) {
  device_connected_ = true;
}

void BleManager::onDisconnect(BLEServer* pServer) {
  device_connected_ = false;
}
