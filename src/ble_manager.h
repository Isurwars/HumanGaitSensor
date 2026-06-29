#ifndef BLE_MANAGER_H_
#define BLE_MANAGER_H_

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

class BleManager : public BLEServerCallbacks {
 public:
  BleManager() = default;

  /**
   * @brief Initializes BLE server, services, and characteristics.
   * @param deviceName Name of the device to advertise.
   */
  void Begin(const char* deviceName);

  /**
   * @brief Formats and updates the IMU data payload, notifying connected clients.
   * @param ax Accelerometer X (m/s^2)
   * @param ay Accelerometer Y (m/s^2)
   * @param az Accelerometer Z (m/s^2)
   * @param gx Gyroscope X (rad/s)
   * @param gy Gyroscope Y (rad/s)
   * @param gz Gyroscope Z (rad/s)
   * @param temp Die temperature (C)
   */
  void UpdateSensorData(float ax, float ay, float az, float gx, float gy, float gz, float temp);

  /**
   * @brief Returns true if a client is currently connected.
   */
  bool IsConnected() const { return device_connected_; }

  /**
   * @brief Handles connection/disconnection housekeeping (e.g. restart advertising).
   * Call this in the main loop.
   */
  void HandleConnectionHousekeeping();

  // BLEServerCallbacks overrides
  void onConnect(BLEServer* pServer) override;
  void onDisconnect(BLEServer* pServer) override;

 private:
  BLEServer* pServer_ = nullptr;
  BLECharacteristic* pTxCharacteristic_ = nullptr;
  bool device_connected_ = false;
  bool old_device_connected_ = false;

  // Custom service and characteristic UUIDs
  static constexpr const char* SERVICE_UUID = "4fafc201-1fb5-459e-8fcc-c5c9c331914b";
  static constexpr const char* CHARACTERISTIC_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8";
};

#endif  // BLE_MANAGER_H_
