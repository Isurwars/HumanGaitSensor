#include <Arduino.h>
#include <Wire.h>
#include "mpu6500.h"

// ESP32-C3 Default I2C Pins
#define I2C_SDA_PIN 8
#define I2C_SCL_PIN 9

Mpu6500 imu(&Wire, Mpu6500::I2cAddr::I2C_ADDR_PRIM);

void setup() {
  // Start Serial communication
  Serial.begin(115200);
  delay(3000); // Wait for USB CDC connection to stabilize on the host
  while (!Serial) {
    delay(10); // Wait for serial port to connect
  }
  Serial.println("HumanGaitSensor (Sensor de Marcha) Starting...");

  // Initialize I2C with default pins
  Serial.printf("Initializing I2C (SDA Pin: %d, SCL Pin: %d)...\n", I2C_SDA_PIN, I2C_SCL_PIN);
  if (!Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN)) {
    while (true) {
      Serial.println("Error: Failed to initialize I2C bus.");
      delay(2000);
    }
  }

  // Keep attempting to connect to MPU-6500, scanning the I2C bus on failure
  bool initialized = false;
  while (!initialized) {
    // Scan I2C bus for troubleshooting
    Serial.println("Scanning I2C bus for devices on configured pins...");
    int devicesFound = 0;
    for (byte address = 1; address < 127; address++) {
      Wire.beginTransmission(address);
      byte error = Wire.endTransmission();
      if (error == 0) {
        Serial.printf(" -> Found device at address 0x%02X\n", address);
        devicesFound++;
      } else if (error == 4) {
        Serial.printf(" -> Unknown error at address 0x%02X\n", address);
      }
    }
    if (devicesFound == 0) {
      Serial.println(" -> No I2C devices found! Check wiring, pull-up resistors, and power.");
    } else {
      Serial.printf(" -> Scan complete. Found %d device(s).\n", devicesFound);
    }

    Serial.println("Connecting and configuring MPU-6500...");
    if (imu.Begin()) {
      initialized = true;
      Serial.println("MPU-6500 successfully initialized!");
    } else {
      Serial.println("Error: Failed to communicate with MPU-6500 sensor.");
      Serial.println("Please check your wiring, sensor power, and I2C pins.");
      Serial.println("Retrying in 5 seconds...\n");
      delay(5000);
    }
  }
}

void loop() {
  // Read sensor data
  if (imu.Read()) {
    Vector3f accel = imu.accel_mps2();
    Vector3f gyro = imu.gyro_radps();
    
    Serial.print("IMU -> Accel [m/s^2]: ");
    Serial.printf("X: %7.3f, Y: %7.3f, Z: %7.3f | ", accel.x, accel.y, accel.z);
    
    Serial.print("Gyro [rad/s]: ");
    Serial.printf("X: %7.4f, Y: %7.4f, Z: %7.4f | ", gyro.x, gyro.y, gyro.z);

    Serial.printf("Temp [C]: %5.1f\n", imu.die_temp_c());
  } else {
    Serial.println("Failed to read MPU-6500 data.");
  }

  delay(200); // Read 5 times per second
}