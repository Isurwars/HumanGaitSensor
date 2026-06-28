/**
 * @file mpu6500.cpp
 * @brief Implementation of the simplified Mpu6500 driver.
 *
 * This file contains only the code needed for 6-DoF IMU operation.
 *
 * Axis convention (applied in Read() / ReadFifo()):
 *   Output X = Sensor Y  (forward)
 *   Output Y = Sensor X  (right)
 *   Output Z = -Sensor Z (down, NED convention when mounted flat top-up)
 */

#include "mpu6500.h"
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cmath>

// ---------------------------------------------------------------------------
// Physical Constants
// ---------------------------------------------------------------------------

/// Standard gravity: converts g to m/s²
static constexpr float G_TO_MPS2 = 9.80665f;

/// Degrees-to-radians. Defined locally to avoid depending on the Arduino.h macro.
static constexpr float DEG_TO_RAD_ = static_cast<float>(M_PI) / 180.0f;

// ---------------------------------------------------------------------------
// Runtime Configuration
// ---------------------------------------------------------------------------

void Mpu6500::Config(TwoWire *i2c, const I2cAddr addr) {
  i2c_   = i2c;
  dev_   = static_cast<uint8_t>(addr);
  iface_ = Interface::I2C;
}

void Mpu6500::Config(SPIClass *spi, const uint8_t cs) {
  spi_   = spi;
  dev_   = cs;
  iface_ = Interface::SPI;
}

// ---------------------------------------------------------------------------
// Initialization
// ---------------------------------------------------------------------------

bool Mpu6500::Begin() {
  if (iface_ == Interface::SPI) {
    /* Toggle CS to latch SPI mode (MPU-6500 datasheet §5.1). */
    pinMode(dev_, OUTPUT);
    digitalWrite(dev_, LOW);
    delay(1);
    digitalWrite(dev_, HIGH);
    delay(1);
  }

  /* Use 1 MHz for all configuration writes. */
  spi_clock_ = SPI_CFG_CLOCK_;

  /* Select PLL clock source before reading WHO_AM_I (required on some revisions). */
  if (!WriteRegister(PWR_MGMNT_1_, CLKSEL_PLL_)) { return false; }

  /* ---- Chip identity check ---- */
  if (!ReadRegisters(WHOAMI_, sizeof(who_am_i_), &who_am_i_)) {
    Serial.println(" -> WHO_AM_I read failed!");
    return false;
  }
  if ((who_am_i_ != WHOAMI_MPU6500_) &&
      (who_am_i_ != WHOAMI_MPU9250_) &&
      (who_am_i_ != WHOAMI_MPU9255_)) {
    Serial.printf(" -> Unrecognised WHO_AM_I: 0x%02X "
                  "(Expected 0x70, 0x71, or 0x73)\n", who_am_i_);
    return false;
  }
  Serial.printf(" -> MPU-6500 (or compatible) detected. WHO_AM_I: 0x%02X\n",
                who_am_i_);

  /* ---- Hardware reset ---- */
  /* H_RESET self-clears after reset, so readback will never match — use the
   * no-verify variant to avoid a spurious validation-mismatch log message. */
  WriteRegisterNoVerify(PWR_MGMNT_1_, H_RESET_);
  delay(100);  /* MPU-6500 clones may need more than the 1 ms in the datasheet. */

  /* Restore PLL clock source (reset cleared it). */
  if (!WriteRegister(PWR_MGMNT_1_, CLKSEL_PLL_)) { return false; }

  /* ---- Apply default sensor configuration ---- */
  if (!ConfigAccelRange(AccelRange::ACCEL_RANGE_16G))              { return false; }
  if (!ConfigGyroRange(GyroRange::GYRO_RANGE_2000DPS))             { return false; }
  if (!ConfigDlpfBandwidth(DlpfBandwidth::DLPF_BANDWIDTH_184HZ))   { return false; }
  if (!ConfigSrd(0))                                                { return false; }

  return true;
}

// ---------------------------------------------------------------------------
// Interrupt Control
// ---------------------------------------------------------------------------

bool Mpu6500::EnableDrdyInt() {
  spi_clock_ = SPI_CFG_CLOCK_;
  /* INT pin: active-high, push-pull, 50 µs pulse, cleared by any register read. */
  if (!WriteRegister(INT_PIN_CFG_, INT_PULSE_50US_)) { return false; }
  if (!WriteRegister(INT_ENABLE_, INT_RAW_RDY_EN_))  { return false; }
  return true;
}

bool Mpu6500::DisableDrdyInt() {
  spi_clock_ = SPI_CFG_CLOCK_;
  if (!WriteRegister(INT_ENABLE_, INT_DISABLE_)) { return false; }
  return true;
}

// ---------------------------------------------------------------------------
// Sensor Range / Filter Configuration
// ---------------------------------------------------------------------------

bool Mpu6500::ConfigAccelRange(const AccelRange range) {
  spi_clock_ = SPI_CFG_CLOCK_;

  /* Scale factor: converts raw 16-bit counts to g.
   * scale = full_scale_g / 32767.5  (32767.5 = midpoint of int16 range) */
  switch (range) {
    case AccelRange::ACCEL_RANGE_2G:
      requested_accel_range_ = range;
      requested_accel_scale_ = 2.0f / 32767.5f;
      break;
    case AccelRange::ACCEL_RANGE_4G:
      requested_accel_range_ = range;
      requested_accel_scale_ = 4.0f / 32767.5f;
      break;
    case AccelRange::ACCEL_RANGE_8G:
      requested_accel_range_ = range;
      requested_accel_scale_ = 8.0f / 32767.5f;
      break;
    case AccelRange::ACCEL_RANGE_16G:
      requested_accel_range_ = range;
      requested_accel_scale_ = 16.0f / 32767.5f;
      break;
    default:
      return false;
  }

  if (!WriteRegister(ACCEL_CONFIG_, static_cast<uint8_t>(requested_accel_range_))) {
    return false;
  }
  accel_range_ = requested_accel_range_;
  accel_scale_ = requested_accel_scale_;
  return true;
}

bool Mpu6500::ConfigGyroRange(const GyroRange range) {
  spi_clock_ = SPI_CFG_CLOCK_;

  /* Scale factor: converts raw 16-bit counts to degrees/s.
   * scale = full_scale_dps / 32767.5 */
  switch (range) {
    case GyroRange::GYRO_RANGE_250DPS:
      requested_gyro_range_ = range;
      requested_gyro_scale_ = 250.0f / 32767.5f;
      break;
    case GyroRange::GYRO_RANGE_500DPS:
      requested_gyro_range_ = range;
      requested_gyro_scale_ = 500.0f / 32767.5f;
      break;
    case GyroRange::GYRO_RANGE_1000DPS:
      requested_gyro_range_ = range;
      requested_gyro_scale_ = 1000.0f / 32767.5f;
      break;
    case GyroRange::GYRO_RANGE_2000DPS:
      requested_gyro_range_ = range;
      requested_gyro_scale_ = 2000.0f / 32767.5f;
      break;
    default:
      return false;
  }

  if (!WriteRegister(GYRO_CONFIG_, static_cast<uint8_t>(requested_gyro_range_))) {
    return false;
  }
  gyro_range_ = requested_gyro_range_;
  gyro_scale_ = requested_gyro_scale_;
  return true;
}

bool Mpu6500::ConfigSrd(const uint8_t srd) {
  spi_clock_ = SPI_CFG_CLOCK_;
  /* ODR = 1000 Hz / (1 + srd) when DLPF is enabled.
   * No magnetometer rate synchronisation required (IMU-only). */
  if (!WriteRegister(SMPLRT_DIV_, srd)) { return false; }
  srd_ = srd;
  return true;
}

bool Mpu6500::ConfigDlpfBandwidth(const DlpfBandwidth dlpf) {
  spi_clock_ = SPI_CFG_CLOCK_;

  switch (dlpf) {
    case DlpfBandwidth::DLPF_BANDWIDTH_184HZ:
    case DlpfBandwidth::DLPF_BANDWIDTH_92HZ:
    case DlpfBandwidth::DLPF_BANDWIDTH_41HZ:
    case DlpfBandwidth::DLPF_BANDWIDTH_20HZ:
    case DlpfBandwidth::DLPF_BANDWIDTH_10HZ:
    case DlpfBandwidth::DLPF_BANDWIDTH_5HZ:
      requested_dlpf_ = dlpf;
      break;
    default:
      return false;
  }

  /* Write the same value to both the accel (ACCEL_CONFIG2) and gyro (CONFIG)
   * DLPF registers to keep both sensors in sync. */
  if (!WriteRegister(ACCEL_CONFIG2_, static_cast<uint8_t>(requested_dlpf_))) {
    return false;
  }
  if (!WriteRegister(CONFIG_, static_cast<uint8_t>(requested_dlpf_))) {
    return false;
  }
  dlpf_bandwidth_ = requested_dlpf_;
  return true;
}

// ---------------------------------------------------------------------------
// Wake-on-Motion
// ---------------------------------------------------------------------------

bool Mpu6500::EnableWom(int16_t threshold_mg, const WomRate wom_rate) {
  /* Hardware supports 4–1020 mg in 4 mg steps (1 LSB = 4 mg). */
  if ((threshold_mg < 4) || (threshold_mg > 1020)) { return false; }

  spi_clock_ = SPI_CFG_CLOCK_;

  /* Full reset to start from a known state. */
  WriteRegisterNoVerify(PWR_MGMNT_1_, H_RESET_);
  delay(100);

  /* Clear sleep/cycle/standby bits; use internal oscillator. */
  if (!WriteRegister(PWR_MGMNT_1_, 0x00)) { return false; }

  /* Power down gyroscope to reduce current in WOM mode. */
  if (!WriteRegister(PWR_MGMNT_2_, DISABLE_GYRO_)) { return false; }

  /* Accel DLPF at 184 Hz — recommended bandwidth for WOM comparator. */
  if (!WriteRegister(ACCEL_CONFIG2_,
      static_cast<uint8_t>(DlpfBandwidth::DLPF_BANDWIDTH_184HZ))) {
    return false;
  }

  /* Route only the WOM interrupt to the INT pin. */
  if (!WriteRegister(INT_ENABLE_, INT_WOM_EN_)) { return false; }

  /* Enable accel hardware intelligence:
   *   ACCEL_INTEL_EN   = enable the WOM comparator
   *   ACCEL_INTEL_MODE = compare current sample vs. previous */
  if (!WriteRegister(MOT_DETECT_CTRL_,
      (ACCEL_INTEL_EN_ | ACCEL_INTEL_MODE_))) {
    return false;
  }

  /* Set detection threshold (1 LSB = 4 mg). */
  if (!WriteRegister(WOM_THR_, static_cast<uint8_t>(threshold_mg / 4))) {
    return false;
  }

  /* Configure low-power accel sample rate. */
  if (!WriteRegister(LP_ACCEL_ODR_, static_cast<uint8_t>(wom_rate))) {
    return false;
  }

  /* Enter cycle mode: accelerometer wakes at WomRate, checks threshold, sleeps. */
  if (!WriteRegister(PWR_MGMNT_1_, PWR_CYCLE_WOM_)) { return false; }

  return true;
}

// ---------------------------------------------------------------------------
// FIFO Control (conditional compile)
// ---------------------------------------------------------------------------

#if !defined(DISABLE_MPU6500_FIFO)

bool Mpu6500::EnableFifo() {
  spi_clock_ = SPI_CFG_CLOCK_;
  /* Enable FIFO subsystem in USER_CTRL. */
  if (!WriteRegister(USER_CTRL_, FIFO_EN_CTRL_)) { return false; }
  /* Select accel and all three gyro axes as FIFO data sources. */
  if (!WriteRegister(FIFO_EN_, FIFO_ACCEL_ | FIFO_GYRO_)) { return false; }
  return true;
}

bool Mpu6500::DisableFifo() {
  spi_clock_ = SPI_CFG_CLOCK_;
  /* Stop all sensors from writing to the FIFO. */
  if (!WriteRegister(FIFO_EN_, 0x00)) { return false; }
  /* Reset the FIFO buffer. */
  WriteRegisterNoVerify(USER_CTRL_, FIFO_RESET_);
  return true;
}

#endif  // !defined(DISABLE_MPU6500_FIFO)

// ---------------------------------------------------------------------------
// Reset
// ---------------------------------------------------------------------------

void Mpu6500::Reset() {
  spi_clock_ = SPI_CFG_CLOCK_;
  /* H_RESET self-clears — use no-verify variant to suppress noise. */
  WriteRegisterNoVerify(PWR_MGMNT_1_, H_RESET_);
  delay(100);
}

// ---------------------------------------------------------------------------
// Data Acquisition
// ---------------------------------------------------------------------------

bool Mpu6500::Read() {
  spi_clock_ = SPI_READ_CLOCK_;
  new_imu_data_ = false;

  /*
   * Burst-read 15 bytes starting at INT_STATUS (0x3A):
   *   data_buf_[0]    = INT_STATUS
   *   data_buf_[1–6]  = ACCEL X/Y/Z (big-endian, 2 bytes each)
   *   data_buf_[7–8]  = TEMP (big-endian)
   *   data_buf_[9–14] = GYRO X/Y/Z (big-endian, 2 bytes each)
   *
   * This reads exactly through GYRO_ZOUT_L (0x48), with no mag data.
   */
  if (!ReadRegisters(INT_STATUS_, sizeof(data_buf_), data_buf_)) {
    return false;
  }

#if !defined(DISABLE_MPU6500_FIFO)
  /* Bit 4 of INT_STATUS: FIFO overflow. */
  fifo_overflow_ = (data_buf_[0] & FIFO_OVERFLOW_INT_);
#endif

  /* Bit 0 of INT_STATUS: all sensor registers have been refreshed. */
  new_imu_data_ = (data_buf_[0] & RAW_DATA_RDY_INT_);
  if (!new_imu_data_) { return false; }

  /* ---- Unpack raw 16-bit counts (big-endian) ---- */
  accel_cnts_[0] = static_cast<int16_t>(data_buf_[1])  << 8 | data_buf_[2];
  accel_cnts_[1] = static_cast<int16_t>(data_buf_[3])  << 8 | data_buf_[4];
  accel_cnts_[2] = static_cast<int16_t>(data_buf_[5])  << 8 | data_buf_[6];
  temp_cnts_     = static_cast<int16_t>(data_buf_[7])  << 8 | data_buf_[8];
  gyro_cnts_[0]  = static_cast<int16_t>(data_buf_[9])  << 8 | data_buf_[10];
  gyro_cnts_[1]  = static_cast<int16_t>(data_buf_[11]) << 8 | data_buf_[12];
  gyro_cnts_[2]  = static_cast<int16_t>(data_buf_[13]) << 8 | data_buf_[14];

  /*
   * ---- Convert to SI units with axis rotation ----
   *
   * The MPU-6500 PCB axes differ from the desired body frame. Mapping:
   *   Output X  = Sensor Y  (forward)
   *   Output Y  = Sensor X  (right)
   *   Output Z  = -Sensor Z (down, NED convention)
   *
   * Adjust if the sensor is mounted differently on your PCB.
   */
  accel_.x = static_cast<float>(accel_cnts_[1]) * accel_scale_ * G_TO_MPS2;
  accel_.y = static_cast<float>(accel_cnts_[0]) * accel_scale_ * G_TO_MPS2;
  accel_.z = static_cast<float>(accel_cnts_[2]) * accel_scale_ * -1.0f * G_TO_MPS2;

  /*
   * Temperature (MPU-6500 datasheet §4.18):
   *   T [°C] = (TEMP_OUT - RoomTemp_Offset) / Temp_Sensitivity + 21
   * where RoomTemp_Offset = 0 (reference at 21 °C), Sensitivity = 333.87 LSB/°C.
   */
  temp_ = (static_cast<float>(temp_cnts_) - 21.0f) / TEMP_SCALE_ + 21.0f;

  gyro_.x = static_cast<float>(gyro_cnts_[1]) * gyro_scale_ * DEG_TO_RAD_;
  gyro_.y = static_cast<float>(gyro_cnts_[0]) * gyro_scale_ * DEG_TO_RAD_;
  gyro_.z = static_cast<float>(gyro_cnts_[2]) * gyro_scale_ * -1.0f * DEG_TO_RAD_;

  return true;
}

// ---------------------------------------------------------------------------
// FIFO Data Acquisition (conditional compile)
// ---------------------------------------------------------------------------

#if !defined(DISABLE_MPU6500_FIFO)

int8_t Mpu6500::ReadFifo() {
  spi_clock_ = SPI_READ_CLOCK_;

  /* Read 2-byte FIFO byte count (bits [12:0] are valid). */
  if (!ReadRegisters(FIFO_COUNT_, 2, data_buf_)) { return -1; }
  fifo_bytes_      = static_cast<int16_t>(data_buf_[0] & 0x0F) << 8 | data_buf_[1];
  fifo_num_frames_ = fifo_bytes_ / FIFO_FRAME_SIZE_;

  const int8_t frames_to_read = std::min(fifo_num_frames_, FIFO_MAX_NUM_FRAMES_);

  for (int8_t i = 0; i < frames_to_read; i++) {
    /*
     * Each 12-byte FIFO frame layout:
     *   bytes [0–5]  = ACCEL X/Y/Z (big-endian, 2 bytes each)
     *   bytes [6–11] = GYRO  X/Y/Z (big-endian, 2 bytes each)
     * (Temperature and magnetometer are NOT written to the FIFO.)
     */
    if (!ReadRegisters(FIFO_READ_, FIFO_FRAME_SIZE_, data_buf_)) { return -1; }

    accel_cnts_[0] = static_cast<int16_t>(data_buf_[0]) << 8 | data_buf_[1];
    accel_cnts_[1] = static_cast<int16_t>(data_buf_[2]) << 8 | data_buf_[3];
    accel_cnts_[2] = static_cast<int16_t>(data_buf_[4]) << 8 | data_buf_[5];
    gyro_cnts_[0]  = static_cast<int16_t>(data_buf_[6]) << 8 | data_buf_[7];
    gyro_cnts_[1]  = static_cast<int16_t>(data_buf_[8]) << 8 | data_buf_[9];
    gyro_cnts_[2]  = static_cast<int16_t>(data_buf_[10]) << 8 | data_buf_[11];

    /* Same axis rotation as Read() — see comments there. */
    fifo_ax_[i] = static_cast<float>(accel_cnts_[1]) * accel_scale_ * G_TO_MPS2;
    fifo_ay_[i] = static_cast<float>(accel_cnts_[0]) * accel_scale_ * G_TO_MPS2;
    fifo_az_[i] = static_cast<float>(accel_cnts_[2]) * accel_scale_ * -1.0f * G_TO_MPS2;
    fifo_gx_[i] = static_cast<float>(gyro_cnts_[1]) * gyro_scale_ * DEG_TO_RAD_;
    fifo_gy_[i] = static_cast<float>(gyro_cnts_[0]) * gyro_scale_ * DEG_TO_RAD_;
    fifo_gz_[i] = static_cast<float>(gyro_cnts_[2]) * gyro_scale_ * -1.0f * DEG_TO_RAD_;
  }

  return fifo_num_frames_;
}

int8_t Mpu6500::fifo_accel_x_mps2(float *data, std::size_t len) {
  if (!data) { return -1; }
  int8_t n = std::min(fifo_num_frames_, static_cast<int8_t>(len));
  memcpy(data, fifo_ax_, n * sizeof(float));
  return n;
}

int8_t Mpu6500::fifo_accel_y_mps2(float *data, std::size_t len) {
  if (!data) { return -1; }
  int8_t n = std::min(fifo_num_frames_, static_cast<int8_t>(len));
  memcpy(data, fifo_ay_, n * sizeof(float));
  return n;
}

int8_t Mpu6500::fifo_accel_z_mps2(float *data, std::size_t len) {
  if (!data) { return -1; }
  int8_t n = std::min(fifo_num_frames_, static_cast<int8_t>(len));
  memcpy(data, fifo_az_, n * sizeof(float));
  return n;
}

int8_t Mpu6500::fifo_gyro_x_radps(float *data, std::size_t len) {
  if (!data) { return -1; }
  int8_t n = std::min(fifo_num_frames_, static_cast<int8_t>(len));
  memcpy(data, fifo_gx_, n * sizeof(float));
  return n;
}

int8_t Mpu6500::fifo_gyro_y_radps(float *data, std::size_t len) {
  if (!data) { return -1; }
  int8_t n = std::min(fifo_num_frames_, static_cast<int8_t>(len));
  memcpy(data, fifo_gy_, n * sizeof(float));
  return n;
}

int8_t Mpu6500::fifo_gyro_z_radps(float *data, std::size_t len) {
  if (!data) { return -1; }
  int8_t n = std::min(fifo_num_frames_, static_cast<int8_t>(len));
  memcpy(data, fifo_gz_, n * sizeof(float));
  return n;
}

#endif  // !defined(DISABLE_MPU6500_FIFO)

// ---------------------------------------------------------------------------
// Low-Level Communication
// ---------------------------------------------------------------------------

bool Mpu6500::WriteRegister(uint8_t reg, uint8_t data) {
  /*
   * Write the byte, wait for the register to settle, then read back to
   * confirm. Self-clearing registers (H_RESET, WOM bits) must NOT use
   * this function — use WriteRegisterNoVerify() instead.
   */
  if (iface_ == Interface::I2C) {
    i2c_->beginTransmission(dev_);
    i2c_->write(reg);
    i2c_->write(data);
    byte err = i2c_->endTransmission();
    if (err != 0) {
      Serial.printf(" -> WriteRegister(0x%02X, 0x%02X): I2C error %d\n",
                    reg, data, err);
      return false;
    }
  } else {
    spi_->beginTransaction(SPISettings(spi_clock_, MSBFIRST, SPI_MODE3));
    digitalWrite(dev_, LOW);
    spi_->transfer(reg);
    spi_->transfer(data);
    digitalWrite(dev_, HIGH);
    spi_->endTransaction();
  }

  delay(10);  /* Allow register to settle before readback. */

  uint8_t ret_val;
  if (!ReadRegisters(reg, sizeof(ret_val), &ret_val)) {
    Serial.printf(" -> WriteRegister(0x%02X, 0x%02X): readback failed!\n", reg, data);
    return false;
  }
  if (data == ret_val) { return true; }

  Serial.printf(" -> WriteRegister(0x%02X, 0x%02X): mismatch! Got 0x%02X\n",
                reg, data, ret_val);
  return false;
}

void Mpu6500::WriteRegisterNoVerify(uint8_t reg, uint8_t data) {
  /*
   * Write without readback — for use with self-clearing or transitional
   * registers (e.g. PWR_MGMNT_1 H_RESET bit, FIFO_RESET) where the value
   * will have changed by the time we could read it back.
   */
  if (iface_ == Interface::I2C) {
    i2c_->beginTransmission(dev_);
    i2c_->write(reg);
    i2c_->write(data);
    i2c_->endTransmission();
  } else {
    spi_->beginTransaction(SPISettings(spi_clock_, MSBFIRST, SPI_MODE3));
    digitalWrite(dev_, LOW);
    spi_->transfer(reg);
    spi_->transfer(data);
    digitalWrite(dev_, HIGH);
    spi_->endTransaction();
  }
}

bool Mpu6500::ReadRegisters(uint8_t reg, uint8_t count, uint8_t *data) {
  if (iface_ == Interface::I2C) {
    /* Repeated-START: set register pointer without releasing the bus. */
    i2c_->beginTransmission(dev_);
    i2c_->write(reg);
    i2c_->endTransmission(false);
    bytes_rx_ = i2c_->requestFrom(static_cast<uint8_t>(dev_), count);
    if (bytes_rx_ == count) {
      for (std::size_t i = 0; i < count; i++) {
        data[i] = i2c_->read();
      }
      return true;
    }
    return false;
  } else {
    /* SPI burst read: MSB of address byte = 1 for a read operation. */
    spi_->beginTransaction(SPISettings(spi_clock_, MSBFIRST, SPI_MODE3));
    digitalWrite(dev_, LOW);
    spi_->transfer(reg | SPI_READ_);  /* Send address with read flag */
    spi_->transfer(data, count);      /* In-place receive into buffer */
    digitalWrite(dev_, HIGH);
    spi_->endTransaction();
    return true;
  }
}
