/**
 * @file mpu9250.cpp
 * @brief Implementation of the Mpu9250 driver.
 *
 * All communication with the MPU-9250 is handled here. The AK8963 magnetometer
 * is accessed indirectly through the MPU's on-chip I2C master via the
 * WriteAk8963Register() and ReadAk8963Registers() helpers.
 *
 * Axis convention (after rotation applied in Read() / ReadFifo()):
 *   X → forward, Y → right, Z → down  (NED-compatible if mounted flat, top-up)
 *
 * The raw register layout from the MPU stores axes in a different order;
 * see the axis-rotation comments in Read() for the exact mapping.
 */

#include "mpu9250.h"
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cmath>

// ---------------------------------------------------------------------------
// Physical Constants
// ---------------------------------------------------------------------------

/// Standard gravity: conversion factor from g to m/s²
static constexpr float G_TO_MPS2 = 9.80665f;

/// Degrees-to-radians conversion. Defined locally to avoid relying on the
/// Arduino.h DEG_TO_RAD macro, making the driver more portable.
static constexpr float DEG_TO_RAD_ = static_cast<float>(M_PI) / 180.0f;

// ---------------------------------------------------------------------------
// Runtime Configuration
// ---------------------------------------------------------------------------

void Mpu9250::Config(TwoWire *i2c, const I2cAddr addr) {
  i2c_   = i2c;
  dev_   = static_cast<uint8_t>(addr);
  iface_ = Interface::I2C;
}

void Mpu9250::Config(SPIClass *spi, const uint8_t cs) {
  spi_   = spi;
  dev_   = cs;
  iface_ = Interface::SPI;
}

// ---------------------------------------------------------------------------
// Initialization
// ---------------------------------------------------------------------------

bool Mpu9250::Begin() {
  if (iface_ == Interface::SPI) {
    pinMode(dev_, OUTPUT);
    /* Toggle CS low then high to latch the device into SPI mode
     * (required per MPU-9250 datasheet §5.1). */
    digitalWrite(dev_, LOW);
    delay(1);
    digitalWrite(dev_, HIGH);
    delay(1);
  }

  /* Use the slow 1 MHz clock for all configuration register writes. */
  spi_clock_ = SPI_CFG_CLOCK_;

  /* Select PLL clock source (auto-selects gyro PLL when available).
   * This is required before reading WHO_AM_I in some silicon revisions. */
  if (!WriteRegister(PWR_MGMNT_1_, CLKSEL_PLL_)) {
    return false;
  }

  /* ---- Chip identity check ---- */
  if (!ReadRegisters(WHOAMI_, sizeof(who_am_i_), &who_am_i_)) {
    Serial.println(" -> Check WHOAMI: Read failed!");
    return false;
  }
  if ((who_am_i_ != WHOAMI_MPU9250_) &&
      (who_am_i_ != WHOAMI_MPU9255_) &&
      (who_am_i_ != WHOAMI_MPU6500_)) {
    Serial.printf(" -> Check WHOAMI: Invalid WHOAMI byte! "
                  "Value: 0x%02X (Expected: 0x70, 0x71 or 0x73)\n", who_am_i_);
    return false;
  }

  if (who_am_i_ == WHOAMI_MPU6500_) {
    /* MPU-6500 is identical to MPU-9250 but ships without the AK8963. */
    has_magnetometer_ = false;
    Serial.println(" -> MPU-6500 detected (No internal magnetometer).");
  } else {
    has_magnetometer_ = true;
    Serial.printf(" -> MPU-9250/9255 detected. WHOAMI: 0x%02X\n", who_am_i_);
  }

  /* ---- Pre-reset: power down AK8963 cleanly before resetting the MPU ---- */
  if (has_magnetometer_) {
    /* Enable the MPU's internal I2C master so we can talk to AK8963. */
    if (!WriteRegister(USER_CTRL_, I2C_MST_EN_)) { return false; }
    /* Set auxiliary I2C bus speed to 400 kHz. */
    if (!WriteRegister(I2C_MST_CTRL_, I2C_MST_CLK_)) { return false; }
    /* Power down AK8963 gracefully before the MPU hard-reset below. */
    WriteAk8963Register(AK8963_CNTL1_, AK8963_PWR_DOWN_);
  }

  /* ---- Hardware reset ---- */
  /* H_RESET clears all registers and reboots internal state machines.
   * The return value is intentionally ignored because the reset causes
   * the register to self-clear, so readback validation will always fail. */
  WriteRegister(PWR_MGMNT_1_, H_RESET_);
  delay(1);  /* Datasheet requires ≥ 1 ms after reset before any access. */

  if (has_magnetometer_) {
    /* Soft-reset the AK8963 as well to clear any stuck state. */
    WriteAk8963Register(AK8963_CNTL2_, AK8963_RESET_);
  }

  /* Restore PLL clock source after reset. */
  if (!WriteRegister(PWR_MGMNT_1_, CLKSEL_PLL_)) { return false; }

  /* ---- AK8963 magnetometer initialisation ---- */
  if (has_magnetometer_) {
    /* Re-enable I2C master and set bus speed (reset cleared these). */
    if (!WriteRegister(USER_CTRL_, I2C_MST_EN_)) { return false; }
    if (!WriteRegister(I2C_MST_CTRL_, I2C_MST_CLK_)) { return false; }

    /* Verify AK8963 identity. */
    if (!ReadAk8963Registers(AK8963_WHOAMI_, sizeof(who_am_i_), &who_am_i_)) {
      Serial.println(" -> Check AK8963 WHOAMI: Read failed!");
      return false;
    }
    if (who_am_i_ != WHOAMI_AK8963_) {
      Serial.printf(" -> Check AK8963 WHOAMI: Invalid! "
                    "Value: 0x%02X (Expected: 0x48)\n", who_am_i_);
      return false;
    }
    Serial.printf(" -> AK8963 detected. WHOAMI: 0x%02X\n", who_am_i_);

    /* ---- Read factory magnetometer sensitivity adjustment (ASA) values ---- */
    /* AK8963 must be put into FUSE ROM mode to expose the ASA registers. */
    if (!WriteAk8963Register(AK8963_CNTL1_, AK8963_PWR_DOWN_)) { return false; }
    delay(100);  /* AK8963 requires ≥ 100 ms between mode changes. */

    if (!WriteAk8963Register(AK8963_CNTL1_, AK8963_FUSE_ROM_)) { return false; }
    delay(100);

    if (!ReadAk8963Registers(AK8963_ASA_, sizeof(asa_buff_), asa_buff_)) {
      return false;
    }

    /*
     * Compute per-axis sensitivity adjustment scale factors.
     *
     * The AK8963 stores a factory-calibrated ASA byte for each axis.
     * Per the AK8963 datasheet §8.3.11, the adjusted sensitivity is:
     *
     *   Hadj = H × ((ASA - 128) / 256 + 1)
     *
     * where H is the raw 16-bit measurement count.
     *
     * We fold the final unit conversion (counts → µT) into the same factor:
     *   • Full-scale range of AK8963 in 16-bit mode: ±4912 µT
     *   • Max 16-bit output value: 32760 counts
     *   ⟹ 1 count = 4912 / 32760 µT ≈ 0.15 µT/count
     *
     * Combined factor stored in mag_scale_[i]:
     *   mag_scale_[i] = ((ASA[i] - 128) / 256 + 1) × (4912 / 32760)
     */
    for (int i = 0; i < 3; i++) {
      mag_scale_[i] = ((static_cast<float>(asa_buff_[i]) - 128.0f) / 256.0f + 1.0f)
                      * 4912.0f / 32760.0f;
    }

    /* Return AK8963 to power-down before setting measurement mode. */
    if (!WriteAk8963Register(AK8963_CNTL1_, AK8963_PWR_DOWN_)) { return false; }

    /* Set AK8963 to 16-bit resolution, continuous measurement mode 2 (100 Hz).
     * This is the default; ConfigSrd() may later switch to 8 Hz if needed. */
    if (!WriteAk8963Register(AK8963_CNTL1_, AK8963_CNT_MEAS2_)) { return false; }
    delay(100);

    /* Restore PLL clock (AK8963 operations can disturb I2C master timing). */
    if (!WriteRegister(PWR_MGMNT_1_, CLKSEL_PLL_)) { return false; }
  }

  /* ---- Apply default IMU configuration ---- */
  if (!ConfigAccelRange(AccelRange::ACCEL_RANGE_16G))     { return false; }
  if (!ConfigGyroRange(GyroRange::GYRO_RANGE_2000DPS))    { return false; }
  if (!ConfigDlpfBandwidth(DlpfBandwidth::DLPF_BANDWIDTH_184HZ)) { return false; }
  if (!ConfigSrd(0))                                       { return false; }

  return true;
}

// ---------------------------------------------------------------------------
// Interrupt Control
// ---------------------------------------------------------------------------

bool Mpu9250::EnableDrdyInt() {
  spi_clock_ = SPI_CFG_CLOCK_;
  /* Configure INT pin: active-high, push-pull, 50 µs pulse, clears on any read. */
  if (!WriteRegister(INT_PIN_CFG_, INT_PULSE_50US_)) { return false; }
  /* Enable the raw data-ready interrupt source. */
  if (!WriteRegister(INT_ENABLE_, INT_RAW_RDY_EN_)) { return false; }
  return true;
}

bool Mpu9250::DisableDrdyInt() {
  spi_clock_ = SPI_CFG_CLOCK_;
  /* Writing 0x00 clears all interrupt enables. */
  if (!WriteRegister(INT_ENABLE_, INT_DISABLE_)) { return false; }
  return true;
}

// ---------------------------------------------------------------------------
// Sensor Range / Filter Configuration
// ---------------------------------------------------------------------------

bool Mpu9250::ConfigAccelRange(const AccelRange range) {
  spi_clock_ = SPI_CFG_CLOCK_;

  /*
   * Map the selected range to its corresponding scale factor.
   * Scale factor converts raw 16-bit signed counts to g:
   *   scale = full_scale_g / 32767.5   (32767.5 = (2^15 - 0.5) midpoint)
   */
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

  /* Write bits [4:3] of ACCEL_CONFIG with the range code. */
  if (!WriteRegister(ACCEL_CONFIG_, static_cast<uint8_t>(requested_accel_range_))) {
    return false;
  }

  /* Commit the new range and scale only after a successful register write. */
  accel_range_ = requested_accel_range_;
  accel_scale_ = requested_accel_scale_;
  return true;
}

bool Mpu9250::ConfigGyroRange(const GyroRange range) {
  spi_clock_ = SPI_CFG_CLOCK_;

  /*
   * Map the selected range to its corresponding scale factor.
   * Scale factor converts raw 16-bit signed counts to degrees/s:
   *   scale = full_scale_dps / 32767.5
   */
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

  /* Write bits [4:3] of GYRO_CONFIG with the range code. */
  if (!WriteRegister(GYRO_CONFIG_, static_cast<uint8_t>(requested_gyro_range_))) {
    return false;
  }

  /* Commit the new range and scale only after a successful register write. */
  gyro_range_ = requested_gyro_range_;
  gyro_scale_ = requested_gyro_scale_;
  return true;
}

bool Mpu9250::ConfigSrd(const uint8_t srd) {
  spi_clock_ = SPI_CFG_CLOCK_;

  if (has_magnetometer_) {
    /*
     * Temporarily set SRD to 19 (ODR = ~50 Hz) before changing mag mode.
     * The AK8963 is sampled by the I2C master once per IMU sample cycle;
     * at the default SRD=0 (1 kHz) the master bus would be hammered. Using
     * SRD=19 gives 50 Hz IMU rate while we reconfigure the AK8963.
     */
    if (!WriteRegister(SMPLRT_DIV_, 19)) { return false; }

    /*
     * Select AK8963 measurement rate based on the requested IMU SRD:
     *   SRD > 9  → IMU ODR < 100 Hz  → use AK8963 8 Hz  mode (CNT_MEAS1)
     *   SRD ≤ 9  → IMU ODR ≥ 100 Hz → use AK8963 100 Hz mode (CNT_MEAS2)
     */
    const uint8_t ak_mode = (srd > 9) ? AK8963_CNT_MEAS1_ : AK8963_CNT_MEAS2_;

    WriteAk8963Register(AK8963_CNTL1_, AK8963_PWR_DOWN_);
    delay(100);  /* AK8963 requires ≥ 100 ms between mode changes. */

    if (!WriteAk8963Register(AK8963_CNTL1_, ak_mode)) { return false; }
    delay(100);

    /* Dummy-read ST1 + data + ST2 to clear the data-ready latch. */
    if (!ReadAk8963Registers(AK8963_ST1_, sizeof(mag_data_), mag_data_)) {
      return false;
    }
  }

  /* Apply the requested sample rate divider to the IMU. */
  if (!WriteRegister(SMPLRT_DIV_, srd)) { return false; }
  srd_ = srd;
  return true;
}

bool Mpu9250::ConfigDlpfBandwidth(const DlpfBandwidth dlpf) {
  spi_clock_ = SPI_CFG_CLOCK_;

  /* Validate the DLPF setting (default case rejects any undefined value). */
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

  /* The same bandwidth value is written to both the accel (ACCEL_CONFIG2)
   * and the gyro (CONFIG) DLPF registers to keep them in sync. */
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

bool Mpu9250::EnableWom(int16_t threshold_mg, const WomRate wom_rate) {
  /* Validate threshold: hardware supports 4–1020 mg in 4 mg steps. */
  if ((threshold_mg < 4) || (threshold_mg > 1020)) { return false; }

  spi_clock_ = SPI_CFG_CLOCK_;

  /* Power down the magnetometer before resetting — only if present. */
  if (has_magnetometer_) {
    WriteAk8963Register(AK8963_CNTL1_, AK8963_PWR_DOWN_);
  }

  /* Full hardware reset to start from a known state.
   * Return value ignored: H_RESET self-clears, so readback will always fail. */
  WriteRegister(PWR_MGMNT_1_, H_RESET_);
  delay(1);

  /* Clear sleep/standby bits; use internal oscillator (gyro off). */
  if (!WriteRegister(PWR_MGMNT_1_, 0x00)) { return false; }

  /* Disable gyroscope axes to reduce power draw in WOM mode. */
  if (!WriteRegister(PWR_MGMNT_2_, DISABLE_GYRO_)) { return false; }

  /* Set accel DLPF to 184 Hz — recommended bandwidth for WOM comparator. */
  if (!WriteRegister(ACCEL_CONFIG2_,
      static_cast<uint8_t>(DlpfBandwidth::DLPF_BANDWIDTH_184HZ))) {
    return false;
  }

  /* Enable only the Wake-on-Motion interrupt on the INT pin. */
  if (!WriteRegister(INT_ENABLE_, INT_WOM_EN_)) { return false; }

  /* Enable accel hardware intelligence and set comparison mode:
   *   ACCEL_INTEL_EN   = enable the WOM comparator block
   *   ACCEL_INTEL_MODE = compare current sample vs. previous (recommended) */
  if (!WriteRegister(MOT_DETECT_CTRL_,
      (ACCEL_INTEL_EN_ | ACCEL_INTEL_MODE_))) {
    return false;
  }

  /* Convert threshold from mg to register counts (1 LSB = 4 mg). */
  const uint8_t wom_threshold = static_cast<uint8_t>(threshold_mg / 4);
  if (!WriteRegister(WOM_THR_, wom_threshold)) { return false; }

  /* Set the low-power accelerometer output data rate. */
  if (!WriteRegister(LP_ACCEL_ODR_, static_cast<uint8_t>(wom_rate))) {
    return false;
  }

  /* Enter cycle mode: accel wakes at WomRate, checks threshold, sleeps again. */
  if (!WriteRegister(PWR_MGMNT_1_, PWR_CYCLE_WOM_)) { return false; }

  return true;
}

// ---------------------------------------------------------------------------
// FIFO Control (conditional compile)
// ---------------------------------------------------------------------------

#if !defined(DISABLE_MPU9250_FIFO)

bool Mpu9250::EnableFifo() {
  spi_clock_ = SPI_CFG_CLOCK_;
  /* Enable both the FIFO subsystem and the I2C master in USER_CTRL. */
  if (!WriteRegister(USER_CTRL_, FIFO_EN_CTRL_ | I2C_MST_EN_)) { return false; }
  /* Select accel and all three gyro axes as FIFO sources. */
  if (!WriteRegister(FIFO_EN_, FIFO_ACCEL_ | FIFO_GYRO_)) { return false; }
  return true;
}

bool Mpu9250::DisableFifo() {
  spi_clock_ = SPI_CFG_CLOCK_;
  /* Stop all sensors from writing to the FIFO. */
  if (!WriteRegister(FIFO_EN_, 0x00)) { return false; }
  /* Reset the FIFO buffer (FIFO_RESET) while keeping I2C master enabled. */
  WriteRegister(USER_CTRL_, FIFO_RESET_ | I2C_MST_EN_);
  return true;
}

#endif  // !defined(DISABLE_MPU9250_FIFO)

// ---------------------------------------------------------------------------
// Reset
// ---------------------------------------------------------------------------

void Mpu9250::Reset() {
  spi_clock_ = SPI_CFG_CLOCK_;
  /* Power down AK8963 cleanly before the MPU hard reset. */
  if (has_magnetometer_) {
    WriteAk8963Register(AK8963_CNTL1_, AK8963_PWR_DOWN_);
  }
  /* Trigger a full MPU hardware reset (H_RESET self-clears, return ignored). */
  WriteRegister(PWR_MGMNT_1_, H_RESET_);
  delay(1);
}

// ---------------------------------------------------------------------------
// Data Acquisition
// ---------------------------------------------------------------------------

bool Mpu9250::Read() {
  spi_clock_ = SPI_READ_CLOCK_;

  /* Clear stale flags before each read attempt. */
  new_mag_data_ = false;
  new_imu_data_ = false;

  /*
   * Burst-read from INT_STATUS (0x3A) through EXT_SENS_DATA_06 (0x50):
   *   data_buf_[ 0]      = INT_STATUS
   *   data_buf_[ 1– 6]   = ACCEL_XOUT_H/L, ACCEL_YOUT_H/L, ACCEL_ZOUT_H/L
   *   data_buf_[ 7– 8]   = TEMP_OUT_H/L
   *   data_buf_[ 9–14]   = GYRO_XOUT_H/L, GYRO_YOUT_H/L, GYRO_ZOUT_H/L
   *   data_buf_[15]      = AK8963 ST1  (via EXT_SENS_DATA_00)
   *   data_buf_[16–21]   = AK8963 HXL–HZH (measurement data)
   *   data_buf_[22]      = AK8963 ST2  (overflow / data-ready status)
   */
  if (!ReadRegisters(INT_STATUS_, sizeof(data_buf_), data_buf_)) {
    return false;
  }

#if !defined(DISABLE_MPU9250_FIFO)
  /* Check for FIFO overflow (bit 4 of INT_STATUS). */
  fifo_overflow_ = (data_buf_[0] & FIFO_OVERFLOW_INT_);
#endif

  /* Bit 0 of INT_STATUS signals that all sensor registers have been updated. */
  new_imu_data_ = (data_buf_[0] & RAW_DATA_RDY_INT_);
  if (!new_imu_data_) {
    return false;
  }

  /* ---- Unpack raw 16-bit counts (big-endian, MSB first) ---- */
  accel_cnts_[0] = static_cast<int16_t>(data_buf_[1])  << 8 | data_buf_[2];
  accel_cnts_[1] = static_cast<int16_t>(data_buf_[3])  << 8 | data_buf_[4];
  accel_cnts_[2] = static_cast<int16_t>(data_buf_[5])  << 8 | data_buf_[6];
  temp_cnts_     = static_cast<int16_t>(data_buf_[7])  << 8 | data_buf_[8];
  gyro_cnts_[0]  = static_cast<int16_t>(data_buf_[9])  << 8 | data_buf_[10];
  gyro_cnts_[1]  = static_cast<int16_t>(data_buf_[11]) << 8 | data_buf_[12];
  gyro_cnts_[2]  = static_cast<int16_t>(data_buf_[13]) << 8 | data_buf_[14];

  if (has_magnetometer_) {
    /* ST1 bit 0 = DRDY (data ready). */
    new_mag_data_ = (data_buf_[15] & AK8963_DATA_RDY_INT_);
    /* AK8963 outputs are little-endian (LSB first), unlike the MPU registers. */
    mag_cnts_[0] = static_cast<int16_t>(data_buf_[17]) << 8 | data_buf_[16];
    mag_cnts_[1] = static_cast<int16_t>(data_buf_[19]) << 8 | data_buf_[18];
    mag_cnts_[2] = static_cast<int16_t>(data_buf_[21]) << 8 | data_buf_[20];
    /* ST2 bit 3 = HOFL: magnetic sensor overflow (field exceeds ±4912 µT). */
    mag_sensor_overflow_ = (data_buf_[22] & AK8963_HOFL_);
    if (mag_sensor_overflow_) {
      new_mag_data_ = false;  /* Discard overflowed data as unreliable. */
    }
  } else {
    new_mag_data_        = false;
    mag_sensor_overflow_ = false;
  }

  /*
   * ---- Convert counts to SI units and apply axis rotation ----
   *
   * The MPU-9250 PCB axes do not necessarily align with the desired body frame.
   * The mapping below rotates the sensor frame so that:
   *   Output X  = Sensor Y  (forward)
   *   Output Y  = Sensor X  (right)
   *   Output Z  = -Sensor Z (down, converting from MPU's Z-up to Z-down)
   *
   * Adjust this mapping if the sensor is mounted differently on your PCB.
   */
  accel_.x = (static_cast<float>(accel_cnts_[1]) * accel_scale_) * G_TO_MPS2;
  accel_.y = (static_cast<float>(accel_cnts_[0]) * accel_scale_) * G_TO_MPS2;
  accel_.z = (static_cast<float>(accel_cnts_[2]) * accel_scale_ * -1.0f) * G_TO_MPS2;

  /*
   * Temperature formula from the MPU-9250 datasheet §4.18:
   *   T (°C) = (TEMP_OUT - RoomTemp_Offset) / Temp_Sensitivity + 21
   * where RoomTemp_Offset = 0 (defined at 21 °C) and Temp_Sensitivity = 333.87 LSB/°C.
   */
  temp_ = (static_cast<float>(temp_cnts_) - 21.0f) / TEMP_SCALE_ + 21.0f;

  gyro_.x = (static_cast<float>(gyro_cnts_[1]) * gyro_scale_) * DEG_TO_RAD_;
  gyro_.y = (static_cast<float>(gyro_cnts_[0]) * gyro_scale_) * DEG_TO_RAD_;
  gyro_.z = (static_cast<float>(gyro_cnts_[2]) * gyro_scale_ * -1.0f) * DEG_TO_RAD_;

  /* Magnetometer values are only updated when fresh data is available. */
  if (new_mag_data_) {
    mag_.x = static_cast<float>(mag_cnts_[0]) * mag_scale_[0];
    mag_.y = static_cast<float>(mag_cnts_[1]) * mag_scale_[1];
    mag_.z = static_cast<float>(mag_cnts_[2]) * mag_scale_[2];
  }

  return true;
}

// ---------------------------------------------------------------------------
// FIFO Data Acquisition (conditional compile)
// ---------------------------------------------------------------------------

#if !defined(DISABLE_MPU9250_FIFO)

int8_t Mpu9250::ReadFifo() {
  spi_clock_ = SPI_READ_CLOCK_;

  /* Read the two-byte FIFO byte count (bits [12:0] are valid). */
  if (!ReadRegisters(FIFO_COUNT_, 2, data_buf_)) { return -1; }
  fifo_bytes_      = static_cast<int16_t>(data_buf_[0] & 0x0F) << 8 | data_buf_[1];
  fifo_num_frames_ = fifo_bytes_ / FIFO_FRAME_SIZE_;

  /* Clamp to our internal buffer capacity to avoid overflow. */
  const int8_t frames_to_read = std::min(fifo_num_frames_, FIFO_MAX_NUM_FRAMES_);

  for (int8_t i = 0; i < frames_to_read; i++) {
    /*
     * Each FIFO frame is exactly 12 bytes:
     *   bytes [0–5]  = ACCEL X/Y/Z (big-endian, 2 bytes each)
     *   bytes [6–11] = GYRO  X/Y/Z (big-endian, 2 bytes each)
     * Temperature and magnetometer are NOT stored in the FIFO.
     */
    if (!ReadRegisters(FIFO_READ_, FIFO_FRAME_SIZE_, data_buf_)) { return -1; }

    /* Unpack raw 16-bit counts (big-endian). */
    accel_cnts_[0] = static_cast<int16_t>(data_buf_[0]) << 8 | data_buf_[1];
    accel_cnts_[1] = static_cast<int16_t>(data_buf_[2]) << 8 | data_buf_[3];
    accel_cnts_[2] = static_cast<int16_t>(data_buf_[4]) << 8 | data_buf_[5];
    gyro_cnts_[0]  = static_cast<int16_t>(data_buf_[6]) << 8 | data_buf_[7];
    gyro_cnts_[1]  = static_cast<int16_t>(data_buf_[8]) << 8 | data_buf_[9];
    gyro_cnts_[2]  = static_cast<int16_t>(data_buf_[10]) << 8 | data_buf_[11];

    /* Apply same axis rotation as Read() — see comments there for rationale. */
    fifo_ax_[i] = (static_cast<float>(accel_cnts_[1]) * accel_scale_) * G_TO_MPS2;
    fifo_ay_[i] = (static_cast<float>(accel_cnts_[0]) * accel_scale_) * G_TO_MPS2;
    fifo_az_[i] = (static_cast<float>(accel_cnts_[2]) * accel_scale_ * -1.0f) * G_TO_MPS2;
    fifo_gx_[i] = (static_cast<float>(gyro_cnts_[1]) * gyro_scale_) * DEG_TO_RAD_;
    fifo_gy_[i] = (static_cast<float>(gyro_cnts_[0]) * gyro_scale_) * DEG_TO_RAD_;
    fifo_gz_[i] = (static_cast<float>(gyro_cnts_[2]) * gyro_scale_ * -1.0f) * DEG_TO_RAD_;
  }

  return fifo_num_frames_;
}

int8_t Mpu9250::fifo_accel_x_mps2(float *data, const std::size_t len) {
  if (!data) { return -1; }
  int8_t cpy_len = std::min(fifo_num_frames_, static_cast<int8_t>(len));
  memcpy(data, fifo_ax_, cpy_len * sizeof(float));
  return cpy_len;
}

int8_t Mpu9250::fifo_accel_y_mps2(float *data, const std::size_t len) {
  if (!data) { return -1; }
  int8_t cpy_len = std::min(fifo_num_frames_, static_cast<int8_t>(len));
  memcpy(data, fifo_ay_, cpy_len * sizeof(float));
  return cpy_len;
}

int8_t Mpu9250::fifo_accel_z_mps2(float *data, const std::size_t len) {
  if (!data) { return -1; }
  int8_t cpy_len = std::min(fifo_num_frames_, static_cast<int8_t>(len));
  memcpy(data, fifo_az_, cpy_len * sizeof(float));
  return cpy_len;
}

int8_t Mpu9250::fifo_gyro_x_radps(float *data, const std::size_t len) {
  if (!data) { return -1; }
  int8_t cpy_len = std::min(fifo_num_frames_, static_cast<int8_t>(len));
  memcpy(data, fifo_gx_, cpy_len * sizeof(float));
  return cpy_len;
}

int8_t Mpu9250::fifo_gyro_y_radps(float *data, const std::size_t len) {
  if (!data) { return -1; }
  int8_t cpy_len = std::min(fifo_num_frames_, static_cast<int8_t>(len));
  memcpy(data, fifo_gy_, cpy_len * sizeof(float));
  return cpy_len;
}

int8_t Mpu9250::fifo_gyro_z_radps(float *data, const std::size_t len) {
  if (!data) { return -1; }
  int8_t cpy_len = std::min(fifo_num_frames_, static_cast<int8_t>(len));
  memcpy(data, fifo_gz_, cpy_len * sizeof(float));
  return cpy_len;
}

#endif  // !defined(DISABLE_MPU9250_FIFO)

// ---------------------------------------------------------------------------
// Low-Level Communication — MPU-9250 Registers
// ---------------------------------------------------------------------------

bool Mpu9250::WriteRegister(uint8_t reg, uint8_t data) {
  /*
   * Write the byte to the register, then read it back after a short delay
   * to verify the write succeeded. Note: self-clearing registers (e.g.
   * H_RESET, INT_STATUS) will always fail this check — callers that write
   * to such registers must ignore the return value.
   */
  if (iface_ == Interface::I2C) {
    i2c_->beginTransmission(dev_);
    i2c_->write(reg);
    i2c_->write(data);
    byte err = i2c_->endTransmission();
    if (err != 0) {
      Serial.printf(" -> WriteRegister(0x%02X, 0x%02X): "
                    "I2C endTransmission error %d\n", reg, data, err);
      return false;
    }
  } else {
    spi_->beginTransaction(SPISettings(spi_clock_, MSBFIRST, SPI_MODE3));
    digitalWrite(dev_, LOW);
    spi_->transfer(reg);          /* Send register address (write, MSB=0) */
    spi_->transfer(data);         /* Send payload byte */
    digitalWrite(dev_, HIGH);
    spi_->endTransaction();
  }

  /* Wait ≥ 1 ms for the register to settle before reading back. */
  delay(10);

  uint8_t ret_val;
  if (!ReadRegisters(reg, sizeof(ret_val), &ret_val)) {
    Serial.printf(" -> WriteRegister(0x%02X, 0x%02X): readback failed!\n", reg, data);
    return false;
  }
  if (data == ret_val) {
    return true;
  }
  Serial.printf(" -> WriteRegister(0x%02X, 0x%02X): "
                "validation mismatch! Read back: 0x%02X\n", reg, data, ret_val);
  return false;
}

bool Mpu9250::ReadRegisters(uint8_t reg, uint8_t count, uint8_t *data) {
  if (iface_ == Interface::I2C) {
    /* Set the register pointer without a STOP condition (repeated START). */
    i2c_->beginTransmission(dev_);
    i2c_->write(reg);
    i2c_->endTransmission(false);
    /* Request `count` bytes; the sensor auto-increments the register pointer. */
    bytes_rx_ = i2c_->requestFrom(static_cast<uint8_t>(dev_), count);
    if (bytes_rx_ == count) {
      for (std::size_t i = 0; i < count; i++) {
        data[i] = i2c_->read();
      }
      return true;
    }
    return false;
  } else {
    /* SPI burst read: OR the MSB of the address byte to signal a read. */
    spi_->beginTransaction(SPISettings(spi_clock_, MSBFIRST, SPI_MODE3));
    digitalWrite(dev_, LOW);
    spi_->transfer(reg | SPI_READ_);  /* Address byte with read flag */
    spi_->transfer(data, count);      /* In-place receive into data buffer */
    digitalWrite(dev_, HIGH);
    spi_->endTransaction();
    return true;
  }
}

// ---------------------------------------------------------------------------
// Low-Level Communication — AK8963 via MPU I2C Master
// ---------------------------------------------------------------------------

bool Mpu9250::WriteAk8963Register(uint8_t reg, uint8_t data) {
  /*
   * The AK8963 sits on the MPU's auxiliary I2C bus (not accessible directly).
   * To write a register, we program the MPU's I2C Slave 0 channel, which
   * autonomously performs the transaction at the next sample cycle:
   *   1. Set slave 0 address to AK8963 (write mode, MSB=0)
   *   2. Set slave 0 sub-address to the target AK8963 register
   *   3. Put the data byte into the slave 0 data-out register
   *   4. Enable slave 0 with a 1-byte transfer length
   * The write is then verified by reading back the register via ReadAk8963Registers().
   */
  uint8_t ret_val;
  if (!WriteRegister(I2C_SLV0_ADDR_, AK8963_I2C_ADDR_))        { return false; }
  if (!WriteRegister(I2C_SLV0_REG_,  reg))                      { return false; }
  if (!WriteRegister(I2C_SLV0_DO_,   data))                     { return false; }
  if (!WriteRegister(I2C_SLV0_CTRL_, I2C_SLV0_EN_ | sizeof(data))) { return false; }

  /* Read back the AK8963 register to verify the write. */
  if (!ReadAk8963Registers(reg, sizeof(ret_val), &ret_val)) { return false; }
  return (data == ret_val);
}

bool Mpu9250::ReadAk8963Registers(uint8_t reg, uint8_t count, uint8_t *data) {
  /*
   * To read from the AK8963, we configure the MPU's I2C Slave 0 channel
   * in read mode. The MPU performs the I2C transaction and stores the result
   * in the EXT_SENS_DATA registers, which we then read over the main bus:
   *   1. Set slave 0 address with the read flag (MSB=1)
   *   2. Set slave 0 sub-address to the target AK8963 register
   *   3. Enable slave 0 with the requested byte count
   *   4. Wait 1 ms for the transfer to complete
   *   5. Read the result from EXT_SENS_DATA_00 onward
   */
  if (!WriteRegister(I2C_SLV0_ADDR_, AK8963_I2C_ADDR_ | I2C_READ_FLAG_)) {
    return false;
  }
  if (!WriteRegister(I2C_SLV0_REG_,  reg))                          { return false; }
  if (!WriteRegister(I2C_SLV0_CTRL_, I2C_SLV0_EN_ | count))         { return false; }

  delay(1);  /* Allow time for the I2C master to complete the transaction. */
  return ReadRegisters(EXT_SENS_DATA_00_, count, data);
}
