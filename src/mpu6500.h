/**
 * @file mpu6500.h
 * @brief Simplified 6-DoF driver for the InvenSense MPU-6500 IMU.
 *
 * This is a stripped-down version of the MPU-9250 driver targeting pure
 * 6-DoF (accelerometer + gyroscope) operation. All AK8963 magnetometer
 * code, internal I2C master registers, auxiliary-bus scanning, and related
 * overhead have been removed, resulting in a smaller binary and simpler
 * initialization sequence.
 *
 * Also compatible with MPU-9250 / MPU-9255 clones that report WHO_AM_I=0x70
 * (no functional magnetometer present).
 *
 * Features:
 *  - I2C and SPI communication
 *  - Configurable accelerometer range (±2 / 4 / 8 / 16 g)
 *  - Configurable gyroscope range (±250 / 500 / 1000 / 2000 °/s)
 *  - Configurable DLPF bandwidth (5–184 Hz)
 *  - Configurable sample rate divider (SRD)
 *  - Data-ready interrupt (DRDY)
 *  - Wake-on-Motion (WOM) with hardware comparator
 *  - Optional FIFO buffering (disable with -DDISABLE_MPU6500_FIFO)
 *
 * Output units: acceleration in m/s², angular rate in rad/s, temperature in °C.
 *
 * Usage:
 * @code
 *   Mpu6500 imu(&Wire, Mpu6500::I2cAddr::I2C_ADDR_PRIM);
 *   imu.Begin();
 *   if (imu.Read()) { float ax = imu.accel_x_mps2(); }
 * @endcode
 */

#ifndef MPU6500_SRC_MPU6500_H_
#define MPU6500_SRC_MPU6500_H_

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <cstddef>
#include <cstdint>
#include "linear_algebra.h"

class Mpu6500 {
 public:
  // -------------------------------------------------------------------------
  // Configuration Enumerations
  // -------------------------------------------------------------------------

  /** @brief I2C device address, selected by the AD0 pin level. */
  enum class I2cAddr : uint8_t {
    I2C_ADDR_PRIM = 0x68,  ///< AD0 = LOW  (default)
    I2C_ADDR_SEC  = 0x69   ///< AD0 = HIGH
  };

  /**
   * @brief Digital Low-Pass Filter (DLPF) bandwidth for accel and gyro.
   * Lower bandwidth = less noise, higher latency.
   */
  enum class DlpfBandwidth : uint8_t {
    DLPF_BANDWIDTH_184HZ = 0x01,  ///< ~184 Hz cutoff, ~2.9 ms delay
    DLPF_BANDWIDTH_92HZ  = 0x02,  ///< ~92 Hz cutoff,  ~3.9 ms delay
    DLPF_BANDWIDTH_41HZ  = 0x03,  ///< ~41 Hz cutoff,  ~5.9 ms delay
    DLPF_BANDWIDTH_20HZ  = 0x04,  ///< ~20 Hz cutoff,  ~9.9 ms delay
    DLPF_BANDWIDTH_10HZ  = 0x05,  ///< ~10 Hz cutoff,  ~17.8 ms delay
    DLPF_BANDWIDTH_5HZ   = 0x06   ///< ~5 Hz cutoff,   ~33.9 ms delay
  };

  /**
   * @brief Accelerometer full-scale range.
   * Register values written to ACCEL_CONFIG bits [4:3].
   */
  enum class AccelRange : uint8_t {
    ACCEL_RANGE_2G  = 0x00,  ///< ±2 g  — 16384 LSB/g
    ACCEL_RANGE_4G  = 0x08,  ///< ±4 g  — 8192 LSB/g
    ACCEL_RANGE_8G  = 0x10,  ///< ±8 g  — 4096 LSB/g
    ACCEL_RANGE_16G = 0x18   ///< ±16 g — 2048 LSB/g
  };

  /**
   * @brief Gyroscope full-scale range.
   * Register values written to GYRO_CONFIG bits [4:3].
   */
  enum class GyroRange : uint8_t {
    GYRO_RANGE_250DPS  = 0x00,  ///< ±250  °/s — 131.0 LSB/(°/s)
    GYRO_RANGE_500DPS  = 0x08,  ///< ±500  °/s — 65.5 LSB/(°/s)
    GYRO_RANGE_1000DPS = 0x10,  ///< ±1000 °/s — 32.8 LSB/(°/s)
    GYRO_RANGE_2000DPS = 0x18   ///< ±2000 °/s — 16.4 LSB/(°/s)
  };

  /**
   * @brief Wake-on-Motion accelerometer output data rate.
   * Controls how often the WOM comparator wakes to sample the accelerometer.
   */
  enum class WomRate : uint8_t {
    WOM_RATE_0_24HZ  = 0x00,
    WOM_RATE_0_49HZ  = 0x01,
    WOM_RATE_0_98HZ  = 0x02,
    WOM_RATE_1_95HZ  = 0x03,
    WOM_RATE_3_91HZ  = 0x04,
    WOM_RATE_7_81HZ  = 0x05,
    WOM_RATE_15_63HZ = 0x06,
    WOM_RATE_31_25HZ = 0x07,
    WOM_RATE_62_50HZ = 0x08,
    WOM_RATE_125HZ   = 0x09,
    WOM_RATE_250HZ   = 0x0A,
    WOM_RATE_500HZ   = 0x0B
  };

  // -------------------------------------------------------------------------
  // Constructors and Runtime Configuration
  // -------------------------------------------------------------------------

  /** @brief Default constructor. Must call Config() before Begin(). */
  Mpu6500() = default;

  /**
   * @brief Construct for I2C.
   * @param i2c  TwoWire bus instance.
   * @param addr I2C device address.
   */
  Mpu6500(TwoWire *i2c, const I2cAddr addr)
      : i2c_(i2c), dev_(static_cast<uint8_t>(addr)), iface_(Interface::I2C) {}

  /**
   * @brief Construct for SPI.
   * @param spi SPI bus instance.
   * @param cs  Chip-select GPIO pin number.
   */
  Mpu6500(SPIClass *spi, const uint8_t cs)
      : spi_(spi), dev_(cs), iface_(Interface::SPI) {}

  /** @brief Runtime I2C configuration (alternative to constructor). */
  void Config(TwoWire *i2c, const I2cAddr addr);

  /** @brief Runtime SPI configuration (alternative to constructor). */
  void Config(SPIClass *spi, const uint8_t cs);

  // -------------------------------------------------------------------------
  // Initialization and Sensor Configuration
  // -------------------------------------------------------------------------

  /**
   * @brief Initialize the IMU.
   *
   * Verifies WHO_AM_I, resets the device, and applies defaults:
   *  - Accel range:  ±16 g
   *  - Gyro range:   ±2000 °/s
   *  - DLPF:         184 Hz
   *  - SRD:          0  (1 kHz output rate)
   *
   * @return true on success, false if communication fails or chip ID is wrong.
   */
  [[nodiscard]] bool Begin();

  /**
   * @brief Enable the Data-Ready interrupt (50 µs pulse on INT pin).
   * @return true on success.
   */
  [[nodiscard]] bool EnableDrdyInt();

  /**
   * @brief Disable the Data-Ready interrupt.
   * @return true on success.
   */
  [[nodiscard]] bool DisableDrdyInt();

  /**
   * @brief Set the accelerometer full-scale range.
   * @param range Desired range from the AccelRange enum.
   * @return true on success.
   */
  [[nodiscard]] bool ConfigAccelRange(const AccelRange range);

  /** @brief Return the currently active accelerometer range. */
  inline AccelRange accel_range() const { return accel_range_; }

  /**
   * @brief Set the gyroscope full-scale range.
   * @param range Desired range from the GyroRange enum.
   * @return true on success.
   */
  [[nodiscard]] bool ConfigGyroRange(const GyroRange range);

  /** @brief Return the currently active gyroscope range. */
  inline GyroRange gyro_range() const { return gyro_range_; }

  /**
   * @brief Set the sample rate divider.
   * Output data rate = 1000 Hz / (1 + srd).
   * @param srd Divider value (0 = 1 kHz, 9 = 100 Hz, 99 = 10 Hz).
   * @return true on success.
   */
  [[nodiscard]] bool ConfigSrd(const uint8_t srd);

  /** @brief Return the current sample rate divider value. */
  inline uint8_t srd() const { return srd_; }

  /**
   * @brief Set the DLPF bandwidth (applied simultaneously to accel and gyro).
   * @param dlpf Desired bandwidth from the DlpfBandwidth enum.
   * @return true on success.
   */
  [[nodiscard]] bool ConfigDlpfBandwidth(const DlpfBandwidth dlpf);

  /** @brief Return the currently active DLPF bandwidth setting. */
  inline DlpfBandwidth dlpf_bandwidth() const { return dlpf_bandwidth_; }

  /**
   * @brief Enter Wake-on-Motion low-power mode.
   *
   * The gyroscope is powered down. The accelerometer wakes periodically at
   * @p wom_rate, compares against the previous sample, and fires the INT pin
   * if the delta exceeds @p threshold_mg. Call Begin() to exit WOM mode.
   *
   * @param threshold_mg  Motion threshold in milli-g (4–1020 mg, 4 mg steps).
   * @param wom_rate      Accel sample rate during WOM.
   * @return true on success, false if threshold is out of range.
   */
  [[nodiscard]] bool EnableWom(int16_t threshold_mg, const WomRate wom_rate);

  // -------------------------------------------------------------------------
  // FIFO Control (disable with -DDISABLE_MPU6500_FIFO)
  // -------------------------------------------------------------------------

  #if !defined(DISABLE_MPU6500_FIFO)
  /**
   * @brief Enable FIFO buffering for accelerometer and gyroscope.
   * Use ReadFifo() to drain the buffer in bulk.
   * @return true on success.
   */
  [[nodiscard]] bool EnableFifo();

  /**
   * @brief Disable FIFO buffering and reset the FIFO.
   * @return true on success.
   */
  [[nodiscard]] bool DisableFifo();
  #endif

  // -------------------------------------------------------------------------
  // Reset and Data Acquisition
  // -------------------------------------------------------------------------

  /**
   * @brief Hardware reset. All registers return to power-on defaults.
   * Call Begin() before reading data again.
   */
  void Reset();

  /**
   * @brief Read the latest IMU data registers.
   *
   * Performs a 15-byte burst read (INT_STATUS + ACCEL + TEMP + GYRO).
   * Returns false if no new data is available or communication fails.
   *
   * @return true if new data was successfully read.
   */
  [[nodiscard]] bool Read();

  #if !defined(DISABLE_MPU6500_FIFO)
  /**
   * @brief Drain the FIFO into the internal frame buffers.
   *
   * Reads up to FIFO_MAX_SIZE() frames. Use fifo_accel_x_mps2() etc.
   * to copy the data out afterward.
   *
   * @return Number of frames read (≥ 0), or -1 on communication error.
   */
  [[nodiscard]] int8_t ReadFifo();
  #endif

  // -------------------------------------------------------------------------
  // IMU Data Accessors
  // -------------------------------------------------------------------------

  /** @brief True if the last Read() returned new IMU data. */
  inline bool new_imu_data() const { return new_imu_data_; }

  /** @brief Accelerometer X-axis [m/s²]. */
  inline float    accel_x_mps2() const { return accel_.x; }
  /** @brief Accelerometer Y-axis [m/s²]. */
  inline float    accel_y_mps2() const { return accel_.y; }
  /** @brief Accelerometer Z-axis [m/s²]. */
  inline float    accel_z_mps2() const { return accel_.z; }
  /** @brief All accelerometer axes as Vector3f [m/s²]. */
  inline Vector3f accel_mps2()   const { return accel_; }

  /** @brief Gyroscope X-axis [rad/s]. */
  inline float    gyro_x_radps() const { return gyro_.x; }
  /** @brief Gyroscope Y-axis [rad/s]. */
  inline float    gyro_y_radps() const { return gyro_.y; }
  /** @brief Gyroscope Z-axis [rad/s]. */
  inline float    gyro_z_radps() const { return gyro_.z; }
  /** @brief All gyroscope axes as Vector3f [rad/s]. */
  inline Vector3f gyro_radps()   const { return gyro_; }

  /** @brief Die temperature [°C]. */
  inline float die_temp_c() const { return temp_; }

  // -------------------------------------------------------------------------
  // FIFO Data Accessors
  // -------------------------------------------------------------------------

  #if !defined(DISABLE_MPU6500_FIFO)
  /**
   * @brief Copy FIFO accel X frames into a caller-provided buffer.
   * @param data  Destination float array. Must not be null.
   * @param len   Capacity of the buffer (number of floats).
   * @return Number of values copied, or -1 if data is null.
   */
  int8_t fifo_accel_x_mps2(float *data, std::size_t len);
  /** @brief Copy FIFO accel Y frames. @see fifo_accel_x_mps2 */
  int8_t fifo_accel_y_mps2(float *data, std::size_t len);
  /** @brief Copy FIFO accel Z frames. @see fifo_accel_x_mps2 */
  int8_t fifo_accel_z_mps2(float *data, std::size_t len);
  /** @brief Copy FIFO gyro X frames [rad/s]. @see fifo_accel_x_mps2 */
  int8_t fifo_gyro_x_radps(float *data, std::size_t len);
  /** @brief Copy FIFO gyro Y frames [rad/s]. @see fifo_accel_x_mps2 */
  int8_t fifo_gyro_y_radps(float *data, std::size_t len);
  /** @brief Copy FIFO gyro Z frames [rad/s]. @see fifo_accel_x_mps2 */
  int8_t fifo_gyro_z_radps(float *data, std::size_t len);

  /** @brief Maximum frames stored internally (512-byte FIFO / 12 bytes). */
  static constexpr int8_t FIFO_MAX_SIZE() { return FIFO_MAX_NUM_FRAMES_; }

  /** @brief True if a FIFO overflow was detected in the last Read(). */
  inline bool fifo_overflow() const { return fifo_overflow_; }
  #endif

 private:
  // -------------------------------------------------------------------------
  // Communication Interface
  // -------------------------------------------------------------------------

  enum class Interface : int8_t { SPI, I2C };

  TwoWire  *i2c_   = nullptr;
  SPIClass *spi_   = nullptr;
  uint8_t   dev_   = 0;        ///< I2C address or SPI CS pin
  Interface iface_ = Interface::I2C;
  uint8_t   bytes_rx_  = 0;
  int32_t   spi_clock_ = SPI_CFG_CLOCK_;

  /*
   * MPU-6500 supports 1 MHz for register writes and up to 20 MHz for reads;
   * capped at 15 MHz for reliable operation on clone hardware.
   */
  static constexpr int32_t SPI_CFG_CLOCK_  = 1000000;   ///< 1 MHz  — config writes
  static constexpr int32_t SPI_READ_CLOCK_ = 15000000;  ///< 15 MHz — data reads
  static constexpr uint8_t SPI_READ_       = 0x80;      ///< SPI read flag (OR onto address)

  // -------------------------------------------------------------------------
  // Runtime Configuration State
  // -------------------------------------------------------------------------

  AccelRange    accel_range_          = AccelRange::ACCEL_RANGE_16G;
  AccelRange    requested_accel_range_ = AccelRange::ACCEL_RANGE_16G;
  GyroRange     gyro_range_           = GyroRange::GYRO_RANGE_2000DPS;
  GyroRange     requested_gyro_range_  = GyroRange::GYRO_RANGE_2000DPS;
  DlpfBandwidth dlpf_bandwidth_       = DlpfBandwidth::DLPF_BANDWIDTH_184HZ;
  DlpfBandwidth requested_dlpf_       = DlpfBandwidth::DLPF_BANDWIDTH_184HZ;

  float   accel_scale_          = 0.0f;  ///< Active accel scale [g/LSB]
  float   requested_accel_scale_ = 0.0f;
  float   gyro_scale_           = 0.0f;  ///< Active gyro scale [dps/LSB]
  float   requested_gyro_scale_  = 0.0f;
  uint8_t srd_                  = 0;
  uint8_t who_am_i_             = 0;     ///< Value read from WHO_AM_I during Begin()

  // -------------------------------------------------------------------------
  // Sensor Data
  // -------------------------------------------------------------------------

  bool new_imu_data_ = false;

  /*
   * Burst read buffer layout (15 bytes, starting at INT_STATUS 0x3A):
   *   [0]      = INT_STATUS
   *   [1–6]    = ACCEL X/Y/Z (big-endian, 2 bytes each)
   *   [7–8]    = TEMP  (big-endian)
   *   [9–14]   = GYRO  X/Y/Z (big-endian, 2 bytes each)
   */
  uint8_t data_buf_[15] = {0};
  int16_t accel_cnts_[3] = {0};  ///< Raw accel counts [X, Y, Z]
  int16_t gyro_cnts_[3]  = {0};  ///< Raw gyro counts  [X, Y, Z]
  int16_t temp_cnts_     = 0;    ///< Raw temperature counts

  Vector3f accel_;       ///< Converted accel [m/s²]
  Vector3f gyro_;        ///< Converted gyro  [rad/s]
  float    temp_ = 0.0f; ///< Converted temperature [°C]

  /// MPU-6500 temperature formula: T = (counts - 21) / TEMP_SCALE_ + 21
  static constexpr float TEMP_SCALE_ = 333.87f;

  // -------------------------------------------------------------------------
  // FIFO State (conditional compile)
  // -------------------------------------------------------------------------

  #if !defined(DISABLE_MPU6500_FIFO)
  bool    fifo_overflow_   = false;  ///< Set when FIFO overflow bit is detected
  int8_t  fifo_num_frames_ = 0;      ///< Frame count from last ReadFifo()
  int16_t fifo_bytes_      = 0;      ///< Raw byte count from FIFO_COUNT

  /// Each FIFO frame = 6 accel + 6 gyro bytes (temp and mag are not in FIFO)
  static constexpr int8_t FIFO_FRAME_SIZE_     = 12;
  static constexpr int8_t FIFO_MAX_NUM_FRAMES_ = 42;  ///< 512 / 12 = 42 frames max

  float fifo_ax_[FIFO_MAX_NUM_FRAMES_] = {0};
  float fifo_ay_[FIFO_MAX_NUM_FRAMES_] = {0};
  float fifo_az_[FIFO_MAX_NUM_FRAMES_] = {0};
  float fifo_gx_[FIFO_MAX_NUM_FRAMES_] = {0};
  float fifo_gy_[FIFO_MAX_NUM_FRAMES_] = {0};
  float fifo_gz_[FIFO_MAX_NUM_FRAMES_] = {0};
  #endif

  // -------------------------------------------------------------------------
  // MPU-6500 Register Map
  // -------------------------------------------------------------------------

  // Power management
  static constexpr uint8_t PWR_MGMNT_1_ = 0x6B;  ///< Power management 1
  static constexpr uint8_t PWR_MGMNT_2_ = 0x6C;  ///< Power management 2
  static constexpr uint8_t H_RESET_     = 0x80;  ///< Device reset bit (self-clears)
  static constexpr uint8_t CLKSEL_PLL_  = 0x01;  ///< Clock: auto-select PLL

  // Identity
  static constexpr uint8_t WHOAMI_         = 0x75;  ///< WHO_AM_I register
  static constexpr uint8_t WHOAMI_MPU6500_ = 0x70;  ///< Expected for MPU-6500
  static constexpr uint8_t WHOAMI_MPU9250_ = 0x71;  ///< Accepted (IMU-only mode)
  static constexpr uint8_t WHOAMI_MPU9255_ = 0x73;  ///< Accepted (IMU-only mode)

  // Sensor configuration
  static constexpr uint8_t ACCEL_CONFIG_  = 0x1C;  ///< Accel full-scale range
  static constexpr uint8_t ACCEL_CONFIG2_ = 0x1D;  ///< Accel DLPF
  static constexpr uint8_t GYRO_CONFIG_   = 0x1B;  ///< Gyro full-scale range
  static constexpr uint8_t CONFIG_        = 0x1A;  ///< Gyro DLPF
  static constexpr uint8_t SMPLRT_DIV_   = 0x19;  ///< Sample rate divider

  // Interrupt control
  static constexpr uint8_t INT_PIN_CFG_      = 0x37;  ///< INT pin configuration
  static constexpr uint8_t INT_ENABLE_       = 0x38;  ///< Interrupt enable
  static constexpr uint8_t INT_DISABLE_      = 0x00;  ///< Clear all interrupts
  static constexpr uint8_t INT_PULSE_50US_   = 0x00;  ///< 50 µs pulse, active-high
  static constexpr uint8_t INT_RAW_RDY_EN_   = 0x01;  ///< Raw data-ready interrupt enable
  static constexpr uint8_t INT_STATUS_       = 0x3A;  ///< Interrupt status (read to clear)
  static constexpr uint8_t RAW_DATA_RDY_INT_ = 0x01;  ///< Data-ready bit in INT_STATUS

  // User control (used only for FIFO enable)
  static constexpr uint8_t USER_CTRL_ = 0x6A;

  // Wake-on-Motion
  static constexpr uint8_t INT_WOM_EN_       = 0x40;  ///< WOM interrupt enable
  static constexpr uint8_t DISABLE_GYRO_     = 0x07;  ///< PWR_MGMNT_2: power down gyro axes
  static constexpr uint8_t MOT_DETECT_CTRL_  = 0x69;  ///< Motion detection control
  static constexpr uint8_t ACCEL_INTEL_EN_   = 0x80;  ///< Enable hardware comparator
  static constexpr uint8_t ACCEL_INTEL_MODE_ = 0x40;  ///< Compare vs. previous sample
  static constexpr uint8_t LP_ACCEL_ODR_     = 0x1E;  ///< Low-power accel ODR
  static constexpr uint8_t WOM_THR_          = 0x1F;  ///< WOM threshold (4 mg/LSB)
  static constexpr uint8_t PWR_CYCLE_WOM_    = 0x20;  ///< Cycle mode for WOM

  // FIFO registers
  #if !defined(DISABLE_MPU6500_FIFO)
  static constexpr uint8_t FIFO_EN_CTRL_      = 0x40;  ///< USER_CTRL bit: enable FIFO
  static constexpr uint8_t FIFO_EN_           = 0x23;  ///< FIFO data source select
  static constexpr uint8_t FIFO_GYRO_         = 0x70;  ///< Enable gyro X/Y/Z in FIFO
  static constexpr uint8_t FIFO_ACCEL_        = 0x08;  ///< Enable accel in FIFO
  static constexpr uint8_t FIFO_COUNT_        = 0x72;  ///< FIFO byte count (MSB at 0x72)
  static constexpr uint8_t FIFO_READ_         = 0x74;  ///< FIFO data register
  static constexpr uint8_t FIFO_OVERFLOW_INT_ = 0x10;  ///< INT_STATUS overflow bit
  static constexpr uint8_t FIFO_RESET_        = 0x04;  ///< USER_CTRL bit: reset FIFO
  #endif

  // -------------------------------------------------------------------------
  // Private Communication Methods
  // -------------------------------------------------------------------------

  /**
   * @brief Write a byte to a register and verify by readback.
   *
   * Not suitable for self-clearing registers (e.g. H_RESET). Use
   * WriteRegisterNoVerify() for those.
   *
   * @return true if write succeeded and readback matched.
   */
  bool WriteRegister(uint8_t reg, uint8_t data);

  /**
   * @brief Write a byte to a register WITHOUT readback verification.
   *
   * Used for registers that self-clear or transition to a different value
   * immediately after the write (e.g. PWR_MGMNT_1 H_RESET bit).
   */
  void WriteRegisterNoVerify(uint8_t reg, uint8_t data);

  /**
   * @brief Read one or more consecutive registers.
   * @param reg    Starting register address.
   * @param count  Number of bytes to read.
   * @param data   Output buffer (≥ count bytes).
   * @return true on success.
   */
  bool ReadRegisters(uint8_t reg, uint8_t count, uint8_t *data);
};

#endif  // MPU6500_SRC_MPU6500_H_
