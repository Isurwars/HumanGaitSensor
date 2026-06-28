/**
 * @file mpu9250.h
 * @brief Driver for the InvenSense MPU-9250 / MPU-9255 / MPU-6500 IMU.
 *
 * Supports both I2C and SPI communication. The MPU-9250 and MPU-9255 variants
 * include an integrated AK8963 magnetometer, accessed via the MPU's internal
 * I2C master. The MPU-6500 is identical but ships without the magnetometer;
 * this driver detects the variant automatically via the WHO_AM_I register and
 * disables magnetometer paths accordingly.
 *
 * Features:
 *  - Configurable accelerometer range (±2 / 4 / 8 / 16 g)
 *  - Configurable gyroscope range (±250 / 500 / 1000 / 2000 °/s)
 *  - Configurable Digital Low-Pass Filter (DLPF) bandwidth
 *  - Configurable sample rate divider (SRD)
 *  - Data-ready interrupt (DRDY)
 *  - Wake-on-Motion (WOM) with configurable threshold and rate
 *  - Optional FIFO buffering for accel and gyro (disable with
 *    -DDISABLE_MPU9250_FIFO build flag)
 *
 * Usage (I2C):
 * @code
 *   Mpu9250 imu(&Wire, Mpu9250::I2cAddr::I2C_ADDR_PRIM);
 *   imu.Begin();
 *   if (imu.Read()) { float ax = imu.accel_x_mps2(); }
 * @endcode
 *
 * @note Output units: acceleration in m/s², angular rate in rad/s,
 *       magnetic field in µT, temperature in °C.
 */

#ifndef MPU9250_SRC_MPU9250_H_
#define MPU9250_SRC_MPU9250_H_

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <cstddef>
#include <cstdint>
#include "linear_algebra.h"

class Mpu9250 {
 public:
  // -------------------------------------------------------------------------
  // Configuration Enumerations
  // -------------------------------------------------------------------------

  /** @brief I2C device address. Set by the AD0 pin level on the sensor. */
  enum class I2cAddr : uint8_t {
    I2C_ADDR_PRIM = 0x68,  ///< AD0 = LOW  (default)
    I2C_ADDR_SEC  = 0x69   ///< AD0 = HIGH
  };

  /**
   * @brief Digital Low-Pass Filter (DLPF) bandwidth for accel and gyro.
   *
   * The DLPF cuts high-frequency noise at the cost of increased group delay.
   * Lower bandwidth = more filtering, higher latency.
   * Register values written to ACCEL_CONFIG2 and CONFIG.
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
   * @brief Accelerometer full-scale measurement range.
   * Register values written to ACCEL_CONFIG (bits [4:3]).
   */
  enum class AccelRange : uint8_t {
    ACCEL_RANGE_2G  = 0x00,  ///< ±2 g  — scale: 16384 LSB/g
    ACCEL_RANGE_4G  = 0x08,  ///< ±4 g  — scale: 8192 LSB/g
    ACCEL_RANGE_8G  = 0x10,  ///< ±8 g  — scale: 4096 LSB/g
    ACCEL_RANGE_16G = 0x18   ///< ±16 g — scale: 2048 LSB/g
  };

  /**
   * @brief Gyroscope full-scale measurement range.
   * Register values written to GYRO_CONFIG (bits [4:3]).
   */
  enum class GyroRange : uint8_t {
    GYRO_RANGE_250DPS  = 0x00,  ///< ±250  °/s — scale: 131.0 LSB/(°/s)
    GYRO_RANGE_500DPS  = 0x08,  ///< ±500  °/s — scale: 65.5 LSB/(°/s)
    GYRO_RANGE_1000DPS = 0x10,  ///< ±1000 °/s — scale: 32.8 LSB/(°/s)
    GYRO_RANGE_2000DPS = 0x18   ///< ±2000 °/s — scale: 16.4 LSB/(°/s)
  };

  /**
   * @brief Wake-on-Motion accelerometer output data rate.
   * Controls how often the WOM comparator samples the accelerometer while
   * the chip is in low-power cycle mode. Written to LP_ACCEL_ODR register.
   */
  enum class WomRate : uint8_t {
    WOM_RATE_0_24HZ  = 0x00,  ///< 0.24 Hz
    WOM_RATE_0_49HZ  = 0x01,  ///< 0.49 Hz
    WOM_RATE_0_98HZ  = 0x02,  ///< 0.98 Hz
    WOM_RATE_1_95HZ  = 0x03,  ///< 1.95 Hz
    WOM_RATE_3_91HZ  = 0x04,  ///< 3.91 Hz
    WOM_RATE_7_81HZ  = 0x05,  ///< 7.81 Hz
    WOM_RATE_15_63HZ = 0x06,  ///< 15.63 Hz
    WOM_RATE_31_25HZ = 0x07,  ///< 31.25 Hz
    WOM_RATE_62_50HZ = 0x08,  ///< 62.50 Hz
    WOM_RATE_125HZ   = 0x09,  ///< 125 Hz
    WOM_RATE_250HZ   = 0x0A,  ///< 250 Hz
    WOM_RATE_500HZ   = 0x0B   ///< 500 Hz
  };

  // -------------------------------------------------------------------------
  // Constructors and Runtime Configuration
  // -------------------------------------------------------------------------

  /** @brief Default constructor. Must call Config() before Begin(). */
  Mpu9250() = default;

  /**
   * @brief Construct and configure for I2C communication.
   * @param i2c  Pointer to the TwoWire (I2C) bus instance.
   * @param addr I2C address of the device (depends on AD0 pin level).
   */
  Mpu9250(TwoWire *i2c, const I2cAddr addr)
      : i2c_(i2c), dev_(static_cast<uint8_t>(addr)), iface_(Interface::I2C) {}

  /**
   * @brief Construct and configure for SPI communication.
   * @param spi Pointer to the SPIClass bus instance.
   * @param cs  Chip-select GPIO pin number.
   */
  Mpu9250(SPIClass *spi, const uint8_t cs)
      : spi_(spi), dev_(cs), iface_(Interface::SPI) {}

  /**
   * @brief Runtime configuration for I2C (alternative to constructor).
   * @param i2c  Pointer to the TwoWire bus instance.
   * @param addr I2C device address.
   */
  void Config(TwoWire *i2c, const I2cAddr addr);

  /**
   * @brief Runtime configuration for SPI (alternative to constructor).
   * @param spi Pointer to the SPIClass bus instance.
   * @param cs  Chip-select GPIO pin number.
   */
  void Config(SPIClass *spi, const uint8_t cs);

  // -------------------------------------------------------------------------
  // Initialization and Sensor Configuration
  // -------------------------------------------------------------------------

  /**
   * @brief Initialize the IMU and configure default settings.
   *
   * Verifies chip identity via WHO_AM_I, performs a hardware reset, and
   * configures the AK8963 magnetometer (if present). Applies defaults:
   *  - Accel range: ±16 g
   *  - Gyro range:  ±2000 °/s
   *  - DLPF:        184 Hz
   *  - SRD:         0 (max sample rate)
   *
   * @return true on success, false if communication fails or chip ID is wrong.
   */
  [[nodiscard]] bool Begin();

  /**
   * @brief Enable the Data-Ready interrupt on the INT pin.
   *
   * Configures the INT pin to pulse for 50 µs on each new sample. The pin
   * can be connected to an MCU GPIO to trigger a data-read ISR.
   *
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
   * @param range  Desired range from the AccelRange enum.
   * @return true on success.
   */
  [[nodiscard]] bool ConfigAccelRange(const AccelRange range);

  /** @brief Return the currently active accelerometer range. */
  inline AccelRange accel_range() const { return accel_range_; }

  /**
   * @brief Set the gyroscope full-scale range.
   * @param range  Desired range from the GyroRange enum.
   * @return true on success.
   */
  [[nodiscard]] bool ConfigGyroRange(const GyroRange range);

  /** @brief Return the currently active gyroscope range. */
  inline GyroRange gyro_range() const { return gyro_range_; }

  /**
   * @brief Set the sample rate divider (SRD).
   *
   * The IMU output data rate (ODR) is derived from the gyro output rate (1 kHz
   * when DLPF is enabled) divided by (1 + SRD):
   *   ODR = 1000 Hz / (1 + SRD)
   *
   * Also configures the AK8963 magnetometer update rate:
   *  - SRD > 9  → 8 Hz mag rate
   *  - SRD ≤ 9  → 100 Hz mag rate
   *
   * @param srd  Sample rate divider value (0 = 1 kHz, 9 = 100 Hz, 99 = 10 Hz).
   * @return true on success.
   */
  [[nodiscard]] bool ConfigSrd(const uint8_t srd);

  /** @brief Return the current sample rate divider value. */
  inline uint8_t srd() const { return srd_; }

  /**
   * @brief Set the Digital Low-Pass Filter (DLPF) bandwidth.
   *
   * The same filter setting is applied to both the accelerometer
   * (ACCEL_CONFIG2) and the gyroscope (CONFIG) simultaneously.
   *
   * @param dlpf  Desired DLPF bandwidth from the DlpfBandwidth enum.
   * @return true on success.
   */
  [[nodiscard]] bool ConfigDlpfBandwidth(const DlpfBandwidth dlpf);

  /** @brief Return the currently active DLPF bandwidth setting. */
  inline DlpfBandwidth dlpf_bandwidth() const { return dlpf_bandwidth_; }

  /**
   * @brief Enter Wake-on-Motion mode.
   *
   * Puts the gyroscope and magnetometer to sleep. Only the accelerometer
   * runs periodically at the specified WomRate. When acceleration exceeds
   * the threshold, the WOM interrupt fires on the INT pin. The device must
   * be re-initialized with Begin() to exit WOM mode.
   *
   * @param threshold_mg  Motion detection threshold in milli-g (4–1020 mg).
   *                      LSB resolution is 4 mg.
   * @param wom_rate      Accelerometer sample rate during WOM.
   * @return true on success, false if threshold is out of range.
   */
  [[nodiscard]] bool EnableWom(int16_t threshold_mg, const WomRate wom_rate);

  // -------------------------------------------------------------------------
  // FIFO Control (optional, disable with -DDISABLE_MPU9250_FIFO)
  // -------------------------------------------------------------------------

  #if !defined(DISABLE_MPU9250_FIFO)
  /**
   * @brief Enable FIFO buffering for accelerometer and gyroscope data.
   *
   * Once enabled, new samples are pushed into the on-chip 512-byte FIFO at
   * the configured ODR. Use ReadFifo() to retrieve them in bulk.
   *
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
   * @brief Perform a hardware reset of the MPU and AK8963.
   *
   * Brings the sensor back to its power-on-reset state. A subsequent call
   * to Begin() is required before reading data.
   */
  void Reset();

  /**
   * @brief Read the latest sensor data from the MPU-9250.
   *
   * Reads the interrupt status register and all sensor data registers in a
   * single burst. Returns false if no new data is available (poll mode) or
   * if the I2C/SPI transaction fails. After a successful read, use the
   * accessor methods (accel_x_mps2(), etc.) to retrieve values.
   *
   * @return true if new IMU data was available and successfully parsed.
   */
  [[nodiscard]] bool Read();

  #if !defined(DISABLE_MPU9250_FIFO)
  /**
   * @brief Read all frames currently stored in the FIFO.
   *
   * Queries the FIFO byte count, reads up to FIFO_MAX_SIZE() frames, and
   * stores them in the internal FIFO arrays. Use fifo_accel_x_mps2() etc.
   * to copy the data out.
   *
   * @return Number of frames read (≥ 0), or -1 on communication error.
   */
  [[nodiscard]] int8_t ReadFifo();
  #endif

  // -------------------------------------------------------------------------
  // IMU Data Accessors
  // -------------------------------------------------------------------------

  /** @brief True if the last Read() call contained new IMU data. */
  inline bool new_imu_data() const { return new_imu_data_; }

  /** @brief Accelerometer X-axis reading in m/s². */
  inline float accel_x_mps2() const { return accel_.x; }
  /** @brief Accelerometer Y-axis reading in m/s². */
  inline float accel_y_mps2() const { return accel_.y; }
  /** @brief Accelerometer Z-axis reading in m/s². */
  inline float accel_z_mps2() const { return accel_.z; }
  /** @brief All three accelerometer axes as a Vector3f [m/s²]. */
  inline Vector3f accel_mps2() const { return accel_; }

  /** @brief Gyroscope X-axis reading in rad/s. */
  inline float gyro_x_radps() const { return gyro_.x; }
  /** @brief Gyroscope Y-axis reading in rad/s. */
  inline float gyro_y_radps() const { return gyro_.y; }
  /** @brief Gyroscope Z-axis reading in rad/s. */
  inline float gyro_z_radps() const { return gyro_.z; }
  /** @brief All three gyroscope axes as a Vector3f [rad/s]. */
  inline Vector3f gyro_radps() const { return gyro_; }

  // -------------------------------------------------------------------------
  // Magnetometer Data Accessors (AK8963, MPU-9250/9255 only)
  // -------------------------------------------------------------------------

  /** @brief True if the last Read() contained fresh magnetometer data. */
  inline bool new_mag_data() const { return new_mag_data_; }

  /** @brief Magnetometer X-axis reading in µT. */
  inline float mag_x_ut() const { return mag_.x; }
  /** @brief Magnetometer Y-axis reading in µT. */
  inline float mag_y_ut() const { return mag_.y; }
  /** @brief Magnetometer Z-axis reading in µT. */
  inline float mag_z_ut() const { return mag_.z; }
  /** @brief All three magnetometer axes as a Vector3f [µT]. */
  inline Vector3f mag_ut() const { return mag_; }

  // -------------------------------------------------------------------------
  // Temperature Data Accessor
  // -------------------------------------------------------------------------

  /** @brief Die temperature in degrees Celsius. */
  inline float die_temp_c() const { return temp_; }

  // -------------------------------------------------------------------------
  // FIFO Data Accessors (optional)
  // -------------------------------------------------------------------------

  #if !defined(DISABLE_MPU9250_FIFO)
  /**
   * @brief Copy FIFO accel X samples into a user-provided buffer.
   * @param data  Destination buffer. Must not be null.
   * @param len   Capacity of the buffer (number of floats).
   * @return Number of samples copied, or -1 if data is null.
   */
  int8_t fifo_accel_x_mps2(float *data, const std::size_t len);

  /** @brief Copy FIFO accel Y samples. @see fifo_accel_x_mps2 */
  int8_t fifo_accel_y_mps2(float *data, const std::size_t len);

  /** @brief Copy FIFO accel Z samples. @see fifo_accel_x_mps2 */
  int8_t fifo_accel_z_mps2(float *data, const std::size_t len);

  /** @brief Copy FIFO gyro X samples (rad/s). @see fifo_accel_x_mps2 */
  int8_t fifo_gyro_x_radps(float *data, const std::size_t len);

  /** @brief Copy FIFO gyro Y samples (rad/s). @see fifo_accel_x_mps2 */
  int8_t fifo_gyro_y_radps(float *data, const std::size_t len);

  /** @brief Copy FIFO gyro Z samples (rad/s). @see fifo_accel_x_mps2 */
  int8_t fifo_gyro_z_radps(float *data, const std::size_t len);

  /** @brief Maximum number of accel+gyro frames the internal FIFO cache holds. */
  static constexpr int8_t FIFO_MAX_SIZE() { return FIFO_MAX_NUM_FRAMES_; }

  /** @brief True if a FIFO overflow interrupt was detected in the last Read(). */
  inline bool fifo_overflow() const { return fifo_overflow_; }
  #endif

 private:
  // -------------------------------------------------------------------------
  // Communication Interface
  // -------------------------------------------------------------------------

  /** @brief Selects the active communication bus (I2C or SPI). */
  enum class Interface : int8_t { SPI, I2C };

  TwoWire  *i2c_  = nullptr;   ///< I2C bus handle (used when iface_ == I2C)
  SPIClass *spi_  = nullptr;   ///< SPI bus handle (used when iface_ == SPI)
  uint8_t   dev_  = 0;         ///< I2C address or SPI chip-select pin number
  Interface iface_ = Interface::I2C;  ///< Active interface type
  uint8_t   bytes_rx_ = 0;     ///< Byte count returned by I2C requestFrom()
  int32_t   spi_clock_ = SPI_CFG_CLOCK_;  ///< Current SPI clock speed (Hz)

  /*
   * The MPU-9250 supports an SPI clock of 1 MHz for register writes and
   * 20 MHz for burst data reads. In practice 20 MHz can be unreliable on
   * some hardware, so reads are capped at 15 MHz.
   */
  static constexpr int32_t SPI_CFG_CLOCK_  = 1000000;   ///< 1 MHz — config writes
  static constexpr int32_t SPI_READ_CLOCK_ = 15000000;  ///< 15 MHz — data reads
  static constexpr uint8_t SPI_READ_       = 0x80;      ///< SPI read-flag OR'd onto register address

  // -------------------------------------------------------------------------
  // Runtime Configuration State
  // -------------------------------------------------------------------------

  AccelRange    accel_range_          = AccelRange::ACCEL_RANGE_16G;
  AccelRange    requested_accel_range_ = AccelRange::ACCEL_RANGE_16G;
  GyroRange     gyro_range_           = GyroRange::GYRO_RANGE_2000DPS;
  GyroRange     requested_gyro_range_  = GyroRange::GYRO_RANGE_2000DPS;
  DlpfBandwidth dlpf_bandwidth_       = DlpfBandwidth::DLPF_BANDWIDTH_184HZ;
  DlpfBandwidth requested_dlpf_       = DlpfBandwidth::DLPF_BANDWIDTH_184HZ;

  float   accel_scale_          = 0.0f;  ///< g-per-LSB for the active accel range
  float   requested_accel_scale_ = 0.0f;
  float   gyro_scale_           = 0.0f;  ///< deg/s per LSB for the active gyro range
  float   requested_gyro_scale_  = 0.0f;
  uint8_t srd_                  = 0;     ///< Sample rate divider (0 = 1 kHz)

  // -------------------------------------------------------------------------
  // Chip Identity and Magnetometer
  // -------------------------------------------------------------------------

  uint8_t who_am_i_ = 0;  ///< Value read from WHO_AM_I register during Begin()

  static constexpr uint8_t WHOAMI_MPU9250_ = 0x71;  ///< WHO_AM_I value for MPU-9250
  static constexpr uint8_t WHOAMI_MPU9255_ = 0x73;  ///< WHO_AM_I value for MPU-9255
  static constexpr uint8_t WHOAMI_MPU6500_ = 0x70;  ///< WHO_AM_I value for MPU-6500 (no mag)
  static constexpr uint8_t WHOAMI_AK8963_  = 0x48;  ///< WHO_AM_I value for AK8963 magnetometer

  bool    has_magnetometer_ = true;  ///< False when an MPU-6500 is detected
  uint8_t asa_buff_[3]      = {0};   ///< Raw AK8963 ASA calibration bytes (X, Y, Z)
  float   mag_scale_[3]     = {0};   ///< Sensitivity-adjusted scale factors for mag axes [µT/LSB]

  // -------------------------------------------------------------------------
  // Sensor Data Buffers
  // -------------------------------------------------------------------------

  bool    new_imu_data_        = false;  ///< Set true when Read() gets a fresh IMU sample
  bool    new_mag_data_        = false;  ///< Set true when Read() gets fresh magnetometer data
  bool    mag_sensor_overflow_ = false;  ///< True when AK8963 HOFL bit is set (field too strong)

  uint8_t  mag_data_[8]    = {0};   ///< Raw AK8963 read buffer (ST1 + 6 data bytes + ST2)
  uint8_t  data_buf_[23]   = {0};   ///< Burst read buffer: INT_STATUS + accel + temp + gyro + mag
  int16_t  accel_cnts_[3]  = {0};   ///< Raw 16-bit accel counts [X, Y, Z]
  int16_t  gyro_cnts_[3]   = {0};   ///< Raw 16-bit gyro counts [X, Y, Z]
  int16_t  temp_cnts_       = 0;    ///< Raw 16-bit temperature counts
  int16_t  mag_cnts_[3]    = {0};   ///< Raw 16-bit magnetometer counts [X, Y, Z]

  Vector3f accel_;   ///< Converted accel in m/s²
  Vector3f gyro_;    ///< Converted gyro in rad/s
  Vector3f mag_;     ///< Converted magnetometer reading in µT

  float temp_ = 0.0f;  ///< Converted die temperature in °C

  /// Divisor for the raw temperature formula: (counts - 21) / TEMP_SCALE_ + 21
  static constexpr float TEMP_SCALE_ = 333.87f;

  // -------------------------------------------------------------------------
  // FIFO State (conditional compile)
  // -------------------------------------------------------------------------

  #if !defined(DISABLE_MPU9250_FIFO)
  bool    fifo_overflow_    = false;  ///< Set when FIFO overflow interrupt bit is detected
  int8_t  fifo_num_frames_  = 0;     ///< Number of complete frames in the last ReadFifo() call
  int16_t fifo_bytes_       = 0;     ///< Raw FIFO byte count from FIFO_COUNT registers

  /// Bytes per FIFO frame: 6 accel + 6 gyro = 12 bytes (no temp or mag in FIFO)
  static constexpr int8_t FIFO_FRAME_SIZE_     = 12;
  /// Maximum frames this driver buffers internally (limited by 512-byte FIFO / 12)
  static constexpr int8_t FIFO_MAX_NUM_FRAMES_ = 42;

  float fifo_ax_[FIFO_MAX_NUM_FRAMES_] = {0};  ///< FIFO accel X history [m/s²]
  float fifo_ay_[FIFO_MAX_NUM_FRAMES_] = {0};  ///< FIFO accel Y history [m/s²]
  float fifo_az_[FIFO_MAX_NUM_FRAMES_] = {0};  ///< FIFO accel Z history [m/s²]
  float fifo_gx_[FIFO_MAX_NUM_FRAMES_] = {0};  ///< FIFO gyro X history [rad/s]
  float fifo_gy_[FIFO_MAX_NUM_FRAMES_] = {0};  ///< FIFO gyro Y history [rad/s]
  float fifo_gz_[FIFO_MAX_NUM_FRAMES_] = {0};  ///< FIFO gyro Z history [rad/s]
  #endif

  // -------------------------------------------------------------------------
  // MPU-9250 Register Map
  // -------------------------------------------------------------------------

  // --- Power Management ---
  static constexpr uint8_t PWR_MGMNT_1_ = 0x6B;  ///< Power management 1
  static constexpr uint8_t PWR_MGMNT_2_ = 0x6C;  ///< Power management 2
  static constexpr uint8_t H_RESET_     = 0x80;  ///< Device reset bit in PWR_MGMNT_1
  static constexpr uint8_t CLKSEL_PLL_  = 0x01;  ///< Clock source: auto-selects PLL if ready

  // --- Identity ---
  static constexpr uint8_t WHOAMI_ = 0x75;  ///< WHO_AM_I register address

  // --- Sensor Configuration ---
  static constexpr uint8_t ACCEL_CONFIG_  = 0x1C;  ///< Accel full-scale range
  static constexpr uint8_t ACCEL_CONFIG2_ = 0x1D;  ///< Accel DLPF and averaging
  static constexpr uint8_t GYRO_CONFIG_   = 0x1B;  ///< Gyro full-scale range
  static constexpr uint8_t CONFIG_        = 0x1A;  ///< Gyro DLPF
  static constexpr uint8_t SMPLRT_DIV_   = 0x19;  ///< Sample rate divider

  // --- Interrupt Control ---
  static constexpr uint8_t INT_PIN_CFG_    = 0x37;  ///< INT pin / bypass config
  static constexpr uint8_t INT_ENABLE_     = 0x38;  ///< Interrupt enable
  static constexpr uint8_t INT_DISABLE_    = 0x00;  ///< Value to clear all interrupt enables
  static constexpr uint8_t INT_PULSE_50US_ = 0x00;  ///< INT pin: 50 µs pulse, active high
  static constexpr uint8_t INT_RAW_RDY_EN_ = 0x01;  ///< Enable raw data ready interrupt
  static constexpr uint8_t INT_STATUS_     = 0x3A;  ///< Interrupt status (read to clear)
  static constexpr uint8_t RAW_DATA_RDY_INT_ = 0x01;  ///< Bit mask for data-ready in INT_STATUS

  // --- User Control ---
  static constexpr uint8_t USER_CTRL_   = 0x6A;  ///< User control (FIFO/I2C master enable)
  static constexpr uint8_t I2C_MST_EN_  = 0x20;  ///< Enable internal I2C master (for AK8963)

  // --- Internal I2C Master (for AK8963 access) ---
  static constexpr uint8_t I2C_MST_CLK_    = 0x0D;  ///< I2C master clock: 400 kHz
  static constexpr uint8_t I2C_MST_CTRL_   = 0x24;  ///< I2C master control register
  static constexpr uint8_t I2C_SLV0_ADDR_  = 0x25;  ///< Slave 0 I2C address
  static constexpr uint8_t I2C_SLV0_REG_   = 0x26;  ///< Slave 0 register to access
  static constexpr uint8_t I2C_SLV0_CTRL_  = 0x27;  ///< Slave 0 transfer length + enable
  static constexpr uint8_t I2C_SLV0_DO_    = 0x63;  ///< Slave 0 data-out (write payload)
  static constexpr uint8_t I2C_READ_FLAG_  = 0x80;  ///< OR onto slave address for a read transaction
  static constexpr uint8_t I2C_SLV0_EN_    = 0x80;  ///< Enable slave 0 transfer in CTRL register
  static constexpr uint8_t EXT_SENS_DATA_00_ = 0x49;  ///< First external sensor data register (read result)

  // --- Wake-on-Motion ---
  static constexpr uint8_t INT_WOM_EN_      = 0x40;  ///< Enable WOM interrupt
  static constexpr uint8_t DISABLE_GYRO_    = 0x07;  ///< PWR_MGMNT_2: power down gyro axes
  static constexpr uint8_t MOT_DETECT_CTRL_ = 0x69;  ///< Motion detect control
  static constexpr uint8_t ACCEL_INTEL_EN_  = 0x80;  ///< Enable accel hardware intelligence
  static constexpr uint8_t ACCEL_INTEL_MODE_= 0x40;  ///< Compare current vs previous sample
  static constexpr uint8_t LP_ACCEL_ODR_    = 0x1E;  ///< Low-power accel output data rate
  static constexpr uint8_t WOM_THR_         = 0x1F;  ///< WOM threshold (4 mg per LSB)
  static constexpr uint8_t PWR_CYCLE_WOM_   = 0x20;  ///< PWR_MGMNT_1: cycle mode for WOM

  // --- FIFO Registers (conditional compile) ---
  #if !defined(DISABLE_MPU9250_FIFO)
  static constexpr uint8_t FIFO_EN_CTRL_      = 0x40;  ///< USER_CTRL bit to enable FIFO
  static constexpr uint8_t FIFO_EN_           = 0x23;  ///< FIFO enable (selects which sensors write)
  static constexpr uint8_t FIFO_GYRO_         = 0x70;  ///< FIFO_EN: enable gyro X/Y/Z
  static constexpr uint8_t FIFO_ACCEL_        = 0x08;  ///< FIFO_EN: enable accel
  static constexpr uint8_t FIFO_COUNT_        = 0x72;  ///< FIFO byte count (high byte at 0x72)
  static constexpr uint8_t FIFO_READ_         = 0x74;  ///< FIFO data register (burst read)
  static constexpr uint8_t FIFO_OVERFLOW_INT_ = 0x10;  ///< INT_STATUS bit for FIFO overflow
  static constexpr uint8_t FIFO_RESET_        = 0x04;  ///< USER_CTRL bit to reset FIFO
  #endif

  // -------------------------------------------------------------------------
  // AK8963 Magnetometer Register Map
  // -------------------------------------------------------------------------

  static constexpr uint8_t AK8963_I2C_ADDR_    = 0x0C;  ///< AK8963 I2C address on internal bus
  static constexpr uint8_t AK8963_WHOAMI_      = 0x00;  ///< AK8963 WHO_AM_I register
  static constexpr uint8_t AK8963_ST1_         = 0x02;  ///< Status 1 (data ready bit)
  static constexpr uint8_t AK8963_DATA_RDY_INT_ = 0x01;  ///< ST1 bit: data ready
  static constexpr uint8_t AK8963_HXL_         = 0x03;  ///< X-axis low byte (start of measurement burst)
  static constexpr uint8_t AK8963_CNTL1_       = 0x0A;  ///< Control 1 (mode + resolution)
  static constexpr uint8_t AK8963_CNTL2_       = 0x0B;  ///< Control 2 (soft reset)
  static constexpr uint8_t AK8963_PWR_DOWN_    = 0x00;  ///< CNTL1: power-down mode
  static constexpr uint8_t AK8963_CNT_MEAS1_   = 0x12;  ///< CNTL1: 16-bit, continuous mode 1 (8 Hz)
  static constexpr uint8_t AK8963_CNT_MEAS2_   = 0x16;  ///< CNTL1: 16-bit, continuous mode 2 (100 Hz)
  static constexpr uint8_t AK8963_FUSE_ROM_    = 0x0F;  ///< CNTL1: Fuse ROM access mode (for ASA cal)
  static constexpr uint8_t AK8963_RESET_       = 0x01;  ///< CNTL2: soft reset
  static constexpr uint8_t AK8963_ASA_         = 0x10;  ///< Start address of ASA calibration registers
  static constexpr uint8_t AK8963_HOFL_        = 0x08;  ///< ST2 bit: magnetic sensor overflow

  // -------------------------------------------------------------------------
  // Private Communication Methods
  // -------------------------------------------------------------------------

  /**
   * @brief Write a single byte to an MPU-9250 register and verify readback.
   * @param reg   Register address.
   * @param data  Byte to write.
   * @return true if the write succeeded and readback matched.
   */
  bool WriteRegister(uint8_t reg, uint8_t data);

  /**
   * @brief Read one or more consecutive MPU-9250 registers.
   * @param reg    Starting register address.
   * @param count  Number of bytes to read.
   * @param data   Output buffer (must be at least `count` bytes).
   * @return true on success.
   */
  bool ReadRegisters(uint8_t reg, uint8_t count, uint8_t *data);

  /**
   * @brief Write a byte to an AK8963 register via the MPU's I2C master.
   *
   * The MPU acts as an I2C master on its auxiliary bus. This function
   * configures the slave 0 channel to write one byte to the AK8963,
   * then reads back the register via ReadAk8963Registers() to verify.
   *
   * @param reg   AK8963 register address.
   * @param data  Byte to write.
   * @return true if the write succeeded and readback matched.
   */
  bool WriteAk8963Register(uint8_t reg, uint8_t data);

  /**
   * @brief Read one or more registers from the AK8963 via the MPU's I2C master.
   *
   * Configures slave 0 to read `count` bytes starting at `reg`, triggers the
   * transfer, waits 1 ms, then reads the result from EXT_SENS_DATA_00.
   *
   * @param reg    AK8963 register address.
   * @param count  Number of bytes to read.
   * @param data   Output buffer (must be at least `count` bytes).
   * @return true on success.
   */
  bool ReadAk8963Registers(uint8_t reg, uint8_t count, uint8_t *data);
};

#endif  // MPU9250_SRC_MPU9250_H_
