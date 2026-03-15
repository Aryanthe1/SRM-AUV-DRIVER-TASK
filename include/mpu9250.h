#ifndef MPU9250_H
#define MPU9250_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* ── I2C Addresses ──────────────────────────────────────────────────────── */
#define MPU9250_I2C_ADDR_LOW   (0x68 << 1)   // AD0 = 0
#define MPU9250_I2C_ADDR_HIGH  (0x69 << 1)   // AD0 = 1
#define AK8963_I2C_ADDR        (0x0C << 1)   // Magnetometer (internal)

/* ── MPU9250 Register Map ───────────────────────────────────────────────── */
#define MPU9250_REG_SELF_TEST_X_GYRO   0x00
#define MPU9250_REG_SELF_TEST_Y_GYRO   0x01
#define MPU9250_REG_SELF_TEST_Z_GYRO   0x02
#define MPU9250_REG_SELF_TEST_X_ACCEL  0x0D
#define MPU9250_REG_SELF_TEST_Y_ACCEL  0x0E
#define MPU9250_REG_SELF_TEST_Z_ACCEL  0x0F
#define MPU9250_REG_SMPLRT_DIV         0x19
#define MPU9250_REG_CONFIG             0x1A
#define MPU9250_REG_GYRO_CONFIG        0x1B
#define MPU9250_REG_ACCEL_CONFIG       0x1C
#define MPU9250_REG_ACCEL_CONFIG2      0x1D
#define MPU9250_REG_INT_PIN_CFG        0x37
#define MPU9250_REG_INT_ENABLE         0x38
#define MPU9250_REG_INT_STATUS         0x3A
#define MPU9250_REG_ACCEL_XOUT_H       0x3B
#define MPU9250_REG_ACCEL_XOUT_L       0x3C
#define MPU9250_REG_ACCEL_YOUT_H       0x3D
#define MPU9250_REG_ACCEL_YOUT_L       0x3E
#define MPU9250_REG_ACCEL_ZOUT_H       0x3F
#define MPU9250_REG_ACCEL_ZOUT_L       0x40
#define MPU9250_REG_TEMP_OUT_H         0x41
#define MPU9250_REG_TEMP_OUT_L         0x42
#define MPU9250_REG_GYRO_XOUT_H        0x43
#define MPU9250_REG_GYRO_XOUT_L        0x44
#define MPU9250_REG_GYRO_YOUT_H        0x45
#define MPU9250_REG_GYRO_YOUT_L        0x46
#define MPU9250_REG_GYRO_ZOUT_H        0x47
#define MPU9250_REG_GYRO_ZOUT_L        0x48
#define MPU9250_REG_USER_CTRL          0x6A
#define MPU9250_REG_PWR_MGMT_1         0x6B
#define MPU9250_REG_PWR_MGMT_2         0x6C
#define MPU9250_REG_WHO_AM_I           0x75

/* ── AK8963 (Magnetometer) Register Map ─────────────────────────────────── */
#define AK8963_REG_WIA      0x00   // Who I Am
#define AK8963_REG_INFO     0x01
#define AK8963_REG_ST1      0x02   // Status 1 (data ready)
#define AK8963_REG_HXL      0x03   // Mag X low byte
#define AK8963_REG_HXH      0x04
#define AK8963_REG_HYL      0x05
#define AK8963_REG_HYH      0x06
#define AK8963_REG_HZL      0x07
#define AK8963_REG_HZH      0x08
#define AK8963_REG_ST2      0x09   // Status 2 (overflow)
#define AK8963_REG_CNTL1    0x0A   // Control 1 (mode)
#define AK8963_REG_CNTL2    0x0B   // Control 2 (soft reset)
#define AK8963_REG_ASTC     0x0C   // Self-test
#define AK8963_REG_ASAX     0x10   // Sensitivity X adjustment
#define AK8963_REG_ASAY     0x11
#define AK8963_REG_ASAZ     0x12

/* ── Expected WHO_AM_I values ────────────────────────────────────────────── */
#define MPU9250_WHO_AM_I_VAL   0x71
#define MPU9255_WHO_AM_I_VAL   0x73   // Some modules use MPU9255 silicon
#define AK8963_WHO_AM_I_VAL    0x48

/* ── Gyroscope Full-Scale Range ─────────────────────────────────────────── */
typedef enum {
    MPU9250_GYRO_FS_250DPS  = 0x00,   // ±250  °/s  — 131.0 LSB/(°/s)
    MPU9250_GYRO_FS_500DPS  = 0x08,   // ±500  °/s  —  65.5 LSB/(°/s)
    MPU9250_GYRO_FS_1000DPS = 0x10,   // ±1000 °/s  —  32.8 LSB/(°/s)
    MPU9250_GYRO_FS_2000DPS = 0x18,   // ±2000 °/s  —  16.4 LSB/(°/s)
} MPU9250_GyroFS;

/* ── Accelerometer Full-Scale Range ─────────────────────────────────────── */
typedef enum {
    MPU9250_ACCEL_FS_2G  = 0x00,   // ±2g  — 16384 LSB/g
    MPU9250_ACCEL_FS_4G  = 0x08,   // ±4g  —  8192 LSB/g
    MPU9250_ACCEL_FS_8G  = 0x10,   // ±8g  —  4096 LSB/g
    MPU9250_ACCEL_FS_16G = 0x18,   // ±16g —  2048 LSB/g
} MPU9250_AccelFS;

/* ── Magnetometer Output Bit Resolution ─────────────────────────────────── */
typedef enum {
    AK8963_MAG_14BIT = 0x00,   // 14-bit output
    AK8963_MAG_16BIT = 0x10,   // 16-bit output (recommended)
} AK8963_MagBits;

/* ── Magnetometer Operating Mode ────────────────────────────────────────── */
typedef enum {
    AK8963_MODE_POWER_DOWN   = 0x00,
    AK8963_MODE_SINGLE       = 0x01,
    AK8963_MODE_CONT_8HZ     = 0x02,   // Continuous measurement @ 8 Hz
    AK8963_MODE_CONT_100HZ   = 0x06,   // Continuous measurement @ 100 Hz
    AK8963_MODE_SELF_TEST    = 0x08,
    AK8963_MODE_FUSE_ROM     = 0x0F,
} AK8963_Mode;

/* ── Status / Error codes ────────────────────────────────────────────────── */
typedef enum {
    MPU9250_OK              =  0,
    MPU9250_ERR_I2C         = -1,   // HAL I2C error
    MPU9250_ERR_WHO_AM_I    = -2,   // Unexpected device ID
    MPU9250_ERR_MAG_WHO_AM_I= -3,   // Magnetometer not found
    MPU9250_ERR_TIMEOUT     = -4,   // HAL timeout
    MPU9250_ERR_MAG_OVF     = -5,   // Magnetometer overflow
    MPU9250_MAG_NOT_READY   = -6,   // Magnetometer DRDY not set yet (not an error — retry)
} MPU9250_Status;

/* ── Raw sensor data ─────────────────────────────────────────────────────── */
typedef struct {
    int16_t x, y, z;
} MPU9250_RawVec3;

/* ── Scaled (SI) sensor data ─────────────────────────────────────────────── */
typedef struct {
    float x, y, z;
} MPU9250_Vec3;

/* ── Magnetometer calibration (factory ASA values) ──────────────────────── */
typedef struct {
    float x, y, z;   // Sensitivity adjustment: val = raw * ((ASA-128)/256 + 1)
} AK8963_Sensitivity;

/* ── Driver handle ───────────────────────────────────────────────────────── */
typedef struct {
    I2C_HandleTypeDef *hi2c;          // Pointer to HAL I2C handle
    uint8_t            addr;          // MPU9250 I2C address (already shifted)
    MPU9250_GyroFS     gyro_fs;       // Current gyro full-scale
    MPU9250_AccelFS    accel_fs;      // Current accel full-scale
    AK8963_MagBits     mag_bits;      // Magnetometer bit depth
    AK8963_Sensitivity mag_sens;      // Factory sensitivity adjustment
    uint32_t           timeout_ms;    // I2C timeout (default: 10 ms)
} MPU9250_Handle;

/* ── Public API ──────────────────────────────────────────────────────────── */

/**
 * @brief  Initialise the MPU9250 and AK8963 magnetometer.
 *
 * Call once after HAL_Init() and I2C MX init. Resets the device, verifies
 * WHO_AM_I, configures full-scale ranges, enables bypass mode so the
 * AK8963 is reachable on the main I2C bus, and reads factory calibration.
 *
 * @param  hdev     Pointer to a zero-initialised MPU9250_Handle.
 * @param  hi2c     Pointer to the HAL I2C handle (e.g. &hi2c1).
 * @param  ad0_high Pass 1 if the AD0 pin is pulled HIGH (addr = 0x69).
 * @param  gyro_fs  Gyroscope full-scale range.
 * @param  accel_fs Accelerometer full-scale range.
 * @return MPU9250_OK on success, negative error code otherwise.
 */
MPU9250_Status MPU9250_Init(MPU9250_Handle    *hdev,
                            I2C_HandleTypeDef *hi2c,
                            uint8_t            ad0_high,
                            MPU9250_GyroFS     gyro_fs,
                            MPU9250_AccelFS    accel_fs);

/**
 * @brief  Read raw accelerometer counts.
 * @param  hdev  Driver handle.
 * @param  out   Output structure (raw 16-bit values).
 */
MPU9250_Status MPU9250_ReadAccelRaw(MPU9250_Handle *hdev, MPU9250_RawVec3 *out);

/**
 * @brief  Read accelerometer in g.
 */
MPU9250_Status MPU9250_ReadAccel(MPU9250_Handle *hdev, MPU9250_Vec3 *out);

/**
 * @brief  Read raw gyroscope counts.
 */
MPU9250_Status MPU9250_ReadGyroRaw(MPU9250_Handle *hdev, MPU9250_RawVec3 *out);

/**
 * @brief  Read gyroscope in degrees per second.
 */
MPU9250_Status MPU9250_ReadGyro(MPU9250_Handle *hdev, MPU9250_Vec3 *out);

/**
 * @brief  Read die temperature in degrees Celsius.
 * @param  temp_c  Output pointer.
 */
MPU9250_Status MPU9250_ReadTemp(MPU9250_Handle *hdev, float *temp_c);

/**
 * @brief  Read raw magnetometer counts (AK8963).
 *         Returns MPU9250_ERR_MAG_OVF if the sensor overflowed.
 */
MPU9250_Status MPU9250_ReadMagRaw(MPU9250_Handle *hdev, MPU9250_RawVec3 *out);

/**
 * @brief  Read magnetometer in microtesla (µT).
 */
MPU9250_Status MPU9250_ReadMag(MPU9250_Handle *hdev, MPU9250_Vec3 *out);

/**
 * @brief  Read all sensors in one shot (burst read).
 * @param  accel   Accelerometer output (g).
 * @param  gyro    Gyroscope output (°/s).
 * @param  temp_c  Temperature output (°C).
 * @param  mag     Magnetometer output (µT).  Pass NULL to skip.
 */
MPU9250_Status MPU9250_ReadAll(MPU9250_Handle *hdev,
                               MPU9250_Vec3   *accel,
                               MPU9250_Vec3   *gyro,
                               float          *temp_c,
                               MPU9250_Vec3   *mag);

/* ── Low-level helpers (exposed for advanced users) ─────────────────────── */
MPU9250_Status MPU9250_WriteReg(MPU9250_Handle *hdev, uint8_t reg, uint8_t val);
MPU9250_Status MPU9250_ReadReg (MPU9250_Handle *hdev, uint8_t reg, uint8_t *val);
MPU9250_Status MPU9250_ReadRegs(MPU9250_Handle *hdev, uint8_t reg,
                                uint8_t *buf, uint16_t len);
MPU9250_Status AK8963_WriteReg (MPU9250_Handle *hdev, uint8_t reg, uint8_t val);
MPU9250_Status AK8963_ReadReg  (MPU9250_Handle *hdev, uint8_t reg, uint8_t *val);
MPU9250_Status AK8963_ReadRegs (MPU9250_Handle *hdev, uint8_t reg,
                                uint8_t *buf, uint16_t len);

#endif /* MPU9250_H */