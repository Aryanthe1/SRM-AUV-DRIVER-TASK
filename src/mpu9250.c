#include "mpu9250.h"

/* ══════════════════════════════════════════════════════════════════════════
 *  Internal helpers
 * ══════════════════════════════════════════════════════════════════════════ */

/* Sensitivity scale factors (LSB → physical unit) */
static float accel_scale(MPU9250_AccelFS fs)
{
    switch (fs) {
        case MPU9250_ACCEL_FS_2G:  return 1.0f / 16384.0f;
        case MPU9250_ACCEL_FS_4G:  return 1.0f / 8192.0f;
        case MPU9250_ACCEL_FS_8G:  return 1.0f / 4096.0f;
        case MPU9250_ACCEL_FS_16G: return 1.0f / 2048.0f;
        default:                   return 1.0f / 16384.0f;
    }
}

static float gyro_scale(MPU9250_GyroFS fs)
{
    switch (fs) {
        case MPU9250_GYRO_FS_250DPS:  return 1.0f / 131.0f;
        case MPU9250_GYRO_FS_500DPS:  return 1.0f / 65.5f;
        case MPU9250_GYRO_FS_1000DPS: return 1.0f / 32.8f;
        case MPU9250_GYRO_FS_2000DPS: return 1.0f / 16.4f;
        default:                      return 1.0f / 131.0f;
    }
}

/* Magnetometer LSB → µT: 16-bit = 0.15 µT/LSB, 14-bit = 0.6 µT/LSB */
static float mag_scale(AK8963_MagBits bits)
{
    return (bits == AK8963_MAG_16BIT) ? 0.15f : 0.6f;
}

/* Map HAL status to driver status */
static MPU9250_Status hal_to_drv(HAL_StatusTypeDef s)
{
    switch (s) {
        case HAL_OK:      return MPU9250_OK;
        case HAL_TIMEOUT: return MPU9250_ERR_TIMEOUT;
        default:          return MPU9250_ERR_I2C;
    }
}

/* ══════════════════════════════════════════════════════════════════════════
 *  Low-level register I/O — MPU9250
 * ══════════════════════════════════════════════════════════════════════════ */

MPU9250_Status MPU9250_WriteReg(MPU9250_Handle *hdev, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return hal_to_drv(
        HAL_I2C_Master_Transmit(hdev->hi2c, hdev->addr,
                                buf, 2, hdev->timeout_ms));
}

MPU9250_Status MPU9250_ReadReg(MPU9250_Handle *hdev, uint8_t reg, uint8_t *val)
{
    MPU9250_Status st;
    st = hal_to_drv(HAL_I2C_Master_Transmit(hdev->hi2c, hdev->addr,
                                             &reg, 1, hdev->timeout_ms));
    if (st != MPU9250_OK) return st;
    return hal_to_drv(HAL_I2C_Master_Receive(hdev->hi2c, hdev->addr,
                                             val, 1, hdev->timeout_ms));
}

MPU9250_Status MPU9250_ReadRegs(MPU9250_Handle *hdev, uint8_t reg,
                                uint8_t *buf, uint16_t len)
{
    MPU9250_Status st;
    st = hal_to_drv(HAL_I2C_Master_Transmit(hdev->hi2c, hdev->addr,
                                             &reg, 1, hdev->timeout_ms));
    if (st != MPU9250_OK) return st;
    return hal_to_drv(HAL_I2C_Master_Receive(hdev->hi2c, hdev->addr,
                                             buf, len, hdev->timeout_ms));
}

/* ══════════════════════════════════════════════════════════════════════════
 *  Low-level register I/O — AK8963 magnetometer
 * ══════════════════════════════════════════════════════════════════════════ */

MPU9250_Status AK8963_WriteReg(MPU9250_Handle *hdev, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return hal_to_drv(
        HAL_I2C_Master_Transmit(hdev->hi2c, AK8963_I2C_ADDR,
                                buf, 2, hdev->timeout_ms));
}

MPU9250_Status AK8963_ReadReg(MPU9250_Handle *hdev, uint8_t reg, uint8_t *val)
{
    MPU9250_Status st;
    st = hal_to_drv(HAL_I2C_Master_Transmit(hdev->hi2c, AK8963_I2C_ADDR,
                                             &reg, 1, hdev->timeout_ms));
    if (st != MPU9250_OK) return st;
    return hal_to_drv(HAL_I2C_Master_Receive(hdev->hi2c, AK8963_I2C_ADDR,
                                             val, 1, hdev->timeout_ms));
}

MPU9250_Status AK8963_ReadRegs(MPU9250_Handle *hdev, uint8_t reg,
                               uint8_t *buf, uint16_t len)
{
    MPU9250_Status st;
    st = hal_to_drv(HAL_I2C_Master_Transmit(hdev->hi2c, AK8963_I2C_ADDR,
                                             &reg, 1, hdev->timeout_ms));
    if (st != MPU9250_OK) return st;
    return hal_to_drv(HAL_I2C_Master_Receive(hdev->hi2c, AK8963_I2C_ADDR,
                                             buf, len, hdev->timeout_ms));
}

/* ══════════════════════════════════════════════════════════════════════════
 *  Initialisation
 * ══════════════════════════════════════════════════════════════════════════ */

MPU9250_Status MPU9250_Init(MPU9250_Handle    *hdev,
                            I2C_HandleTypeDef *hi2c,
                            uint8_t            ad0_high,
                            MPU9250_GyroFS     gyro_fs,
                            MPU9250_AccelFS    accel_fs)
{
    MPU9250_Status st;
    uint8_t who;

    /* Populate handle defaults */
    hdev->hi2c       = hi2c;
    hdev->addr       = ad0_high ? MPU9250_I2C_ADDR_HIGH : MPU9250_I2C_ADDR_LOW;
    hdev->gyro_fs    = gyro_fs;
    hdev->accel_fs   = accel_fs;
    hdev->mag_bits   = AK8963_MAG_16BIT;
    hdev->timeout_ms = 10;

    /* ── 1. Reset the device ─────────────────────────────────────────────── */
    st = MPU9250_WriteReg(hdev, MPU9250_REG_PWR_MGMT_1, 0x80); // H_RESET
    if (st != MPU9250_OK) return st;
    HAL_Delay(100);

    /* ── 2. Wake up, select best available clock (PLL with gyro ref) ──────── */
    st = MPU9250_WriteReg(hdev, MPU9250_REG_PWR_MGMT_1, 0x01);
    if (st != MPU9250_OK) return st;
    HAL_Delay(10);

    /* ── 3. Verify WHO_AM_I ──────────────────────────────────────────────── */
    st = MPU9250_ReadReg(hdev, MPU9250_REG_WHO_AM_I, &who);
    if (st != MPU9250_OK) return st;
    if (who != MPU9250_WHO_AM_I_VAL && who != MPU9255_WHO_AM_I_VAL)
        return MPU9250_ERR_WHO_AM_I;

    /* ── 4. Configure sample-rate divider (1 kHz / (1+0) = 1 kHz) ─────────── */
    st = MPU9250_WriteReg(hdev, MPU9250_REG_SMPLRT_DIV, 0x00);
    if (st != MPU9250_OK) return st;

    /* ── 5. DLPF config: Gyro BW=92 Hz, Temp BW=98 Hz, Fs=1 kHz ─────────── */
    st = MPU9250_WriteReg(hdev, MPU9250_REG_CONFIG, 0x02);
    if (st != MPU9250_OK) return st;

    /* ── 6. Gyro full-scale range ────────────────────────────────────────── */
    st = MPU9250_WriteReg(hdev, MPU9250_REG_GYRO_CONFIG, (uint8_t)gyro_fs);
    if (st != MPU9250_OK) return st;

    /* ── 7. Accel full-scale range ───────────────────────────────────────── */
    st = MPU9250_WriteReg(hdev, MPU9250_REG_ACCEL_CONFIG, (uint8_t)accel_fs);
    if (st != MPU9250_OK) return st;

    /* ── 8. Accel DLPF: BW=99 Hz ─────────────────────────────────────────── */
    st = MPU9250_WriteReg(hdev, MPU9250_REG_ACCEL_CONFIG2, 0x02);
    if (st != MPU9250_OK) return st;

    /* ── 9. Enable I2C bypass so AK8963 is visible on the main bus ─────────── */
    // INT_PIN_CFG: BYPASS_EN = 1, ACTL = 0 (active high)
    st = MPU9250_WriteReg(hdev, MPU9250_REG_INT_PIN_CFG, 0x02);
    if (st != MPU9250_OK) return st;
    HAL_Delay(10);

    /* ── 10. Initialise AK8963 ────────────────────────────────────────────── */
    /* 10a. Verify magnetometer WHO_AM_I */
    st = AK8963_ReadReg(hdev, AK8963_REG_WIA, &who);
    if (st != MPU9250_OK) return st;
    if (who != AK8963_WHO_AM_I_VAL) return MPU9250_ERR_MAG_WHO_AM_I;

    /* 10b. Power down before changing mode */
    st = AK8963_WriteReg(hdev, AK8963_REG_CNTL1, AK8963_MODE_POWER_DOWN);
    if (st != MPU9250_OK) return st;
    HAL_Delay(10);

    /* 10c. Enter Fuse ROM mode to read factory sensitivity adjustment */
    st = AK8963_WriteReg(hdev, AK8963_REG_CNTL1, AK8963_MODE_FUSE_ROM);
    if (st != MPU9250_OK) return st;
    HAL_Delay(10);

    uint8_t asa[3];
    st = AK8963_ReadRegs(hdev, AK8963_REG_ASAX, asa, 3);
    if (st != MPU9250_OK) return st;

    hdev->mag_sens.x = ((float)((int16_t)asa[0] - 128) / 256.0f) + 1.0f;
    hdev->mag_sens.y = ((float)((int16_t)asa[1] - 128) / 256.0f) + 1.0f;
    hdev->mag_sens.z = ((float)((int16_t)asa[2] - 128) / 256.0f) + 1.0f;

    /* 10d. Power down again */
    st = AK8963_WriteReg(hdev, AK8963_REG_CNTL1, AK8963_MODE_POWER_DOWN);
    if (st != MPU9250_OK) return st;
    HAL_Delay(10);

    /* 10e. Set continuous measurement @ 100 Hz, 16-bit output */
    uint8_t cntl1 = (uint8_t)hdev->mag_bits | (uint8_t)AK8963_MODE_CONT_100HZ;
    st = AK8963_WriteReg(hdev, AK8963_REG_CNTL1, cntl1);
    if (st != MPU9250_OK) return st;
    HAL_Delay(10);

    return MPU9250_OK;
}

/* ══════════════════════════════════════════════════════════════════════════
 *  Accelerometer
 * ══════════════════════════════════════════════════════════════════════════ */

MPU9250_Status MPU9250_ReadAccelRaw(MPU9250_Handle *hdev, MPU9250_RawVec3 *out)
{
    uint8_t buf[6];
    MPU9250_Status st = MPU9250_ReadRegs(hdev, MPU9250_REG_ACCEL_XOUT_H, buf, 6);
    if (st != MPU9250_OK) return st;

    out->x = (int16_t)((buf[0] << 8) | buf[1]);
    out->y = (int16_t)((buf[2] << 8) | buf[3]);
    out->z = (int16_t)((buf[4] << 8) | buf[5]);
    return MPU9250_OK;
}

MPU9250_Status MPU9250_ReadAccel(MPU9250_Handle *hdev, MPU9250_Vec3 *out)
{
    MPU9250_RawVec3 raw;
    MPU9250_Status st = MPU9250_ReadAccelRaw(hdev, &raw);
    if (st != MPU9250_OK) return st;

    float scale = accel_scale(hdev->accel_fs);
    out->x = raw.x * scale;
    out->y = raw.y * scale;
    out->z = raw.z * scale;
    return MPU9250_OK;
}

/* ══════════════════════════════════════════════════════════════════════════
 *  Gyroscope
 * ══════════════════════════════════════════════════════════════════════════ */

MPU9250_Status MPU9250_ReadGyroRaw(MPU9250_Handle *hdev, MPU9250_RawVec3 *out)
{
    uint8_t buf[6];
    MPU9250_Status st = MPU9250_ReadRegs(hdev, MPU9250_REG_GYRO_XOUT_H, buf, 6);
    if (st != MPU9250_OK) return st;

    out->x = (int16_t)((buf[0] << 8) | buf[1]);
    out->y = (int16_t)((buf[2] << 8) | buf[3]);
    out->z = (int16_t)((buf[4] << 8) | buf[5]);
    return MPU9250_OK;
}

MPU9250_Status MPU9250_ReadGyro(MPU9250_Handle *hdev, MPU9250_Vec3 *out)
{
    MPU9250_RawVec3 raw;
    MPU9250_Status st = MPU9250_ReadGyroRaw(hdev, &raw);
    if (st != MPU9250_OK) return st;

    float scale = gyro_scale(hdev->gyro_fs);
    out->x = raw.x * scale;
    out->y = raw.y * scale;
    out->z = raw.z * scale;
    return MPU9250_OK;
}

/* ══════════════════════════════════════════════════════════════════════════
 *  Temperature
 * ══════════════════════════════════════════════════════════════════════════ */

MPU9250_Status MPU9250_ReadTemp(MPU9250_Handle *hdev, float *temp_c)
{
    uint8_t buf[2];
    MPU9250_Status st = MPU9250_ReadRegs(hdev, MPU9250_REG_TEMP_OUT_H, buf, 2);
    if (st != MPU9250_OK) return st;

    int16_t raw = (int16_t)((buf[0] << 8) | buf[1]);
    /* Formula from datasheet: Temp(°C) = (TEMP_OUT / 333.87) + 21.0 */
    *temp_c = ((float)raw / 333.87f) + 21.0f;
    return MPU9250_OK;
}

/* ══════════════════════════════════════════════════════════════════════════
 *  Magnetometer (AK8963)
 * ══════════════════════════════════════════════════════════════════════════ */

MPU9250_Status MPU9250_ReadMagRaw(MPU9250_Handle *hdev, MPU9250_RawVec3 *out)
{
    uint8_t st1;
    /* Poll data-ready bit in ST1 (non-blocking: caller should check timing) */
    MPU9250_Status st = AK8963_ReadReg(hdev, AK8963_REG_ST1, &st1);
    if (st != MPU9250_OK) return st;
    if (!(st1 & 0x01)) return MPU9250_MAG_NOT_READY;  // DRDY=0 — no new sample yet

    /* Read HXL..HZH (6 bytes) plus ST2 in a single burst.
       IMPORTANT: ST2 must be read after data to unlock the next measurement. */
    uint8_t buf[7];
    st = AK8963_ReadRegs(hdev, AK8963_REG_HXL, buf, 7);
    if (st != MPU9250_OK) return st;

    uint8_t st2 = buf[6];
    if (st2 & 0x08) return MPU9250_ERR_MAG_OVF;   // HOFL overflow bit

    /* AK8963 is little-endian */
    out->x = (int16_t)((buf[1] << 8) | buf[0]);
    out->y = (int16_t)((buf[3] << 8) | buf[2]);
    out->z = (int16_t)((buf[5] << 8) | buf[4]);
    return MPU9250_OK;
}

MPU9250_Status MPU9250_ReadMag(MPU9250_Handle *hdev, MPU9250_Vec3 *out)
{
    MPU9250_RawVec3 raw;
    MPU9250_Status st = MPU9250_ReadMagRaw(hdev, &raw);
    if (st != MPU9250_OK) return st;

    float scale = mag_scale(hdev->mag_bits);
    out->x = raw.x * scale * hdev->mag_sens.x;
    out->y = raw.y * scale * hdev->mag_sens.y;
    out->z = raw.z * scale * hdev->mag_sens.z;
    return MPU9250_OK;
}

/* ══════════════════════════════════════════════════════════════════════════
 *  Read all sensors (burst)
 * ══════════════════════════════════════════════════════════════════════════ */

MPU9250_Status MPU9250_ReadAll(MPU9250_Handle *hdev,
                               MPU9250_Vec3   *accel,
                               MPU9250_Vec3   *gyro,
                               float          *temp_c,
                               MPU9250_Vec3   *mag)
{
    /* Burst read ACCEL + TEMP + GYRO in one 14-byte transaction */
    uint8_t buf[14];
    MPU9250_Status st = MPU9250_ReadRegs(hdev, MPU9250_REG_ACCEL_XOUT_H, buf, 14);
    if (st != MPU9250_OK) return st;

    float ascale = accel_scale(hdev->accel_fs);
    float gscale = gyro_scale(hdev->gyro_fs);

    if (accel) {
        accel->x = (int16_t)((buf[0]  << 8) | buf[1])  * ascale;
        accel->y = (int16_t)((buf[2]  << 8) | buf[3])  * ascale;
        accel->z = (int16_t)((buf[4]  << 8) | buf[5])  * ascale;
    }

    if (temp_c) {
        int16_t raw_t = (int16_t)((buf[6] << 8) | buf[7]);
        *temp_c = ((float)raw_t / 333.87f) + 21.0f;
    }

    if (gyro) {
        gyro->x = (int16_t)((buf[8]  << 8) | buf[9])  * gscale;
        gyro->y = (int16_t)((buf[10] << 8) | buf[11]) * gscale;
        gyro->z = (int16_t)((buf[12] << 8) | buf[13]) * gscale;
    }

    if (mag) {
        st = MPU9250_ReadMag(hdev, mag);
        /* MAG_NOT_READY is non-fatal: accel/gyro data is still valid */
        if (st != MPU9250_OK &&
            st != MPU9250_ERR_MAG_OVF &&
            st != MPU9250_MAG_NOT_READY) return st;
    }

    return MPU9250_OK;
}