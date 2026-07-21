#include "mpu9250.h"
#include "i2c_driver.h"
#include <math.h>

#ifndef PI
#define PI 3.14159265358979323846f
#endif

/* MPU-9250 registers (accel/gyro) */
#define MPU_REG_SMPLRT_DIV    0x19
#define MPU_REG_CONFIG        0x1A
#define MPU_REG_GYRO_CONFIG   0x1B
#define MPU_REG_ACCEL_CONFIG  0x1C
#define MPU_REG_ACCEL_CONFIG2 0x1D
#define MPU_REG_INT_PIN_CFG   0x37
#define MPU_REG_ACCEL_XOUT_H  0x3B
#define MPU_REG_GYRO_XOUT_H   0x43
#define MPU_REG_PWR_MGMT_1    0x6B
#define MPU_REG_PWR_MGMT_2    0x6C
#define MPU_REG_WHO_AM_I      0x75

/* AK8963 magnetometer (separate I2C device via bypass) */
#define AK8963_ADDR           0x0C
#define AK8963_REG_WIA        0x00   /* device ID = 0x48 */
#define AK8963_REG_ST1        0x02   /* bit0 DRDY */
#define AK8963_REG_HXL        0x03   /* measurements (little-endian!) */
#define AK8963_REG_ST2        0x09   /* must be read after measurement */
#define AK8963_REG_CNTL1      0x0A
#define AK8963_REG_CNTL2      0x0B
#define AK8963_REG_ASAX       0x10   /* factory sensitivity (3 bytes) */
#define AK8963_WIA_VAL        0x48

/* Scales */
#define GYRO_SENS   16.4f    /* gyro +-2000 deg/s -> 16.4 LSB/(deg/s) */
#define ACCEL_SENS  8192.0f  /* accel +-4g -> 8192 LSB/g */

/* Mahony filter gains */
#define MAHONY_KP   2.0f
#define MAHONY_KI   0.005f

/* Driver state */
static uint8_t  s_dev_addr   = 0x68;
static mpu_chip_t s_chip     = MPU_CHIP_UNKNOWN;
static uint8_t  s_has_mag    = 0;

/* Magnetometer factory sensitivity (ASA), per-axis correction */
static float s_mag_adj[3]    = { 1.0f, 1.0f, 1.0f };

/* Gyro bias (LSB), subtracted from each reading */
static float s_gyro_bias[3]  = { 0.0f, 0.0f, 0.0f };

/* Mahony filter quaternion state (w,x,y,z) */
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
/* Integral term for gyro bias compensation */
static float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;

/* Helpers */
static int16_t be16(const uint8_t *p)   /* big-endian (MPU accel/gyro) */
{
    return (int16_t)(((uint16_t)p[0] << 8) | p[1]);
}
static int16_t le16(const uint8_t *p)   /* little-endian (AK8963 mag) */
{
    return (int16_t)(((uint16_t)p[1] << 8) | p[0]);
}

/* Init AK8963 magnetometer via I2C bypass (9250/9255 only). Returns 0 on success. */
static int ak8963_init(void)
{
    uint8_t id = 0;

    /* Enable I2C bypass so the main bus sees AK8963 directly */
    if (I2C1_WriteByte(s_dev_addr, MPU_REG_INT_PIN_CFG, 0x02)) return -1;
    for (volatile int i = 0; i < 10000; i++);   /* short pause */

    if (I2C1_ReadReg(AK8963_ADDR, AK8963_REG_WIA, &id, 1)) return -1;
    if (id != AK8963_WIA_VAL) return -1;        /* no magnetometer */

    /* Power down, then fuse ROM access to read factory sensitivity */
    I2C1_WriteByte(AK8963_ADDR, AK8963_REG_CNTL1, 0x00);
    for (volatile int i = 0; i < 10000; i++);
    I2C1_WriteByte(AK8963_ADDR, AK8963_REG_CNTL1, 0x0F);   /* fuse ROM */
    for (volatile int i = 0; i < 10000; i++);

    uint8_t asa[3] = { 128, 128, 128 };
    I2C1_ReadReg(AK8963_ADDR, AK8963_REG_ASAX, asa, 3);
    for (int i = 0; i < 3; i++) {
        s_mag_adj[i] = ((float)asa[i] - 128.0f) / 256.0f + 1.0f;
    }

    /* Power down, then continuous 100 Hz 16-bit measurement (0x16) */
    I2C1_WriteByte(AK8963_ADDR, AK8963_REG_CNTL1, 0x00);
    for (volatile int i = 0; i < 10000; i++);
    I2C1_WriteByte(AK8963_ADDR, AK8963_REG_CNTL1, 0x16);
    for (volatile int i = 0; i < 10000; i++);

    return 0;
}

/* Detect MPU chip, reset and configure accel/gyro (and magnetometer if present). */
mpu_chip_t MPU_Init(void)
{
    uint8_t who = 0;

    /* Try both addresses */
    if (I2C1_Ping(0x68) == 0)      s_dev_addr = 0x68;
    else if (I2C1_Ping(0x69) == 0) s_dev_addr = 0x69;
    else return MPU_CHIP_UNKNOWN;

    if (I2C1_ReadReg(s_dev_addr, MPU_REG_WHO_AM_I, &who, 1)) return MPU_CHIP_UNKNOWN;

    switch (who) {
        case MPU_WHOAMI_9250: s_chip = MPU_CHIP_9250; break;
        case MPU_WHOAMI_9255: s_chip = MPU_CHIP_9255; break;
        case MPU_WHOAMI_6500: s_chip = MPU_CHIP_6500; break;
        default:              s_chip = MPU_CHIP_UNKNOWN; return MPU_CHIP_UNKNOWN;
    }

    /* Reset, wake, select gyro-X PLL clock */
    I2C1_WriteByte(s_dev_addr, MPU_REG_PWR_MGMT_1, 0x80);   /* H_RESET */
    for (volatile int i = 0; i < 200000; i++);
    I2C1_WriteByte(s_dev_addr, MPU_REG_PWR_MGMT_1, 0x01);   /* clk = gyro X PLL */
    for (volatile int i = 0; i < 50000; i++);
    I2C1_WriteByte(s_dev_addr, MPU_REG_PWR_MGMT_2, 0x00);   /* all sensors on */

    /* DLPF: gyro ~41 Hz (CONFIG=0x03), accel ~41 Hz (ACCEL_CONFIG2=0x03) */
    I2C1_WriteByte(s_dev_addr, MPU_REG_CONFIG, 0x03);
    I2C1_WriteByte(s_dev_addr, MPU_REG_SMPLRT_DIV, 0x04);   /* 1kHz/(1+4)=200 Hz */
    I2C1_WriteByte(s_dev_addr, MPU_REG_GYRO_CONFIG, 0x18);  /* +-2000 deg/s */
    I2C1_WriteByte(s_dev_addr, MPU_REG_ACCEL_CONFIG, 0x08); /* +-4g */
    I2C1_WriteByte(s_dev_addr, MPU_REG_ACCEL_CONFIG2, 0x03);

    /* Magnetometer only on 9250/9255 */
    s_has_mag = 0;
    if (s_chip == MPU_CHIP_9250 || s_chip == MPU_CHIP_9255) {
        if (ak8963_init() == 0) s_has_mag = 1;
    }

    return s_chip;
}

/* Return 1 if magnetometer is present. */
uint8_t MPU_HasMag(void) { return s_has_mag; }

/* Read WHO_AM_I, trying the detected address then 0x68/0x69. */
uint8_t MPU_WhoAmI(void)
{
    uint8_t who = 0xFF;
    /* Try detected address; fall back to 0x68/0x69 */
    uint8_t addr = s_dev_addr;
    if (I2C1_ReadReg(addr, MPU_REG_WHO_AM_I, &who, 1)) {
        if (I2C1_ReadReg(0x68, MPU_REG_WHO_AM_I, &who, 1)) {
            if (I2C1_ReadReg(0x69, MPU_REG_WHO_AM_I, &who, 1)) return 0xFF;
        }
    }
    return who;
}

/* Return the detected I2C address. */
uint8_t MPU_Address(void) { return s_dev_addr; }

/* Read accel (g) and gyro (deg/s) in sensor frame. Returns 0 on success. */
static int read_accel_gyro(float a[3], float g[3])
{
    uint8_t buf[14];
    if (I2C1_ReadReg(s_dev_addr, MPU_REG_ACCEL_XOUT_H, buf, 14)) return -1;

    a[0] = (float)be16(&buf[0])  / ACCEL_SENS;
    a[1] = (float)be16(&buf[2])  / ACCEL_SENS;
    a[2] = (float)be16(&buf[4])  / ACCEL_SENS;
    /* buf[6..7] = temperature, skipped */
    g[0] = ((float)be16(&buf[8])  - s_gyro_bias[0]) / GYRO_SENS;
    g[1] = ((float)be16(&buf[10]) - s_gyro_bias[1]) / GYRO_SENS;
    g[2] = ((float)be16(&buf[12]) - s_gyro_bias[2]) / GYRO_SENS;
    return 0;
}

/* Read magnetometer (uT, ASA-scaled). Returns 0 ok, !=0 missing/not ready. */
static int read_mag(float m[3])
{
    uint8_t st1 = 0;
    if (!s_has_mag) return -1;
    if (I2C1_ReadReg(AK8963_ADDR, AK8963_REG_ST1, &st1, 1)) return -1;
    if ((st1 & 0x01) == 0) return -2;   /* DRDY=0, no new data */

    uint8_t buf[7];   /* HXL..HZH + ST2 */
    if (I2C1_ReadReg(AK8963_ADDR, AK8963_REG_HXL, buf, 7)) return -1;
    if (buf[6] & 0x08) return -3;        /* ST2 HOFL = magnetic overflow */

    /* AK8963 axes differ from MPU: mag_X=sensor_Y, mag_Y=sensor_X, mag_Z=-sensor_Z. */
    float mx = (float)le16(&buf[0]) * s_mag_adj[0];
    float my = (float)le16(&buf[2]) * s_mag_adj[1];
    float mz = (float)le16(&buf[4]) * s_mag_adj[2];
    m[0] =  my;
    m[1] =  mx;
    m[2] = -mz;
    return 0;
}

/* Average N gyro samples to estimate bias (board must be still). */
void MPU_CalibrateGyro(void)
{
    float sum[3] = { 0.0f, 0.0f, 0.0f };
    const int N = 256;
    uint8_t buf[14];

    s_gyro_bias[0] = s_gyro_bias[1] = s_gyro_bias[2] = 0.0f;

    for (int n = 0; n < N; n++) {
        if (I2C1_ReadReg(s_dev_addr, MPU_REG_ACCEL_XOUT_H, buf, 14) == 0) {
            sum[0] += (float)be16(&buf[8]);
            sum[1] += (float)be16(&buf[10]);
            sum[2] += (float)be16(&buf[12]);
        }
        for (volatile int i = 0; i < 20000; i++);   /* spread samples */
    }
    s_gyro_bias[0] = sum[0] / (float)N;
    s_gyro_bias[1] = sum[1] / (float)N;
    s_gyro_bias[2] = sum[2] / (float)N;
}

/* Reciprocal square root. */
static float inv_sqrt(float x)
{
    return 1.0f / sqrtf(x);
}

/* Mahony AHRS update from gyro/accel (+mag if available); integrates the quaternion. */
static void mahony_update(float gx, float gy, float gz,
                          float ax, float ay, float az,
                          float mx, float my, float mz,
                          int use_mag, float dt)
{
    /* gyro deg/s -> rad/s */
    gx *= (PI / 180.0f);
    gy *= (PI / 180.0f);
    gz *= (PI / 180.0f);

    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex = 0.0f, halfey = 0.0f, halfez = 0.0f;

    /* Normalize accel (only if nonzero) */
    if (!(ax == 0.0f && ay == 0.0f && az == 0.0f)) {
        recipNorm = inv_sqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

        /* Estimated gravity direction from quaternion */
        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5f + q3 * q3;

        /* Error = cross(measured gravity, estimated gravity) */
        halfex += (ay * halfvz - az * halfvy);
        halfey += (az * halfvx - ax * halfvz);
        halfez += (ax * halfvy - ay * halfvx);
    }

    /* Magnetometer corrects yaw */
    if (use_mag && !(mx == 0.0f && my == 0.0f && mz == 0.0f)) {
        recipNorm = inv_sqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm; my *= recipNorm; mz *= recipNorm;

        /* Reference direction of Earth's magnetic field */
        float hx = 2.0f * (mx * (0.5f - q2 * q2 - q3 * q3) + my * (q1 * q2 - q0 * q3) + mz * (q1 * q3 + q0 * q2));
        float hy = 2.0f * (mx * (q1 * q2 + q0 * q3) + my * (0.5f - q1 * q1 - q3 * q3) + mz * (q2 * q3 - q0 * q1));
        float bx = sqrtf(hx * hx + hy * hy);
        float bz = 2.0f * (mx * (q1 * q3 - q0 * q2) + my * (q2 * q3 + q0 * q1) + mz * (0.5f - q1 * q1 - q2 * q2));

        float halfwx = bx * (0.5f - q2 * q2 - q3 * q3) + bz * (q1 * q3 - q0 * q2);
        float halfwy = bx * (q1 * q2 - q0 * q3) + bz * (q0 * q1 + q2 * q3);
        float halfwz = bx * (q0 * q2 + q1 * q3) + bz * (0.5f - q1 * q1 - q2 * q2);

        halfex += (my * halfwz - mz * halfwy);
        halfey += (mz * halfwx - mx * halfwz);
        halfez += (mx * halfwy - my * halfwx);
    }

    /* PI controller: integral term compensates gyro bias */
    if (MAHONY_KI > 0.0f) {
        integralFBx += MAHONY_KI * halfex * dt;
        integralFBy += MAHONY_KI * halfey * dt;
        integralFBz += MAHONY_KI * halfez * dt;
        gx += integralFBx;
        gy += integralFBy;
        gz += integralFBz;
    }
    gx += MAHONY_KP * halfex;
    gy += MAHONY_KP * halfey;
    gz += MAHONY_KP * halfez;

    /* Integrate quaternion */
    gx *= 0.5f * dt; gy *= 0.5f * dt; gz *= 0.5f * dt;
    float qa = q0, qb = q1, qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += ( qa * gx + qc * gz - q3 * gy);
    q2 += ( qa * gy - qb * gz + q3 * gx);
    q3 += ( qa * gz + qb * gy - qc * gx);

    recipNorm = inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm; q1 *= recipNorm; q2 *= recipNorm; q3 *= recipNorm;
}

/* Read sensors, run Mahony update, output roll/pitch/yaw (deg). Returns 0 on success. */
int MPU_Update(float dt_s, mpu_orientation_t *out)
{
    float a[3], g[3], m[3] = { 0.0f, 0.0f, 0.0f };

    if (read_accel_gyro(a, g)) return -1;
    int mag_ok = (read_mag(m) == 0);

    mahony_update(g[0], g[1], g[2],
                  a[0], a[1], a[2],
                  m[0], m[1], m[2],
                  mag_ok, dt_s);

    /* Quaternion -> Tait-Bryan angles (ZYX) */
    float roll  = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2));
    float pitch = asinf(2.0f * (q0 * q2 - q3 * q1));
    float yaw   = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3));

    out->roll_deg  = roll  * (180.0f / PI);
    out->pitch_deg = pitch * (180.0f / PI);

    float yaw_deg = yaw * (180.0f / PI);
    if (yaw_deg < 0.0f) yaw_deg += 360.0f;   /* wrap to 0..360 */
    out->yaw_deg = yaw_deg;

    return 0;
}
