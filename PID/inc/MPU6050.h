#include "stm32f4xx.h"

#ifndef MPU6050_H_
#define MPU6050_H_

#define MPU6050_ADDRESS               (0x68) // 0x69 when AD0 pin to Vcc
#define MPU6050_REG_ACCEL_XOFFS_H     (0x06)
#define MPU6050_REG_ACCEL_XOFFS_L     (0x07)
#define MPU6050_REG_ACCEL_YOFFS_H     (0x08)
#define MPU6050_REG_ACCEL_YOFFS_L     (0x09)
#define MPU6050_REG_ACCEL_ZOFFS_H     (0x0A)
#define MPU6050_REG_ACCEL_ZOFFS_L     (0x0B)
#define MPU6050_REG_GYRO_XOFFS_H      (0x13)
#define MPU6050_REG_GYRO_XOFFS_L      (0x14)
#define MPU6050_REG_GYRO_YOFFS_H      (0x15)
#define MPU6050_REG_GYRO_YOFFS_L      (0x16)
#define MPU6050_REG_GYRO_ZOFFS_H      (0x17)
#define MPU6050_REG_GYRO_ZOFFS_L      (0x18)
#define MPU6050_REG_CONFIG            (0x1A)
#define MPU6050_REG_GYRO_CONFIG       (0x1B) // Gyroscope Configuration
#define MPU6050_REG_ACCEL_CONFIG      (0x1C) // Accelerometer Configuration
#define MPU6050_REG_FF_THRESHOLD      (0x1D)
#define MPU6050_REG_FF_DURATION       (0x1E)
#define MPU6050_REG_MOT_THRESHOLD     (0x1F)
#define MPU6050_REG_MOT_DURATION      (0x20)
#define MPU6050_REG_ZMOT_THRESHOLD    (0x21)
#define MPU6050_REG_ZMOT_DURATION     (0x22)
#define MPU6050_REG_INT_PIN_CFG       (0x37) // INT Pin. Bypass Enable Configuration
#define MPU6050_REG_INT_ENABLE        (0x38) // INT Enable
#define MPU6050_REG_INT_STATUS        (0x3A)
#define MPU6050_REG_ACCEL_XOUT_H      (0x3B)
#define MPU6050_REG_ACCEL_XOUT_L      (0x3C)
#define MPU6050_REG_ACCEL_YOUT_H      (0x3D)
#define MPU6050_REG_ACCEL_YOUT_L      (0x3E)
#define MPU6050_REG_ACCEL_ZOUT_H      (0x3F)
#define MPU6050_REG_ACCEL_ZOUT_L      (0x40)
#define MPU6050_REG_TEMP_OUT_H        (0x41)
#define MPU6050_REG_TEMP_OUT_L        (0x42)
#define MPU6050_REG_GYRO_XOUT_H       (0x43)
#define MPU6050_REG_GYRO_XOUT_L       (0x44)
#define MPU6050_REG_GYRO_YOUT_H       (0x45)
#define MPU6050_REG_GYRO_YOUT_L       (0x46)
#define MPU6050_REG_GYRO_ZOUT_H       (0x47)
#define MPU6050_REG_GYRO_ZOUT_L       (0x48)
#define MPU6050_REG_MOT_DETECT_STATUS (0x61)
#define MPU6050_REG_MOT_DETECT_CTRL   (0x69)
#define MPU6050_REG_USER_CTRL         (0x6A) // User Control
#define MPU6050_REG_PWR_MGMT_1        (0x6B) // Power Management 1
#define MPU6050_REG_WHO_AM_I          (0x75) // Who Am I

struct Vector
{
    float XAxis;
    float YAxis;
    float ZAxis;
};

struct Activites
{
    uint8_t isOverflow;
    uint8_t isFreeFall;
    uint8_t isInactivity;
    uint8_t isActivity;
    uint8_t isPosActivityOnX;
    uint8_t isPosActivityOnY;
    uint8_t isPosActivityOnZ;
    uint8_t isNegActivityOnX;
    uint8_t isNegActivityOnY;
    uint8_t isNegActivityOnZ;
    uint8_t isDataReady;
};

typedef enum
{
    MPU6050_CLOCK_KEEP_RESET      = 0b111,
    MPU6050_CLOCK_EXTERNAL_19MHZ  = 0b101,
    MPU6050_CLOCK_EXTERNAL_32KHZ  = 0b100,
    MPU6050_CLOCK_PLL_ZGYRO       = 0b011,
    MPU6050_CLOCK_PLL_YGYRO       = 0b010,
    MPU6050_CLOCK_PLL_XGYRO       = 0b001,
    MPU6050_CLOCK_INTERNAL_8MHZ   = 0b000
} mpu6050_clockSource_t;

typedef enum
{
    MPU6050_SCALE_2000DPS         = 0b11,
    MPU6050_SCALE_1000DPS         = 0b10,
    MPU6050_SCALE_500DPS          = 0b01,
    MPU6050_SCALE_250DPS          = 0b00
} mpu6050_dps_t;

typedef enum
{
    MPU6050_RANGE_16G             = 0b11,
    MPU6050_RANGE_8G              = 0b10,
    MPU6050_RANGE_4G              = 0b01,
    MPU6050_RANGE_2G              = 0b00,
} mpu6050_range_t;

typedef enum
{
    MPU6050_DELAY_3MS             = 0b11,
    MPU6050_DELAY_2MS             = 0b10,
    MPU6050_DELAY_1MS             = 0b01,
    MPU6050_NO_DELAY              = 0b00,
} mpu6050_onDelay_t;

typedef enum
{
    MPU6050_DHPF_HOLD             = 0b111,
    MPU6050_DHPF_0_63HZ           = 0b100,
    MPU6050_DHPF_1_25HZ           = 0b011,
    MPU6050_DHPF_2_5HZ            = 0b010,
    MPU6050_DHPF_5HZ              = 0b001,
    MPU6050_DHPF_RESET            = 0b000,
} mpu6050_dhpf_t;

typedef enum
{
    MPU6050_DLPF_6                = 0b110,
    MPU6050_DLPF_5                = 0b101,
    MPU6050_DLPF_4                = 0b100,
    MPU6050_DLPF_3                = 0b011,
    MPU6050_DLPF_2                = 0b010,
    MPU6050_DLPF_1                = 0b001,
    MPU6050_DLPF_0                = 0b000,
} mpu6050_dlpf_t;

struct MPU6050_struct
{
	struct Vector ra, rg; // Raw vectors
	struct Vector na, ng; // Normalized vectors
	struct Vector tg, dg; // Threshold and Delta for Gyro
	struct Vector th;     // Threshold
	struct Activites a;   // Activities

	float dpsPerDigit, rangePerDigit;
	float actualThreshold;
	uint8_t useCalibrate;
	uint8_t bus_address;
};

uint8_t IMU_begin(struct MPU6050_struct *IMU, mpu6050_dps_t scale, mpu6050_range_t range, int mpua);
void IMU_setDLPFMode(struct MPU6050_struct *IMU, mpu6050_dlpf_t dlpf);
void IMU_setSleepEnabled(struct MPU6050_struct *IMU, uint8_t state);
void IMU_setRange(struct MPU6050_struct *IMU, mpu6050_range_t range);
void IMU_setScale(struct MPU6050_struct *IMU, mpu6050_dps_t scale);
void IMU_setClockSource(struct MPU6050_struct *IMU, mpu6050_clockSource_t source);

void IMU_writeReg(struct MPU6050_struct *IMU, uint8_t reg, uint8_t value);
void IMU_writeReg16Bit(struct MPU6050_struct *IMU, uint8_t reg, uint16_t value);
uint8_t IMU_readReg(struct MPU6050_struct *IMU, uint8_t reg);
uint16_t IMU_readReg16Bit(struct MPU6050_struct *IMU, uint8_t reg);

struct Vector IMU_readRawGyro(struct MPU6050_struct *IMU);
struct Vector IMU_readNormalizeGyro(struct MPU6050_struct *IMU);
float IMU_readTemperature(struct MPU6050_struct *IMU);
void IMU_calibrateGyro(struct MPU6050_struct *IMU, uint8_t samples);
void IMU_setThreshold(struct MPU6050_struct *IMU, uint8_t multiple);
void IMU_writeRegisterBit(struct MPU6050_struct *IMU, uint8_t reg, uint8_t pos, uint8_t state);


#endif /* MPU6050_H_ */
