
#include "stm32f4xx.h"
#include "MPU6050.h"
#include <math.h>

uint8_t IMU_begin(struct MPU6050_struct *IMU, mpu6050_dps_t scale, mpu6050_range_t range, int mpua)
{
    // Set Address
    IMU->bus_address = mpua;


    // Reset calibrate values
    IMU->dg.XAxis = 0;
    IMU->dg.YAxis = 0;
    IMU->dg.ZAxis = 0;
    IMU->useCalibrate = 0;

    // Reset threshold values
    IMU->tg.XAxis = 0;
    IMU->tg.YAxis = 0;
    IMU->tg.ZAxis = 0;
    IMU->actualThreshold = 0;

    // Check MPU6050 Who Am I Register
    if (IMU_readReg(IMU, MPU6050_REG_WHO_AM_I) != 0x68)
    {
    	return 0;
    }

    // Set Clock Source
    IMU_setClockSource(IMU, MPU6050_CLOCK_PLL_XGYRO);

    // Set Scale & Range
    IMU_setScale(IMU, scale);
    IMU_setRange(IMU, range);

    // Disable Sleep Mode
    IMU_setSleepEnabled(IMU, 0);

    return 1;
}

void IMU_setDLPFMode(struct MPU6050_struct *IMU, mpu6050_dlpf_t dlpf)
{
    uint8_t value;
    value = IMU_readReg(IMU, MPU6050_REG_CONFIG);
    value &= 0b11111000;
    value |= dlpf;
    IMU_writeReg(IMU, MPU6050_REG_CONFIG, value);
}

void IMU_setSleepEnabled(struct MPU6050_struct *IMU, uint8_t state)
{
    IMU_writeRegisterBit(IMU, MPU6050_REG_PWR_MGMT_1, 6, state);
}

void IMU_setRange(struct MPU6050_struct *IMU, mpu6050_range_t range)
{
    uint8_t value;

    switch (range)
    {
	case MPU6050_RANGE_2G:
	    IMU->rangePerDigit = .000061f;
	    break;
	case MPU6050_RANGE_4G:
		IMU->rangePerDigit = .000122f;
	    break;
	case MPU6050_RANGE_8G:
		IMU->rangePerDigit = .000244f;
	    break;
	case MPU6050_RANGE_16G:
		IMU->rangePerDigit = .0004882f;
	    break;
	default:
	    break;
    }

    value = IMU_readReg(IMU, MPU6050_REG_ACCEL_CONFIG);
    value &= 0b11100111;
    value |= (range << 3);
    IMU_writeReg(IMU, MPU6050_REG_ACCEL_CONFIG, value);
}

void IMU_setScale(struct MPU6050_struct *IMU, mpu6050_dps_t scale)
{
    uint8_t value;

    switch (scale)
    {
	case MPU6050_SCALE_250DPS:
		IMU->dpsPerDigit = .007633f;
	    break;
	case MPU6050_SCALE_500DPS:
		IMU->dpsPerDigit = .015267f;
	    break;
	case MPU6050_SCALE_1000DPS:
		IMU->dpsPerDigit = .030487f;
	    break;
	case MPU6050_SCALE_2000DPS:
		IMU->dpsPerDigit = .060975f;
	    break;
	default:
	    break;
    }

    value = IMU_readReg(IMU, MPU6050_REG_GYRO_CONFIG);
    value &= 0b11100111;
    value |= (scale << 3);
    IMU_writeReg(IMU, MPU6050_REG_GYRO_CONFIG, value);
}

void IMU_setClockSource(struct MPU6050_struct *IMU, mpu6050_clockSource_t source)
{
    uint8_t value;
    value = IMU_readReg(IMU, MPU6050_REG_PWR_MGMT_1);
    value &= 0b11111000;
    value |= source;
    IMU_writeReg(IMU, MPU6050_REG_PWR_MGMT_1, value);
}

void IMU_writeReg(struct MPU6050_struct *IMU, uint8_t reg, uint8_t value)
{
	I2C_GenerateSTART(I2C1, ENABLE);                                                          //wygenerowanie sygna³u START
	while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) != SUCCESS);                    //czekanie na zakoñczenie

	I2C_Send7bitAddress(I2C1, (IMU->bus_address << 1), I2C_Direction_Transmitter);            //rozpoczêcie transmisji do danego adresu
	while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) != SUCCESS);

	I2C_SendData(I2C1, reg);                                                                  //rejestr, do którego chcê pisac
	while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTING) != SUCCESS);

	I2C_SendData(I2C1, value);                                                                //wpisanie danych do rejestru
	while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTING) != SUCCESS);

	I2C_GenerateSTOP(I2C1, ENABLE);                                                           //koniec transmisji
}

void IMU_writeReg16Bit(struct MPU6050_struct *IMU, uint8_t reg, uint16_t value)
{
	I2C_GenerateSTART(I2C1, ENABLE);                                                          //wygenerowanie sygna³u START
	while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) != SUCCESS);                    //czekanie na zakoñczenie

	I2C_Send7bitAddress(I2C1, (IMU->bus_address << 1), I2C_Direction_Transmitter);            //rozpoczêcie transmisji do danego adresu
	while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) != SUCCESS);

	I2C_SendData(I2C1, reg);                                                                  //rejestr, do którego chcê pisac
	while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTING) != SUCCESS);

	I2C_SendData(I2C1, (value >> 8) & 0xFF);                                                  //wpisanie danych do rejestru
	while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTING) != SUCCESS);

	I2C_SendData(I2C1, (value       & 0xFF));                                                 //wpisanie danych do rejestru
	while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTING) != SUCCESS);

	I2C_GenerateSTOP(I2C1, ENABLE);                                                           //koniec transmisji
}

uint8_t IMU_readReg(struct MPU6050_struct *IMU, uint8_t reg)
{
	uint8_t value;

	I2C_GenerateSTART(I2C1, ENABLE);                                                          //wygenerowanie sygna³u START
	while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) != SUCCESS);                    //czekanie na zakoñczenie

	I2C_Send7bitAddress(I2C1, (IMU->bus_address << 1), I2C_Direction_Transmitter);            //rozpoczêcie transmisji do danego adresu
	while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) != SUCCESS);

	I2C_SendData(I2C1, reg);                                                                  //rejestr, do którego chcê pisac
	while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTING) != SUCCESS);

	I2C_GenerateSTOP(I2C1, ENABLE);                                                           //koniec transmisji


    I2C_GenerateSTART(I2C1, ENABLE);                                                          //wygenerowanie sygna³u START
    while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) != SUCCESS);

    I2C_AcknowledgeConfig(I2C1, ENABLE);                                                      //aktywacja potwierdzeñ ACK po ka¿dym bajcie

	I2C_Send7bitAddress(I2C1, (IMU->bus_address << 1), I2C_Direction_Receiver);               //zarz¹danie transmisji z adresu
	while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) != SUCCESS);

	I2C_AcknowledgeConfig(I2C1, DISABLE);                                                //po odebraniu najbli¿szego bajtu wyœlij NACK i STOP
	I2C_GenerateSTOP(I2C1, ENABLE);

    while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED) != SUCCESS);              //czekaj, dopóki nie odebrano bajtu
    value = I2C_ReceiveData(I2C1);
	return value;
}

uint16_t IMU_readReg16Bit(struct MPU6050_struct *IMU, uint8_t reg)
{
	uint16_t value = -1;
	uint8_t value_tmp = -1;

	I2C_GenerateSTART(I2C1, ENABLE);                                                          //wygenerowanie sygna³u START
	while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) != SUCCESS);                    //czekanie na zakoñczenie

	I2C_Send7bitAddress(I2C1, (IMU->bus_address << 1), I2C_Direction_Transmitter);            //rozpoczêcie transmisji do danego adresu
	while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) != SUCCESS);

	I2C_SendData(I2C1, reg);                                                                  //rejestr, z którego chcê czytac
	while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTING) != SUCCESS);

	I2C_GenerateSTOP(I2C1, ENABLE);                                                           //koniec transmisji



	I2C_GenerateSTART(I2C1, ENABLE);                                                     //wygenerowanie sygna³u START
	while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) != SUCCESS);

	I2C_AcknowledgeConfig(I2C1, ENABLE);                                                 //aktywacja potwierdzeñ ACK po ka¿dym bajcie

	I2C_Send7bitAddress(I2C1, (IMU->bus_address << 1), I2C_Direction_Receiver);          //zarz¹danie transmisji z adresu
	while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) != SUCCESS);

	while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED) != SUCCESS);              //czekaj, dopóki nie odebrano bajtu
	value = (uint16_t)I2C_ReceiveData(I2C1);
	value = value << 8;

	I2C_AcknowledgeConfig(I2C1, DISABLE);                                                //po odebraniu najbli¿szego bajtu wyœlij NACK i STOP
	I2C_GenerateSTOP(I2C1, ENABLE);

	while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED) != SUCCESS);              //czekaj, dopóki nie odebrano bajtu
	value_tmp = (uint16_t)I2C_ReceiveData(I2C1);
	value |= value_tmp;

	return value;
}

struct Vector IMU_readRawGyro(struct MPU6050_struct *IMU)
{
	I2C_GenerateSTART(I2C1, ENABLE);                                                          //wygenerowanie sygna³u START
	while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) != SUCCESS);                    //czekanie na zakoñczenie

	I2C_Send7bitAddress(I2C1, (IMU->bus_address << 1), I2C_Direction_Transmitter);            //rozpoczêcie transmisji do danego adresu
	while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) != SUCCESS);

	I2C_SendData(I2C1, MPU6050_REG_GYRO_XOUT_H);                                                                  //rejestr, z którego chcê czytac
	while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTING) != SUCCESS);

	I2C_GenerateSTOP(I2C1, ENABLE);                                                           //koniec transmisji

	I2C_GenerateSTART(I2C1, ENABLE);                                                     //wygenerowanie sygna³u START
	while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) != SUCCESS);

	I2C_AcknowledgeConfig(I2C1, ENABLE);                                                 //aktywacja potwierdzeñ ACK po ka¿dym bajcie

	I2C_Send7bitAddress(I2C1, (IMU->bus_address << 1), I2C_Direction_Receiver);          //zarz¹danie transmisji z adresu
	while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) != SUCCESS);

	while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED) != SUCCESS);              //czekaj, dopóki nie odebrano bajtu 1
	uint8_t xha = I2C_ReceiveData(I2C1);

	while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED) != SUCCESS);              //czekaj, dopóki nie odebrano bajtu 2
	uint8_t xla = I2C_ReceiveData(I2C1);

	while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED) != SUCCESS);              //czekaj, dopóki nie odebrano bajtu 3
	uint8_t yha = I2C_ReceiveData(I2C1);

	while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED) != SUCCESS);              //czekaj, dopóki nie odebrano bajtu 4
	uint8_t yla = I2C_ReceiveData(I2C1);

	while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED) != SUCCESS);              //czekaj, dopóki nie odebrano bajtu 5
	uint8_t zha = I2C_ReceiveData(I2C1);

	I2C_AcknowledgeConfig(I2C1, DISABLE);                                                //po odebraniu najbli¿szego bajtu wyœlij NACK i STOP
	I2C_GenerateSTOP(I2C1, ENABLE);

	while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED) != SUCCESS);              //czekaj, dopóki nie odebrano bajtu 6
	uint8_t zla = I2C_ReceiveData(I2C1);

    IMU->rg.XAxis = xha << 8 | xla;
    IMU->rg.YAxis = yha << 8 | yla;
    IMU->rg.ZAxis = zha << 8 | zla;

    return IMU->rg;
}

struct Vector IMU_readNormalizeGyro(struct MPU6050_struct *IMU)
{
    IMU_readRawGyro(IMU);

    if (IMU->useCalibrate == 1)
    {
    	int16_t rawX = IMU->rg.XAxis;
    	int16_t rawY = IMU->rg.YAxis;
    	int16_t rawZ = IMU->rg.ZAxis;

    	IMU->ng.XAxis = (rawX - IMU->dg.XAxis) * IMU->dpsPerDigit;
    	IMU->ng.YAxis = (rawY - IMU->dg.YAxis) * IMU->dpsPerDigit;
    	IMU->ng.ZAxis = (rawZ - IMU->dg.ZAxis) * IMU->dpsPerDigit;
    } else
    {
    	IMU->ng.XAxis = IMU->rg.XAxis * IMU->dpsPerDigit;
    	IMU->ng.YAxis = IMU->rg.YAxis * IMU->dpsPerDigit;
    	IMU->ng.ZAxis = IMU->rg.ZAxis * IMU->dpsPerDigit;
    }

    if (IMU->actualThreshold == 1)
    {
    	if (abs(IMU->ng.XAxis) < IMU->tg.XAxis) IMU->ng.XAxis = 0;
    	if (abs(IMU->ng.YAxis) < IMU->tg.YAxis) IMU->ng.YAxis = 0;
    	if (abs(IMU->ng.ZAxis) < IMU->tg.ZAxis) IMU->ng.ZAxis = 0;
    }

    return IMU->ng;
}

float IMU_readTemperature(struct MPU6050_struct *IMU)
{
    int16_t T;
    T = IMU_readReg16Bit(IMU, MPU6050_REG_TEMP_OUT_H);
    return (float)T/340 + 36.53;
}

void IMU_calibrateGyro(struct MPU6050_struct *IMU, uint8_t samples)
{
    // Set calibrate
    IMU->useCalibrate = 1;

    // Reset values
    float sumX = 0;
    float sumY = 0;
    float sumZ = 0;
    float sigmaX = 0;
    float sigmaY = 0;
    float sigmaZ = 0;

    // Read n-samples
    for (uint8_t i = 0; i < samples; ++i)
    {
    	IMU_readRawGyro(IMU);
    	sumX += IMU->rg.XAxis;
    	sumY += IMU->rg.YAxis;
    	sumZ += IMU->rg.ZAxis;

    	sigmaX += IMU->rg.XAxis * IMU->rg.XAxis;
    	sigmaY += IMU->rg.YAxis * IMU->rg.YAxis;
    	sigmaZ += IMU->rg.ZAxis * IMU->rg.ZAxis;
    	//delay(5);
    }

    // Calculate delta vectors
    IMU->dg.XAxis = sumX / samples;
    IMU->dg.YAxis = sumY / samples;
    IMU->dg.ZAxis = sumZ / samples;

    // Calculate threshold vectors
    IMU->th.XAxis = sqrt((sigmaX / 50) - (IMU->dg.XAxis * IMU->dg.XAxis));
    IMU->th.YAxis = sqrt((sigmaY / 50) - (IMU->dg.YAxis * IMU->dg.YAxis));
    IMU->th.ZAxis = sqrt((sigmaZ / 50) - (IMU->dg.ZAxis * IMU->dg.ZAxis));

    // If already set threshold, recalculate threshold vectors
    if (IMU->actualThreshold > 0)
    {
    	IMU_setThreshold(IMU, IMU->actualThreshold);
    }
}

void IMU_setThreshold(struct MPU6050_struct *IMU, uint8_t multiple)
{
    if (multiple > 0)
    {
    	// If not calibrated, need calibrate
    	if (IMU->useCalibrate == 0)
    	{
    		IMU_calibrateGyro(IMU, 10);
    	}

    	// Calculate threshold vectors
    	IMU->tg.XAxis = IMU->th.XAxis * multiple;
    	IMU->tg.YAxis = IMU->th.YAxis * multiple;
    	IMU->tg.ZAxis = IMU->th.ZAxis * multiple;
    }
    else
    {
    	// No threshold
    	IMU->tg.XAxis = 0;
    	IMU->tg.YAxis = 0;
    	IMU->tg.ZAxis = 0;
    }

    // Remember old threshold value
    IMU->actualThreshold = multiple;
}

void IMU_writeRegisterBit(struct MPU6050_struct *IMU, uint8_t reg, uint8_t pos, uint8_t state)
{
    uint8_t value;
    value = IMU_readReg(IMU, reg);

    if (state == 1)
    {
        value |= (1 << pos);
    }
    else
    {
        value &= ~(1 << pos);
    }

    IMU_writeReg(IMU, reg, value);
}
