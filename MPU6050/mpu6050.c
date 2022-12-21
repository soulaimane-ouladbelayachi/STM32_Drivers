/*
 * mpu6050.c
 *
 *  Created on: Aug 5, 2022
 *      Author: Soulaimane OuladBelayachi
 */


//Header files
#include "mpu6050.h"

//Library Variable
//1- I2C Handle
static I2C_HandleTypeDef i2cHandler;
//2- Accel & Gyro Scaling Factor
static float accelScalingFactor, gyroScalingFactor;

static int16_t GyroRW[3];

//Fucntion Definitions
//1- i2c Handler
void MPU6050_Init(I2C_HandleTypeDef *I2Chnd)
{
	//Copy I2C CubeMX handle to local library
	memcpy(&i2cHandler, I2Chnd, sizeof(*I2Chnd));
}

//2- i2c Read
void I2C_Read(uint8_t ADDR, uint8_t *i2cBif, uint8_t NofData)
{
	uint8_t i2cBuf[2];
	uint8_t MPUADDR;
	//Need to Shift address to make it proper to i2c operation
	MPUADDR = (MPU_ADDR<<1);
	i2cBuf[0] = ADDR;
	HAL_I2C_Master_Transmit(&i2cHandler, MPUADDR, i2cBuf, 1, 10);
	HAL_I2C_Master_Receive(&i2cHandler, MPUADDR, i2cBif, NofData, 100);
}

//3- i2c Write
void I2C_Write8(uint8_t ADDR, uint8_t data)
{
	uint8_t i2cData[2];
	i2cData[0] = ADDR;
	i2cData[1] = data;
	uint8_t MPUADDR = (MPU_ADDR<<1);
	HAL_I2C_Master_Transmit(&i2cHandler, MPUADDR, i2cData, 2,100);
}

//4- MPU6050 Initialaztion Configuration
void MPU6050_Config(MPU_ConfigTypeDef *config)
{
	uint8_t Buffer = 0;
	//Clock Source
	//Reset Device
	I2C_Write8(PWR_MAGT_1_REG, 1<<7);
	HAL_Delay(100);
	Buffer = config ->ClockSource & 0x07; //change the 7th bits of register
	Buffer |= (config ->Sleep_Mode_Bit << 6) & 0x40; // change only the 7th bit in the register
	I2C_Write8(PWR_MAGT_1_REG, Buffer);
	HAL_Delay(100); // should wait 100ms after changing the clock setting.

	//Set the Digital Low Pass Filter
	Buffer = 0;
	Buffer = config->CONFIG_DLPF & 0x07;
	I2C_Write8(CONFIG_REG, Buffer);

	//Select the Gyroscope Full Scale Range
	Buffer = 0;
	Buffer = (config->Gyro_Full_Scale << 3) & 0x18;
	I2C_Write8(GYRO_CONFIG_REG, Buffer);

	//Select the Accelerometer Full Scale Range
	Buffer = 0;
	Buffer = (config->Accel_Full_Scale << 3) & 0x18;
	I2C_Write8(ACCEL_CONFIG_REG, Buffer);
	//Set SRD To Default
	MPU6050_Set_SMPRT_DIV(0x04);


	//Accelerometer Scaling Factor, Set the Accelerometer and Gyroscope Scaling Factor
	switch (config->Accel_Full_Scale)
	{
		case AFS_SEL_2g:
			accelScalingFactor = (2.0f/32768.0f);
			break;

		case AFS_SEL_4g:
			accelScalingFactor = (4.0f/32768.0f);
				break;

		case AFS_SEL_8g:
			accelScalingFactor = (8.0f/32768.0f);
			break;

		case AFS_SEL_16g:
			accelScalingFactor = (16.0f/32768.0f);
			break;

		default:
			break;
	}
	//Gyroscope Scaling Factor
	switch (config->Gyro_Full_Scale)
	{
		case FS_SEL_250:
			gyroScalingFactor = 250.0f/32768.0f;
			break;

		case FS_SEL_500:
				gyroScalingFactor = 500.0f/32768.0f;
				break;

		case FS_SEL_1000:
			gyroScalingFactor = 1000.0f/32768.0f;
			break;

		case FS_SEL_2000:
			gyroScalingFactor = 2000.0f/32768.0f;
			break;

		default:
			break;
	}

}

//5- Get Sample Rate Divider
uint8_t MPU6050_Get_SMPRT_DIV(void)
{
	uint8_t Buffer = 0;

	I2C_Read(SMPLRT_DIV_REG, &Buffer, 1);
	return Buffer;
}

//6- Set Sample Rate Divider
void MPU6050_Set_SMPRT_DIV(uint8_t SMPRTvalue)
{
	I2C_Write8(SMPLRT_DIV_REG, SMPRTvalue);
}


//7- Get Accel Raw Data
void MPU6050_Get_Accel_RawData(RawData_Def *rawDef)
{
	uint8_t i2cBuf[2];
	uint8_t AcceArr[6], GyroArr[6];

	I2C_Read(INT_STATUS_REG, &i2cBuf[1],1);
	if((i2cBuf[1] && 0x01)) //test if data is ready
	{
		I2C_Read(ACCEL_XOUT_H_REG, AcceArr,6);

		//Accel Raw Data
		rawDef->x = ((AcceArr[0]<<8) | AcceArr[1]); // x-Axis
		rawDef->y = ((AcceArr[2]<<8) | AcceArr[3]); // y-Axis
		rawDef->z = ((AcceArr[4]<<8) | AcceArr[5]); // z-Axis
		//Gyro Raw Data
		I2C_Read(GYRO_XOUT_H_REG, GyroArr,6);
		GyroRW[0] = ((GyroArr[0]<<8) | GyroArr[1]);
		GyroRW[1] = (GyroArr[2]<<8) | GyroArr[3];
		GyroRW[2] = ((GyroArr[4]<<8) | GyroArr[5]);
	}
}

//8- Get Accel scaled data (g unit of gravity, 1g = 9.81m/s2)
void MPU6050_Get_Accel_Scale(ScaledData_Def *scaledDef)
{

	RawData_Def AccelRData;
	MPU6050_Get_Accel_RawData(&AccelRData);

	//Accel Scale data
	scaledDef->x = ((AccelRData.x+0.0f)*accelScalingFactor);
	scaledDef->y = ((AccelRData.y+0.0f)*accelScalingFactor);
	scaledDef->z = ((AccelRData.z+0.0f)*accelScalingFactor);
}



//9- Get Gyro Raw Data
void MPU6050_Get_Gyro_RawData(RawData_Def *rawDef)
{

	//Accel Raw Data
	rawDef->x = GyroRW[0];
	rawDef->y = GyroRW[1];
	rawDef->z = GyroRW[2];

}

//10- Get Gyro scaled data
void MPU6050_Get_Gyro_Scale(ScaledData_Def *scaledDef)
{
	RawData_Def myGyroRaw;
	MPU6050_Get_Gyro_RawData(&myGyroRaw);

	//Gyro Scale data
	scaledDef->x = (myGyroRaw.x)*gyroScalingFactor; // x-Axis
	scaledDef->y = (myGyroRaw.y)*gyroScalingFactor; // y-Axis
	scaledDef->z = (myGyroRaw.z)*gyroScalingFactor; // z-Axis
}

//10- Get Euler's angles data
void MPU6050_Get_Euler_Angle(EulerAngles_Def *AnglesDef,ScaledData_Def *scaledDef){
	auto float Ax = scaledDef->x*9.8;
	auto float Ay = scaledDef->y*9.8;
	auto float Az = scaledDef->z*9.8;
	AnglesDef->roll = 180 * atan2(Ay, sqrt(Ax*Ax + Az*Az))/PI;
	AnglesDef->pitch = 180 * atan2(Ax, sqrt(Ay*Ay + Az*Az))/PI;;
}


