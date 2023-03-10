/*
 * mpu6050.h
 *
 *  Created on: Aug 3, 2022
 *      Author: Soulaimane OuladBelayachi
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

//Header Files
#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdbool.h>
#include <math.h>

#define PI 3.142857

//MPU6050 Registers
#define WHO_AM_I_REG			0x75
#define MPU_ADDR				0x68
#define PWR_MAGT_1_REG			0x6B
#define CONFIG_REG				0x1A
#define GYRO_CONFIG_REG			0x1B
#define ACCEL_CONFIG_REG		0x1C
#define SMPLRT_DIV_REG			0x19
#define INT_STATUS_REG			0x3A
#define ACCEL_XOUT_H_REG		0x3B
#define TEMP_OUT_H_REG			0x41
#define GYRO_XOUT_H_REG			0x43
#define FIFO_EN_REG 			0x23
#define INT_ENABLE_REG 			0x38
#define I2CMACO_REG 			0x23
#define USER_CNT_REG			0x6A
#define FIFO_COUNTH_REG 		0x72
#define FIFO_R_W_REG 			0x74

//TypeDefs and Enums

//1- MPU Configuration
typedef struct
{
	uint8_t ClockSource;
	uint8_t Gyro_Full_Scale;
	uint8_t Accel_Full_Scale;
	uint8_t CONFIG_DLPF;
	bool 		Sleep_Mode_Bit;

}MPU_ConfigTypeDef;
//2- Clock Source ENUM
enum PM_CLKSEL_ENUM
{
	Internal_8MHz 		= 0x00,
	X_Axis_Ref			= 0x01,
	Y_Axis_Ref			= 0x02,
	Z_Axis_Ref			= 0x03,
	Ext_32_768KHz		= 0x04,
	Ext_19_2MHz			= 0x05,
	TIM_GENT_INREST		= 0x07
};
//3- Gyro Full Scale Range ENUM (deg/sec)
enum gyro_FullScale_ENUM
{
	FS_SEL_250 	= 0x00,
	FS_SEL_500 	= 0x01,
	FS_SEL_1000 = 0x02,
	FS_SEL_2000	= 0x03
};
//4- Accelerometer Full Scale Range ENUM (1g = 9.81m/s2)
enum accel_FullScale_ENUM
{
	AFS_SEL_2g	= 0x00,
	AFS_SEL_4g,
	AFS_SEL_8g,
	AFS_SEL_16g
};
//5- Digital Low Pass Filter ENUM
enum DLPF_CFG_ENUM
{
	DLPF_260A_256G_Hz = 0x00,
	DLPF_184A_188G_Hz = 0x01,
	DLPF_94A_98G_Hz 	= 0x02,
	DLPF_44A_42G_Hz 	= 0x03,
	DLPF_21A_20G_Hz 	= 0x04,
	DLPF_10_Hz 				= 0x05,
	DLPF_5_Hz 				= 0x06
};
//6- e external Frame Synchronization ENUM
enum EXT_SYNC_SET_ENUM
{
	input_Disable = 0x00,
	TEMP_OUT_L		= 0x01,
	GYRO_XOUT_L		= 0x02,
	GYRO_YOUT_L		= 0x03,
	GYRO_ZOUT_L		= 0x04,
	ACCEL_XOUT_L	= 0x05,
	ACCEL_YOUT_L	= 0x06,
	ACCEL_ZOUT_L	= 0x07
};

//7. Raw data typedef
typedef struct
{
	int16_t x;
	int16_t y;
	int16_t z;
}RawData_Def;

//8. Scaled data typedef
typedef struct
{
	float x;
	float y;
	float z;
}ScaledData_Def;

//9.Euler Angles typedef
typedef struct
{
	float roll;
	float pitch;
	float yaw;
}EulerAngles_Def;






//Function Prototype
//1- i2c Handler
void MPU6050_Init(I2C_HandleTypeDef *I2Chnd);
//2- i2c Read
void I2C_Read(uint8_t ADDR, uint8_t *i2cBuf, uint8_t NofData);
//3- i2c Write 8 Bit
void I2C_Write8(uint8_t ADDR, uint8_t data);
//4- MPU6050 Initialization Configuration
void MPU6050_Config(MPU_ConfigTypeDef *config);
//5- Get Sample Rate Divider
uint8_t MPU6050_Get_SMPRT_DIV(void);
//6- Set Sample Rate Divider
void MPU6050_Set_SMPRT_DIV(uint8_t SMPRTvalue);
//7- External Frame Sync.
void MPU6050_Get_Accel_RawData(RawData_Def *rawDef);
//8- Get Accel scaled data
void MPU6050_Get_Accel_Scale(ScaledData_Def *scaledDef);
//9- Get Gyro Raw Data
void MPU6050_Get_Gyro_RawData(RawData_Def *rawDef);
//10- Get Gyro scaled data
void MPU6050_Get_Gyro_Scale(ScaledData_Def *scaledDef);
//10- Get Euler's angles data
void MPU6050_Get_Euler_Angle(EulerAngles_Def *AnglesDef,ScaledData_Def *scaledDef);



#endif /* INC_MPU6050_H_ */
