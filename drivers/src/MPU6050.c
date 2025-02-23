/*
 * MPU6050.c
 *
 *  Created on: Mar 12, 2021
 *      Author: Mohamed
 */


#ifndef INC_MPU6050_C_
#define INC_MPU6050_C_

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include 		"MPU6050.h"
#include 		"stm32f446xx_driver.h"

//Library Variable
//1- I2C Handle
static I2C_Handle_t i2cHandler;
//2- Accel & Gyro Scaling Factor
static float accelScalingFactor, gyroScalingFactor;
//3- Bias varaibles
static float A_X_Bias = 0.0f;
static float A_Y_Bias = 0.0f;
static float A_Z_Bias = 0.0f;

static int16_t GyroRW[3];
static void delay(void);

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}

//Fucntion Definitions
//1- i2c Handler
void MPU6050_Init(I2C_Handle_t *I2Chnd)
{
	//Copy I2C CubeMX handle to local library
	memcpy(&i2cHandler, I2Chnd, sizeof(*I2Chnd));
}

//2- i2c Read
void I2C_Read(I2C_Handle_t I2C1Handle,uint8_t ADDR, uint8_t *i2cBif, uint8_t NofData)
{

	I2C_MasterReceiveData(&I2C1Handle,i2cBif,sizeof(i2cBif),ADDR,I2C_DISABLE_SR);
}

//3- i2c Write
void I2C_Write8(I2C_Handle_t I2C1Handle,uint8_t ADDR, uint8_t data)
{

	I2C_MasterSendData(&I2C1Handle,&data,sizeof(data),ADDR,I2C_DISABLE_SR);

}

//4- MPU6050 Initialaztion Configuration
void MPU6050_Config(I2C_Handle_t I2C1Handle,MPU_ConfigTypeDef *config)
{
	uint8_t Buffer = 0;
	//Clock Source
	//Reset Device
	I2C_Write8(I2C1Handle,PWR_MAGT_1_REG, 0x80);
	delay();
	Buffer = config ->ClockSource & 0x07; //change the 7th bits of register
	Buffer |= (config ->Sleep_Mode_Bit << 6) &0x40; // change only the 7th bit in the register
	I2C_Write8(I2C1Handle,PWR_MAGT_1_REG, Buffer);
	delay(); // should wait 10ms after changeing the clock setting.

	//Set the Digital Low Pass Filter
	Buffer = 0;
	Buffer = config->CONFIG_DLPF & 0x07;
	I2C_Write8(I2C1Handle,CONFIG_REG, Buffer);

	//Select the Gyroscope Full Scale Range
	Buffer = 0;
	Buffer = (config->Gyro_Full_Scale << 3) & 0x18;
	I2C_Write8(I2C1Handle,GYRO_CONFIG_REG, Buffer);

	//Select the Accelerometer Full Scale Range
	Buffer = 0;
	Buffer = (config->Accel_Full_Scale << 3) & 0x18;
	I2C_Write8(I2C1Handle,ACCEL_CONFIG_REG, Buffer);
	//Set SRD To Default
	MPU6050_Set_SMPRT_DIV(I2C1Handle,0x04);


	//Accelerometer Scaling Factor, Set the Accelerometer and Gyroscope Scaling Factor
	switch (config->Accel_Full_Scale)
	{
		case AFS_SEL_2g:
			accelScalingFactor = (2000.0f/32768.0f);
			break;

		case AFS_SEL_4g:
			accelScalingFactor = (4000.0f/32768.0f);
				break;

		case AFS_SEL_8g:
			accelScalingFactor = (8000.0f/32768.0f);
			break;

		case AFS_SEL_16g:
			accelScalingFactor = (16000.0f/32768.0f);
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
uint8_t MPU6050_Get_SMPRT_DIV(I2C_Handle_t I2CHandle)
{
	uint8_t Buffer = 0;

	I2C_Read(I2CHandle,SMPLRT_DIV_REG, &Buffer, 1);
	return Buffer;
}

//6- Set Sample Rate Divider
void MPU6050_Set_SMPRT_DIV(I2C_Handle_t I2CHandle,uint8_t SMPRTvalue)
{
	I2C_Write8(I2CHandle,SMPLRT_DIV_REG, SMPRTvalue);
}

//7- Get External Frame Sync.
uint8_t MPU6050_Get_FSYNC(I2C_Handle_t I2CHandle)
{
	uint8_t Buffer = 0;

	I2C_Read(I2CHandle,CONFIG_REG, &Buffer, 1);
	Buffer &= 0x38;
	return (Buffer>>3);
}

//8- Set External Frame Sync.
void MPU6050_Set_FSYNC(I2C_Handle_t I2CHandle,enum EXT_SYNC_SET_ENUM ext_Sync)
{
	uint8_t Buffer = 0;
	I2C_Read(I2CHandle,CONFIG_REG, &Buffer,1);
	Buffer &= ~0x38;

	Buffer |= (ext_Sync <<3);
	I2C_Write8(I2CHandle,CONFIG_REG, Buffer);

}

//9- Get Accel Raw Data
void MPU6050_Get_Accel_RawData(I2C_Handle_t I2CHandle,RawData_Def *rawDef)
{
	uint8_t i2cBuf[2];
	uint8_t AcceArr[6], GyroArr[6];

	I2C_Read(I2CHandle,INT_STATUS_REG, &i2cBuf[1],1);
	if((i2cBuf[1]&&0x01))
	{
		I2C_Read(I2CHandle,ACCEL_XOUT_H_REG, AcceArr,6);

		//Accel Raw Data
		rawDef->x = ((AcceArr[0]<<8) + AcceArr[1]); // x-Axis
		rawDef->y = ((AcceArr[2]<<8) + AcceArr[3]); // y-Axis
		rawDef->z = ((AcceArr[4]<<8) + AcceArr[5]); // z-Axis
		//Gyro Raw Data
		I2C_Read(I2CHandle,GYRO_XOUT_H_REG, GyroArr,6);
		GyroRW[0] = ((GyroArr[0]<<8) + GyroArr[1]);
		GyroRW[1] = (GyroArr[2]<<8) + GyroArr[3];
		GyroRW[2] = ((GyroArr[4]<<8) + GyroArr[5]);
	}
}

//10- Get Accel scaled data (g unit of gravity, 1g = 9.81m/s2)
void MPU6050_Get_Accel_Scale(I2C_Handle_t I2CHandle,ScaledData_Def *scaledDef)
{

	RawData_Def AccelRData;
	MPU6050_Get_Accel_RawData( I2CHandle,&AccelRData);

	//Accel Scale data
	scaledDef->x = ((AccelRData.x+0.0f)*accelScalingFactor);
	scaledDef->y = ((AccelRData.y+0.0f)*accelScalingFactor);
	scaledDef->z = ((AccelRData.z+0.0f)*accelScalingFactor);
}

//11- Get Accel calibrated data
void MPU6050_Get_Accel_Cali(I2C_Handle_t I2CHandle,ScaledData_Def *CaliDef)
{
	ScaledData_Def AccelScaled;
	MPU6050_Get_Accel_Scale(I2CHandle,&AccelScaled);

	//Accel Scale data
	CaliDef->x = (AccelScaled.x) - A_X_Bias; // x-Axis
	CaliDef->y = (AccelScaled.y) - A_Y_Bias;// y-Axis
	CaliDef->z = (AccelScaled.z) - A_Z_Bias;// z-Axis
}
//12- Get Gyro Raw Data
void MPU6050_Get_Gyro_RawData(RawData_Def *rawDef)
{

	//Accel Raw Data
	rawDef->x = GyroRW[0];
	rawDef->y = GyroRW[1];
	rawDef->z = GyroRW[2];

}

//13- Get Gyro scaled data
void MPU6050_Get_Gyro_Scale(ScaledData_Def *scaledDef)
{
	RawData_Def myGyroRaw;
	MPU6050_Get_Gyro_RawData(&myGyroRaw);

	//Gyro Scale data
	scaledDef->x = (myGyroRaw.x)*gyroScalingFactor; // x-Axis
	scaledDef->y = (myGyroRaw.y)*gyroScalingFactor; // y-Axis
	scaledDef->z = (myGyroRaw.z)*gyroScalingFactor; // z-Axis
}

//14- Accel Calibration
void _Accel_Cali(float x_min, float x_max, float y_min, float y_max, float z_min, float z_max)
{
	//1* X-Axis calibrate
	A_X_Bias		= (x_max + x_min)/2.0f;

	//2* Y-Axis calibrate
	A_Y_Bias		= (y_max + y_min)/2.0f;

	//3* Z-Axis calibrate
	A_Z_Bias		= (z_max + z_min)/2.0f;
}


#endif /* INC_MPU6050_C_ */
