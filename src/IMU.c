/*
 * IMU.c
 *
 *  Created on: Mar 12, 2021
 *      Author: Mohamed
 */

#include		<stdio.h>
#include		<string.h>
#include 		"stm32f446xx_driver.h"
#include 		"MPU6050.h"
#include 		"MadgwickAHRS.h"

extern void initialise_monitor_handles();

/*
 * I2C1
 * PB6 ->  SCL
 * PB7 ->  SDA

 */

#define MY_ADDR 0x61
#define SLAVE_ADDR  0x68


I2C_Handle_t I2C1Handle;

//rcv buffer
uint8_t rcv_buf[32];


RawData_Def myaccelraw, mygyroraw;
ScaledData_Def myaccelscaled, mygyroscaled;
int16_t x_accel_raw, y_accel_raw, z_accel_raw;
float x_gyro_raw, y_gyro_raw, z_gyro_raw;
float x_accel_scaled, y_accel_scaled, z_accel_scaled;
float x_gyro_scaled, y_gyro_scaled, z_gyro_scaled;
float final_roll = 0;
float final_pitch = 0;
float final_yaw = 0;
char data[100];
float t1 ,t2,t;
float gx,gy,gz;

int main(void)
{
	initialise_monitor_handles();
	MPU_ConfigTypeDef mympuconfig;
	GPIO_ButtonInit();
	GPIOLED_Init();
	//i2c peripheral configuration
	printf("app is ready ");
	I2C1Handle=I2C_Inits(I2C1,I2C_ACK_ENABLE,MY_ADDR,I2C_FM_DUTY_2,I2C_SCL_SPEED_SM);

	//enable the i2c peripheral
	I2C_PeripheralControl(I2C1,Enable);

	//ack bit is made 1 after PE=1
	I2C_ManageAcking(I2C1,I2C_ACK_ENABLE);

	//1: Initializing MPU6050 and I2C modules.
	MPU6050_Init(&I2C1Handle);
	//2: MPU6050(gyro and accelerometer parameters) configuration.
	mympuconfig.Accel_Full_Scale = AFS_SEL_4g;
	mympuconfig.Gyro_Full_Scale = FS_SEL_500;
	mympuconfig.ClockSource = Internal_8MHz;
	mympuconfig.CONFIG_DLPF = DLPF_260A_256G_Hz;
	mympuconfig.Sleep_Mode_Bit = 0;  // 0 is normal mode while 1 is sleep mode
	MPU6050_Config(I2C1Handle,&mympuconfig);

	while(1)
	{

		MPU6050_Get_Accel_RawData(I2C1Handle,&myaccelraw);
		MPU6050_Get_Gyro_RawData(&mygyroraw);
		x_accel_raw = myaccelraw.x;
		y_accel_raw = myaccelraw.y;
		z_accel_raw = myaccelraw.z;
		x_gyro_raw = mygyroraw.x;
		y_gyro_raw = mygyroraw.y;
		z_gyro_raw = mygyroraw.z;
		//Scaled Data
		MPU6050_Get_Accel_Scale(I2C1Handle,&myaccelscaled);
		MPU6050_Get_Gyro_Scale(&mygyroscaled);
		x_accel_scaled = ((myaccelscaled.x/1000.0f)-0.0233996511)/(-1.10105348);
		y_accel_scaled = ((myaccelscaled.y/1000.0f)+0.0261853039) / (-1.10141468);
		z_accel_scaled = ((myaccelscaled.z/1000.0f)-0.0897204578) / (-1.10337627);
		x_gyro_scaled = mygyroscaled.x;
		y_gyro_scaled = mygyroscaled.y;
		z_gyro_scaled = mygyroscaled.z;
		// AHRS UPDATE
		MadgwickAHRSupdateIMU(x_gyro_scaled, y_gyro_scaled, z_gyro_scaled, x_accel_scaled, y_accel_scaled, z_accel_scaled);
		final_roll = roll;
		final_pitch = pitch;
		final_yaw = yaw;
		printf("the yaw value is %d",yaw);
	}


}
