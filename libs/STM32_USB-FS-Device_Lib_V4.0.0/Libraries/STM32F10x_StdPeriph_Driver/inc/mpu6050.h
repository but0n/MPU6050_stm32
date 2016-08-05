#pragma once
#ifndef _MPU6050_
#define _MPU6050_

#include "i2c.h"

//=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_

#define MPU6050_ADDR 0xD0 //    0x68 >> 1

#define SMPLRT_DIV      0x19
#define CONFIG 0x1A
#define GYRO_CONFIG     0x1B
#define ACCEL_CONFIG    0x1C

#define ACCEL_XOUT_H    0x3B
#define ACCEL_XOUT_L    0x3C
#define ACCEL_YOUT_H    0x3D
#define ACCEL_YOUT_L    0x3E
#define ACCEL_ZOUT_H    0x3F
#define ACCEL_ZOUT_L    0x40

#define TEMP_OUT_H      0x41
#define TEMP_OUT_L      0x42

#define GYRO_XOUT_H     0x43
#define GYRO_XOUT_L     0x44
#define GYRO_YOUT_H     0x45
#define GYRO_YOUT_L     0x46
#define GYRO_ZOUT_H     0x47
#define GYRO_ZOUT_L     0x48

#define PWR_MGMT_1      0x6B
#define WHO_AM_I        0x75

//=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_


typedef struct {
    short x;
    short y;
    short z;
}data_TypeDef;

void MPU_Sigle_Write(unsigned char reg_addr, unsigned char reg_data);
unsigned char MPU_Sigle_Read(unsigned reg_addr);
short MPU_GetData(unsigned char REG_Addr);
void MPU_init();
void initUART(unsigned int pclk2, unsigned int bound);
void sendData_uart(unsigned char data);
void showData(short k);
void MPU6050_getStructData(data_TypeDef *cache, unsigned char reg_addr);

#endif
