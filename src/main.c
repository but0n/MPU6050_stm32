#include "stm32f10x.h"
#include "dmp.h"
#include "bit.h"
#include "i2c.h"
#include "mpu6050.h"



//ms
void delay(volatile unsigned int count) {
    for(count *= 12000; count!=0; count--);
}

int main() {
    initLED();
    initUART(72, 115200);

    MPU_init();


    dmp();
    while(1) {

        data_TypeDef GYRO;
        data_TypeDef ACCEL;

        MPU6050_getStructData(&GYRO, GYRO_XOUT_H);
        while(1);

        sendData_uart('X');
        sendData_uart(':');
        showData(GYRO.x);
        sendData_uart(' ');

        sendData_uart('Y');
        sendData_uart(':');
        showData(GYRO.y);
        sendData_uart(' ');

        sendData_uart('Z');
        sendData_uart(':');
        showData(GYRO.z);
        sendData_uart(' ');

        MPU6050_getStructData(&ACCEL, ACCEL_XOUT_H);

        sendData_uart('X');
        sendData_uart(':');
        showData(ACCEL.x);
        sendData_uart(' ');

        sendData_uart('Y');
        sendData_uart(':');
        showData(ACCEL.y);
        sendData_uart(' ');

        sendData_uart('Z');
        sendData_uart(':');
        showData(ACCEL.z);
        sendData_uart(' ');

        sendData_uart(0x0D);
        sendData_uart(0x0A);
        delay(300);

        //short tem = MPU_GetData(TEMP_OUT_H);
        //tem = 35 + ((double) (tem + 13200)) / 200;
        //showData(tem);
        //sendData_uart('T');
        //sendData_uart(':');
        //sendData_uart(0x0D);
        //sendData_uart(0x0A);

    }
}
