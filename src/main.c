#include "stm32f10x.h"
#include "bit.h"
#include "i2c.h"
#include "mpu6050.h"

#include "Euler.h"


//ms
void delay(volatile unsigned int count) {
    for(count *= 12000; count!=0; count--);
}

void initUART(unsigned int pclk2, unsigned int bound) {
    float temp;
    unsigned short mantissa;
    unsigned short fraction;
    temp = (float)(pclk2*1000000)/(bound*16);
    mantissa = temp;
    fraction = (temp - mantissa) * 16;
    mantissa <<= 4;
    mantissa += fraction;
    RCC->APB2ENR |= 1<<2;
    RCC->APB2ENR |= 1<<14;

    GPIOA->CRH &= 0xFFFFF00F;
    GPIOA->CRH |= 0x000008B0;

    RCC->APB2RSTR |= 1<<14;
    RCC->APB2RSTR &= ~(1<<14);

    USART1->BRR = mantissa;
    USART1->CR1 |= 0x200C;

    USART1->CR1 |= 1<<8;
    USART1->CR1 |= 1<<5;
}
void sendData_uart(unsigned char data) {
    USART1->DR = data;
    while((USART1->SR & 0x40) == 0);
}
void showData(short k) {
    unsigned char a, b, c, d, e;

    sendData_uart(k<0?'-':'+');
    if(k<0) k=-k;
    e = (unsigned char)(k % 10);
    d = (unsigned char)(k/10) % 10;
    c = (unsigned char)(k/100) % 10;
    b = (unsigned char)(k/1000) % 10;
    a = (unsigned char)(k/10000);

    sendData_uart(a+0x30);
    sendData_uart(b+0x30);
    sendData_uart(c+0x30);
    sendData_uart(d+0x30);
    sendData_uart(e+0x30);
    sendData_uart('.');
}


int main() {
    initLED();
    initUART(72, 115200);

    MPU_init();

    Comput(0,0,0,0,0,0.4);

    while(1) {

        data_TypeDef Gyro;
        data_TypeDef Accel;

        MPU6050_getStructData(&Gyro, GYRO_XOUT_H);
        //while(1);

        sendData_uart('X');
        sendData_uart(':');
        showData(Gyro.x);
        sendData_uart(' ');

        sendData_uart('Y');
        sendData_uart(':');
        showData(Gyro.y);
        sendData_uart(' ');

        sendData_uart('Z');
        sendData_uart(':');
        showData(Gyro.z);
        sendData_uart(' ');

        MPU6050_getStructData(&Accel, ACCEL_XOUT_H);

        sendData_uart('X');
        sendData_uart(':');
        showData(Accel.x);
        sendData_uart(' ');

        sendData_uart('Y');
        sendData_uart(':');
        showData(Accel.y);
        sendData_uart(' ');

        sendData_uart('Z');
        sendData_uart(':');
        showData(Accel.z);
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
