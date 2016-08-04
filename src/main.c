#include "stm32f10x.h"
#include "dmp.h"
#include "bit.h"
#include "i2c.h"

//=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_
#define GPIOA_ODR_Addr (GPIOA_BASE + 12)

#define LED0 BIT_ADDR((GPIOA_BASE+12), 8)
#define LED1 BIT_ADDR((GPIOD_BASE+12), 2)
//=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_


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

void initLED() {
    RCC->APB2ENR |= 1<<5;
    GPIOD->CRL &= 0xFFFFF0FF;
    GPIOD->CRL |= 0x00000300;
}
//ms
void delay(volatile unsigned int count) {
    for(count *= 12000; count!=0; count--);
}

void MPU_Sigle_Write(unsigned char reg_addr, unsigned char reg_data) {
    IIC_Start();
    IIC_Send(MPU6050_ADDR);
    if(!IIC_Wait_Ack()) LED1 = 1;

    IIC_Send(reg_addr);
    IIC_Wait_Ack();

    IIC_Send(reg_data);
    IIC_Wait_Ack();

    IIC_Stop();

}
unsigned char MPU_Sigle_Read(unsigned reg_addr) {
    unsigned char reg_data;
    IIC_Start();
    IIC_Send(MPU6050_ADDR);
    IIC_Wait_Ack();

    IIC_Send(reg_addr);
    IIC_Wait_Ack();

    IIC_Start();
    IIC_Send(MPU6050_ADDR+1);
    IIC_Wait_Ack();

    reg_data = IIC_Read();
    IIC_Ack(1);
    IIC_Stop();
    return reg_data;
}
short MPU_GetData(unsigned char REG_Addr) {
    unsigned char H, L;
    H = MPU_Sigle_Read(REG_Addr);
    L = MPU_Sigle_Read(REG_Addr+1);
    return (short)(H<<8)+L;
}
void MPU_init() {
    IIC_init();
    delay(500);
    MPU_Sigle_Write(PWR_MGMT_1, 0x00);
    MPU_Sigle_Write(SMPLRT_DIV, 0x07);
    MPU_Sigle_Write(CONFIG, 0x06);
    MPU_Sigle_Write(GYRO_CONFIG, 0x18);
    MPU_Sigle_Write(ACCEL_CONFIG, 0x01);

    LED1 = MPU_Sigle_Read(WHO_AM_I) == 0x68?0:1;
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

void MPU6050_getStructData(data_TypeDef *cache, unsigned char reg_addr) {
    cache->x = MPU_GetData(reg_addr)/16.4;
    cache->y = MPU_GetData(reg_addr + 2)/16.4;
    cache->z = MPU_GetData(reg_addr + 4)/16.4;
}

int main() {
    initLED();
    initUART(72, 115200);

    MPU_init();


    dmp();
    while(1) {
        //GYRO_x = MPU_GetData(ACCEL_XOUT_H);
        //GYRO_x /= 16.4;
        //x += GYRO_x;

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
