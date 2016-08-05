#include "mpu6050.h"

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
