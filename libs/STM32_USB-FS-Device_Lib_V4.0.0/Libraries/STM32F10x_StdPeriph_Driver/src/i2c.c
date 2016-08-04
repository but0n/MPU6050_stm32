#include "i2c.h"

void IIC_init() {
    RCC->APB2ENR |= 1<<3;
    GPIOB->CRH &= 0x0F0FFFFF;
    GPIOB->CRH |= 0x70700000;
    GPIOB->ODR |= 5<<5;

    RCC->APB2ENR |= 1<<4;
    GPIOB->CRL &= 0xF0FFFFFF;
    GPIOB->CRL |= 0x03000000;
    AD0 = 0;
}
void IIC_Start() {
    SDA = 1;
    SCL = 1;
    IIC_DELAY();
    SDA = 0;
    IIC_DELAY();
    SCL = 0;
}
void IIC_Stop() {
    SCL = 0;
    IIC_DELAY();
    SDA = 0;
    IIC_DELAY();
    SCL = 1;
    IIC_DELAY();
    SDA = 1;
    IIC_DELAY();
}
void IIC_Ack(unsigned char ack) {

    SCL = 0;
    IIC_DELAY();
    SDA = ack;
    IIC_DELAY();
    SCL = 1;
    IIC_DELAY();
    SCL = 0;
    IIC_DELAY();
}
unsigned char IIC_Wait_Ack() {

    SCL = 0;
    IIC_DELAY();
    SDA = 1;
    IIC_DELAY();
    SCL = 1;
    IIC_DELAY();
    if(READ_SDA) {
        SCL = 0;
        IIC_DELAY();
        return 0;
    }

    SCL = 0;
    return 1;

}

void IIC_Send(unsigned char dat) {
    unsigned char i;
    SCL = 0;
    for(i = 0; i < 8; i++) {
        IIC_DELAY();
        SDA = (dat&0x80)>>7;
        dat <<= 1;
        IIC_DELAY();
        SCL = 1;
        IIC_DELAY();
        SCL = 0;
    }
}

unsigned char IIC_Read() {
    unsigned char i, dat = 0;
    SDA = 1;
    for(i = 0; i < 8; i++) {
        SCL = 0;
        IIC_DELAY();
        SCL = 1;
        IIC_DELAY();
        dat<<=1;
        dat |= READ_SDA;
    }
    SCL = 0;
    return dat;
}
