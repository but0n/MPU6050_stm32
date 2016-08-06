#include <math.h>
#include "stm32f10x.h"
#include "bit.h"
#include "i2c.h"
#include "mpu6050.h"



#define Kp      100.0f      //比例增益支配率(常量)
#define Ki      0x002f      //积分增益支配率
#define halfT   0x001f      //采样周期的一半

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;   //Quaternion
float exInt = 0, eyInt = 0, ezInt = 0;
float Yaw, Pitch, Roll;     //eular

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


void Comput(float gx, float gy, float gz, float ax, float ay, float az) {

    float norm;     //模
    float vx, vy, vz;
    float ex, ey, ez;

    norm = sqrt(ax*ax + ay*ay + az*az);     //取模
    showData((short)norm);
    sendData_uart(0x0D);
    sendData_uart(0x0A);
    //向量化
    ax = ax / norm;
    ay = ay / norm;
    az = az / norm;

    //估计方向的重力
    vx = 2 * (q1 * q3 - q0 * q2);
    vy = 2 * (q0 * q1 + q2 * q3);
    vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    //错误的领域和方向传感器测量参考方向几件的交叉乘积的总和
    ex = (ay * vz - az * vy);
    ey = (az * vx - ax * vz);
    ez = (ax * vy - ay * vx);

    //积分误差比例积分增益
    exInt += ex * Ki;
    eyInt += ey * Ki;
    ezInt += ez * Ki;

    //调整后的陀螺仪测量
    gx += Kp * ex + exInt;
    gy += Kp * ey + exInt;
    gz += Kp * ez + exInt;

    //整合四元数率和正常化
    q0 += (-q1 * gx - q2 * gy - q3 * gz) * halfT;
    q1 += (q0 * gx + q2 * gz - q3 * gy) * halfT;
    q2 += (q0 * gy - q1 * gz + q3 * gx) * halfT;
    q3 += (q0 * gz + q1 * gy - q2 * gx) * halfT;

    //正常化四元
    norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 = q0 / norm;
    q1 = q1 / norm;
    q2 = q2 / norm;
    q3 = q3 / norm;

    Pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) * 57.3;
    Roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1*q1 - 2 * q2*q2 + 1) * 57.3;
}



int main() {
    initLED();
    initUART(72, 115200);

    MPU_init();

    while(1) {

        data_TypeDef Gyro;
        data_TypeDef Accel;

        MPU6050_getStructData(&Gyro, GYRO_XOUT_H, 16.4f);
        MPU6050_getStructData(&Accel, ACCEL_XOUT_H, 16384);
        Comput(Gyro.x, Gyro.y, Gyro.z, Accel.x, Accel.y, Accel.z);

        sendData_uart('P');
        sendData_uart('i');
        sendData_uart('t');
        sendData_uart('c');
        sendData_uart('h');
        sendData_uart(':');
        sendData_uart(' ');

        showData(Pitch);
        sendData_uart(' ');

        sendData_uart('R');
        sendData_uart('o');
        sendData_uart('l');
        sendData_uart('l');
        sendData_uart(':');
        sendData_uart(' ');

        showData(Roll);

        sendData_uart(' ');
        showData(Gyro.x);
        sendData_uart(',');
        showData(q1);

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
