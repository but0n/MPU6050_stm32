#include <math.h>
#include "Euler.h"

#define Kp      100.0f      //比例增益支配率(常量)
#define Ki      0x002f      //积分增益支配率
#define halfT   0x001f      //采样周期的一半

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;   //Quaternion
float exInt = 0, eyInt = 0, ezInt = 0;
float Yaw, Pitch, Roll;     //eular

void Comput(float gx, float gy, float gz, float ax, float ay, float az) {

    float norm;     //模
    float vx, vy, vz;
    float ex, ey, ez;

    norm = sqrt(ax*ax + ay*ay + az*az);     //取模
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
