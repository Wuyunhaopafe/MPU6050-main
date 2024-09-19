#include "init.h"
#include "alldata.h"

volatile uint32_t SysTick_count; //系统时间计数
_st_Mpu MPU6050;   //MPU6050原始数据
_st_AngE Angle;    //当前角度姿态值
_st_Remote Remote; //遥控通道值

_st_ALL_flag ALL_flag; //系统标志位，包含解锁标志位等



