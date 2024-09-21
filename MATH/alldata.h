#ifndef __ALLDATA_H
#define __ALLDATA_H

typedef   signed          char int8_t;
typedef   signed short     int int16_t;
typedef   signed           int int32_t;
typedef   signed       long long int64_t;

/* exact-width unsigned integer types */
typedef unsigned          char uint8_t;
typedef unsigned short     int uint16_t;
typedef unsigned           int uint32_t;
typedef unsigned       long long uint64_t;

//#define NULL 0
extern volatile uint32_t SysTick_count;

typedef struct{
	int16_t accX;
	int16_t accY;
	int16_t accZ;
	int16_t gyroX;
	int16_t gyroY;
	int16_t gyroZ;
}_st_Mpu;

typedef struct{
	float roll;
	float pitch;
	float yaw;
}_st_AngE;

typedef struct
{
	uint16_t roll;
	uint16_t pitch;
	uint16_t thr;
	uint16_t yaw;
	uint16_t AUX1;
	uint16_t AUX2;
	uint16_t AUX3;
	uint16_t AUX4;	
}_st_Remote;


typedef volatile struct
{
	uint8_t unlock;
	

}_st_ALL_flag;

extern _st_Remote Remote;
extern _st_Mpu MPU6050;
extern _st_AngE Angle;

extern _st_ALL_flag ALL_flag;


#endif

