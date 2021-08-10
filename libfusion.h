#ifndef _LIB_SENSOR_FUSION_H_
#define _LIB_SENSOR_FUSION_H_

#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#ifdef __cplusplus 
extern "C"{ 
#endif

#define TYPE_ACCELEROMETER 					1
#define TYPE_MAGNETIC_FIELD 				2
#define TYPE_ORIENTATION 					3
#define TYPE_GYROSCOPE						4
#define TYPE_LIGHT							5
#define TYPE_PRESSURE						6
#define TYPE_TEMPERATURE 					7
#define TYPE_PROXIMITY						8
#define TYPE_GRAVITY						9
#define TYPE_LINEAR_ACCELERATION			10
#define TYPE_ROTATION_VECTOR 				11
#define TYPE_RELATIVE_HUMIDITY 				12
#define TYPE_AMBIENT_TEMPERATURE 			13
#define TYPE_MAGNETIC_FIELD_UNCALIBRATED 	14
#define TYPE_GAME_ROTATION_VECTOR 			15
#define TYPE_GYROSCOPE_UNCALIBRATED			16
#define TYPE_SIGNIFICANT_MOTION 			17
#define TYPE_STEP_DETECTOR 					18
#define TYPE_STEP_COUNTER 					19
#define TYPE_GEOMAGNETIC_ROTATION_VECTOR 	20
#define TYPE_HEART_RATE 					21
#define TYPE_TILT_DETECTOR 					22
#define TYPE_WAKE_GESTURE 					23
#define TYPE_GLANCE_GESTURE 				24
#define TYPE_PICK_UP_GESTURE 				25
#define TYPE_WRIST_TILT_GESTURE 			26
#define TYPE_DEVICE_ORIENTATION				27
#define TYPE_POSE_6DOF						28
#define TYPE_STATIONARY_DETECT 				29
#define TYPE_MOTION_DETECT 					30
#define TYPE_HEART_BEAT 					31
#define TYPE_DYNAMIC_SENSOR_META			32
#define TYPE_LOW_LATENCY_OFF_BODY_DETECT 	34
#define TYPE_ACCELEROMETER_UNCALIBRATED		35
#define TYPE_HINGE_ANGLE 					36
#define TYPE_ALL							-1

#define BAD_VALUE      -1
#define NO_ERROR        0

typedef int status_t;

typedef struct _axis{
	float x;
	float y;
	float z;
}sf_axis;

typedef struct _rv{
	float i;
	float j;
	float k;
	float w;
}sf_rv;

typedef struct _matrix{
	float r[3][3];
}sf_matrix;

typedef struct _sf_data{
	uint8_t type;
	uint64_t timeStamp;
	union{
		sf_axis axis;
		sf_rv	rv; 
		sf_matrix matrix;
		float   raw[9];
	};
}sf_data;

void* fusion_init(void);
void  fusion_enable_sensor(void *h, int sensor_type,  bool enable);
bool  fusion_is_enabled(void *h, int sensor_type);
void  fusion_process(void*h, const sf_data* value);
void  fusion_get_data(void*, sf_data* value);
void  fusion_release(void *h);
void  fusion_dump(void *h);
#ifdef __cplusplus 
}
#endif

#endif

