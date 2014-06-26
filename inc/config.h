#ifndef CONFIG_H
#define CONFIG_H

//define whether in DEBUG mode to have more text output
#define DEBUG					0

//define the SIZE of a window to increase the ZOOM value
#define ZOOM_SIZE				190*190

 //define how many times to count(when cannot detect a face)
 //before changing to a lower zoom value
#define SENTIVITY_LEVEL2		10
#define SENTIVITY_LEVEL3		5

//define parameters for virtual PTZ
#define MAX_PTZ					36000
#define MIN_PTZ					-36000
#define STEP					3600

//define command type
#define FACE_MOVE_UP			1
#define FACE_MOVE_DOWN			2
#define FACE_MOVE_LEFT			3
#define FACE_MOVE_RIGHT			4

//define parameters for PID control
#define ZOOM1_PIXEL_PER_H		11
#define ZOOM1_H_ERROR_POSITIVE	20
#define ZOOM1_H_ERROR_NEGATIVE 	-20

#define ZOOM2_PIXEL_PER_H		42
#define ZOOM2_H_ERROR_POSITIVE	60
#define ZOOM2_H_ERROR_NEGATIVE 	-60
#define ZOOM2_PIXEL_PER_V		31
#define ZOOM2_V_ERROR_POSITIVE	30
#define ZOOM2_V_ERROR_NEGATIVE 	-30

#define ZOOM3_PIXEL_PER_H		70
#define ZOOM3_H_ERROR_POSITIVE	60
#define ZOOM3_H_ERROR_NEGATIVE 	-60
#define ZOOM3_PIXEL_PER_V		60
#define ZOOM3_V_ERROR_POSITIVE	40
#define ZOOM3_V_ERROR_NEGATIVE 	-40

//define PID tuning parameters
#define KP						0.7
#define KI						0
#define KD						600

#endif
