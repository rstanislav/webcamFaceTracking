#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include "PTZ.h"
#include "config.h"

/*
 * @brief:
 * 		This is the PID implementaion of the system.
 * 		Since the resolution is 640*480, center (320, 240) is choose as the setpoint.
 */
int PID(cv::Point &pt1, cv::Point &pt2, double &hOut, double &vOut){
	 int hCenter, vCenter;
	double timeDiff;
	double hInputLast = 320, vInputLast = 240, hInputDiff, vInputDiff, hITerm = 0, vITerm = 0;
	double Kp = KP, Ki = KI, Kd = KD, hError = 0, vError = 0;
	struct timespec tstart={0,0}, tend={0,0};

	clock_gettime(CLOCK_MONOTONIC, &tend);
	timeDiff = (((double)tend.tv_sec + 1.0e-9*tend.tv_nsec) -
		   ((double)tstart.tv_sec + 1.0e-9*tstart.tv_nsec));
#if	DEBUG
	printf("timeDiff = %lf\n", timeDiff);
#endif
	//get input value
	hCenter = (pt1.x + pt2.x)/2;
	vCenter = (pt1.y + pt2.y)/2;
	//P
	hError = 320 - hCenter;
	vError = 240 - vCenter;
	//I
	hITerm += (Ki*hError*timeDiff);
	vITerm += (Ki*vError*timeDiff);
	//D
	hInputDiff = (hCenter - hInputLast)/timeDiff;
	vInputDiff = (vCenter - vInputLast)/timeDiff;
	//calculate  PID output
	hOut = Kp*hError + hITerm - Kd*hInputDiff;
	vOut = Kp*vError + vITerm - Kd*vInputDiff;
#if DEBUG
	printf("Kp*hError = %f, -Kd*hInputDiff = %f\n", Kp*hError, -Kd*hInputDiff);
#endif
	//remember
	hInputLast = hCenter;
	vInputLast = vCenter;
	tstart = tend;
	return timeDiff;
}

/*
 * @brief: For uvcdynctrl control, different pixel is changed based on different value.
 * 			The following value is measured approximately, and is subject to change
 *
 * 			Zoom 1: (5, 4)
 * 			Zoom 2: (42, 31)
 * 			Zoom 3: (70, 60)
 */
void manualTracking(char key, int keyZoomNum, int keyPanNum, int keyTiltNum){
	switch(key){
		case 'j':
			keyZoomNum++;
			break;
		case 'k':
			keyZoomNum--;
			break;
		case 'h':
			keyPanNum += STEP;
			break;
		case 'l':
			keyPanNum -= STEP;
			break;
		case 'u':
			keyTiltNum += STEP;
			break;
		case 'n':
			keyTiltNum -= STEP;
			break;
	}
	updatePTZ(keyZoomNum, keyPanNum, keyTiltNum);
}

/*
 * @brief:
 * 		system() command is called to carry shell command.
 */
void updatePTZ(int keyZoomNum, int keyPanNum, int keyTiltNum){
	char command[70];

	sprintf(command, "uvcdynctrl -d video0 -s 'Zoom, Absolute' -- %d\n", keyZoomNum);
	system(command);
	sprintf(command, "uvcdynctrl -d video0 -s 'Pan (Absolute)' -- %d\n", keyPanNum);
	system(command);
	sprintf(command, "uvcdynctrl -d video0 -s 'Tilt (Absolute)' -- %d\n", keyTiltNum);
	system(command);
}

