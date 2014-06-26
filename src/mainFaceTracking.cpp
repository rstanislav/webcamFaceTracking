/*
 *	@File Description:
 *		This is the main file for the face tracking program.
 *
 *	@brief:
 *		This project tries to implement a face tracking system. A Logitech C910
 *		webcam with virtual PTZ control is used. It first detects the face and "move"
 *		the face to the center wiht PID control and zoom as large as possible.
 *
 *	@Environment and Tools used
 *		OS: 		Ubuntu 14.04 64 bits
 *		Tools:		OpenCV 2.4.9
 *					uvcdynctrl command
 *
 *	@NOTE:
 *		The code is free to use, and anyone want to discuss the code with me can
 *		email me at
 *
 *			mint.wzy@gmail.com
 *
 *		Although it's still quite simple, a demo of the code is available on Youtube with
 *		the following link:
 *
 */
#include "faceTracking.h"

int main(){
	faceTracking();
	return 0;
}
