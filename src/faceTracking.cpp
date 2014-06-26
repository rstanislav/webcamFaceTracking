#include <iostream>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>

#include "PTZ.h"
#include "config.h"
#include "faceTracking.h"

using namespace std;
using namespace cv;

void autoTrackingPID(double hOut, double vOut, int area, int faceNum,
		int &keyZoomNum, int &keyPanNum, int &keyTiltNum);
void detectAndDisplay(Mat frame, double &hOut, double &vOut, int &area, int &faceNum);

// Global variables
int keyPanNum = 0, keyTiltNum = 0, keyZoomNum = 1;
int faceCount2 = 0, faceCount3 = 0;

CascadeClassifier face_cascade;
string face_cascade_name = "Debug/haarcascade_frontalface_alt.xml";

/*
 *	@brief:
 *		faceTracking() controls the logic of the program.
 */
int faceTracking(void){
	char key, autoMode = 1, track = 1;
	int faceNum, area;
	double hOut, vOut;

	//open camera and check
	VideoCapture capture(0);
	if (!capture.isOpened()){
		cout << "camera is not opened" << endl;
		return -1;
	}

    // Load the cascade
    if (!face_cascade.load(face_cascade_name)){
        cout << "--(!)Error loading the xml file\n";
        return (-1);
    };

    //begin reading the frame
    Mat currentMat;
    //initilize the PTZ value for the camera
    if(track)
    	updatePTZ(keyZoomNum, keyPanNum, keyTiltNum);
    //adjust camera first
	while(1){
		capture >> currentMat;
		key = waitKey(30);
		if(track){
//-------------	AUTO TRACKING ALGORITHM BELOW THIS LINE---------------------
			if(autoMode)
				autoTrackingPID(hOut, vOut, area, faceNum,
						keyZoomNum, keyPanNum, keyTiltNum);
			else
				manualTracking(key, keyZoomNum, keyPanNum, keyTiltNum);
//-------------AUTO TRACKING ALGOEITHMS ABOVE THIS LINE--------------------
		}
		if(key == 'q'){
			return 0;
		}else if(key == 'm'){
			autoMode = 1 - autoMode;
		}else if(key == 't'){
			track = 1 - track;
		}
		//camera adjustment done
		capture >> currentMat;
		detectAndDisplay(currentMat, hOut, vOut, area, faceNum);
	}
	return 0;
}

/*
 * @brief:
 * 		This function is called when zoomValue decreases, which means the face cannot
 * 		be detected at a large zoom value, so the camera is moved based on previous
 * 		command to correct for it
 */
void recorrection(int commandValue, int &keyPanNum, int &keyTiltNum){
	if(commandValue == FACE_MOVE_LEFT){
		keyPanNum -= 2*STEP;
		if(keyPanNum < MIN_PTZ)
			keyPanNum = MIN_PTZ;
	}
	else if(commandValue == FACE_MOVE_RIGHT){
		keyPanNum += 2*STEP;
		if(keyPanNum > MAX_PTZ)
			keyPanNum = MAX_PTZ;
	}
	else if(commandValue == FACE_MOVE_UP){
		keyTiltNum += STEP;
		if(keyTiltNum > MAX_PTZ)
			keyTiltNum = MAX_PTZ;
	}
	else if(commandValue == FACE_MOVE_DOWN){
		keyTiltNum -= STEP;
		if(keyTiltNum < MIN_PTZ)
			keyTiltNum = MIN_PTZ;
	}
}

/*
 * @brief: Assume set point is (320, 240), then calculate the error and
 * 			apply PID control to the system. Since a precise result cannot
 * 			be achieved, so an error range is set to ensure a stable point
 * 			is reached.
 *
 * 			if the face is not detected on a zoom value, the zoom value will
 * 			decrease to get a large view of image
 */
void autoTrackingPID(double hOut, double vOut, int area, int faceNum,
		int &keyZoomNum, int &keyPanNum, int &keyTiltNum){

	static int commandValue = 0;

	switch(keyZoomNum){
		case 1:

			if(hOut > ZOOM1_H_ERROR_POSITIVE || hOut < ZOOM1_H_ERROR_NEGATIVE){
				keyPanNum += (STEP*hOut/ZOOM1_PIXEL_PER_H);

				if(keyPanNum < MIN_PTZ)
					keyPanNum = MIN_PTZ;
				else if(keyPanNum > MAX_PTZ)
					keyPanNum = MAX_PTZ;
			}else if(area < ZOOM_SIZE && faceNum == 1 && keyPanNum != MAX_PTZ && keyPanNum != MIN_PTZ)
				keyZoomNum++;
			if(faceCount2 != 0)
				recorrection(commandValue, keyPanNum, keyTiltNum);
			faceCount2 = 0;
			updatePTZ(keyZoomNum, keyPanNum, keyTiltNum);
			break;
		case 2:
			if(faceNum == 0){
				faceCount2++;
				if(faceCount2 > SENTIVITY_LEVEL2)
					keyZoomNum--;
			}else if(hOut > ZOOM2_H_ERROR_POSITIVE || hOut < ZOOM2_H_ERROR_NEGATIVE){
				keyPanNum += (STEP*hOut/ZOOM2_PIXEL_PER_H);
				if(hOut > ZOOM2_H_ERROR_POSITIVE)
					commandValue = FACE_MOVE_LEFT;
				else
					commandValue = FACE_MOVE_RIGHT;
				if(keyPanNum < MIN_PTZ)
					keyPanNum = MIN_PTZ;
				else if(keyPanNum > MAX_PTZ)
					keyPanNum = MAX_PTZ;
			}else if(vOut > ZOOM2_V_ERROR_POSITIVE || vOut < ZOOM2_V_ERROR_NEGATIVE){
				keyTiltNum += (STEP*vOut/ZOOM2_PIXEL_PER_V);
				if(vOut > ZOOM2_V_ERROR_POSITIVE)
					commandValue = FACE_MOVE_UP;
				else
					commandValue = FACE_MOVE_DOWN;
				if(keyTiltNum > MAX_PTZ)
					keyTiltNum = MAX_PTZ;
				else if(keyTiltNum < MIN_PTZ)
					keyTiltNum = MIN_PTZ;
				else if(area < ZOOM_SIZE && faceNum == 1)
					keyZoomNum++;
			}
			if(faceCount3 != 0)
				recorrection(commandValue, keyPanNum, keyTiltNum);
			faceCount3 = 0;
			updatePTZ(keyZoomNum, keyPanNum, keyTiltNum);
			break;
		case 3:
			if(faceNum == 0){
				faceCount3++;
				if(faceCount3 > SENTIVITY_LEVEL3)
					keyZoomNum--;
			}else if(hOut > ZOOM3_H_ERROR_POSITIVE || hOut < ZOOM3_H_ERROR_NEGATIVE){
				keyPanNum += (STEP*hOut/ZOOM3_PIXEL_PER_H);
				if(hOut > ZOOM2_H_ERROR_POSITIVE)
					commandValue = FACE_MOVE_LEFT;
				else
					commandValue = FACE_MOVE_RIGHT;
				if(keyPanNum < MIN_PTZ)
					keyPanNum = MIN_PTZ;
				else if(keyPanNum > MAX_PTZ)
					keyPanNum = MAX_PTZ;
			}else if(vOut > ZOOM3_V_ERROR_POSITIVE || vOut < ZOOM3_V_ERROR_NEGATIVE){
				keyTiltNum += (STEP*vOut/ZOOM3_PIXEL_PER_V);
				if(vOut > ZOOM2_V_ERROR_POSITIVE)
					commandValue = FACE_MOVE_UP;
				else
					commandValue = FACE_MOVE_DOWN;
				if(keyTiltNum > MAX_PTZ)
					keyTiltNum = MAX_PTZ;
				else if(keyTiltNum < MIN_PTZ)
					keyTiltNum = MIN_PTZ;
			}
			updatePTZ(keyZoomNum, keyPanNum, keyTiltNum);
			break;
	}
}

/*
 * 	@brief:
 * 		This function will detect the face and draw a minimum rectangle around it
 */
// Function detectAndDisplay
void detectAndDisplay(Mat frame, double &hOut, double &vOut, int &area, int &faceNum){
#if DEBUG
	Point pt1Copy, pt2Copy;
#endif
	double timeDiff;
	std::vector<Rect> faces;
    Mat frame_gray, crop, res, gray;

    //need grayscale picture
    cvtColor(frame, frame_gray, COLOR_BGR2GRAY);
    //equalise histogram
    equalizeHist(frame_gray, frame_gray);

// Detect faces
    face_cascade.detectMultiScale(frame_gray, faces, 1.1, 5, 0 | CASCADE_SCALE_IMAGE, Size(100, 100));
    faceNum = faces.size();


// Set Region of Interest
    cv::Rect roi_c, roi_b;

    size_t ic = 0; // ic is index of current element
    int ac = 0; // ac is area of current element

    size_t ib = 0; // ib is index of biggest element
    int ab = 0; // ab is area of biggest element

    for (ic = 0; ic < faces.size(); ic++) // Iterate through all current elements (detected faces)
    {
        roi_c.x = faces[ic].x;
        roi_c.y = faces[ic].y;
        roi_c.width = (faces[ic].width);
        roi_c.height = (faces[ic].height);

        ac = roi_c.width * roi_c.height; // Get the area of current element (detected face)

        roi_b.x = faces[ib].x;
        roi_b.y = faces[ib].y;
        roi_b.width = (faces[ib].width);
        roi_b.height = (faces[ib].height);

        ab = roi_b.width * roi_b.height; // Get the area of biggest element, at beginning it is same as "current" element
        //store the largest element
        if (ac > ab){
            ib = ic;
            roi_b.x = faces[ib].x;
            roi_b.y = faces[ib].y;
            roi_b.width = (faces[ib].width);
            roi_b.height = (faces[ib].height);
        }

        crop = frame(roi_b);

        resize(crop, res, Size(128, 128), 0, 0, INTER_LINEAR); // This will be needed later while saving images
        cvtColor(crop, gray, CV_BGR2GRAY); // Convert cropped image to Grayscale

        Point pt1(faces[ic].x, faces[ic].y); // Display detected faces on main window - live stream from camera
        Point pt2((faces[ic].x + faces[ic].height), (faces[ic].y + faces[ic].width));
//----------------------------BELOW for PID control-------------------------------------
        timeDiff = PID(pt1, pt2, hOut, vOut);
//----------------------------ABOVE for PID control-------------------------------------
        area = (pt2.x - pt1.x) * (pt2.y - pt1.y);
        rectangle(frame, pt1, pt2, Scalar(0, 255, 0), 2, 8, 0);
#if DEBUG
       pt1Copy = pt1;
       pt2Copy = pt2;
#endif
    }
#if DEBUG
	string text1, text0, text2, text3;
	stringstream sstm;
	sstm << "pt1: (" << pt1Copy.x << ", " << pt1Copy.y << ") "
			<< "pt2: (" << pt2Copy.x << ", " << pt2Copy.y << ") ";
	text0 = sstm.str();
	sstm.str("");
	sstm << "center: (" << (pt1Copy.x + pt2Copy.x)/2 << ", " <<
			(pt1Copy.y + pt2Copy.y)/2 << ")" << " hOut " << hOut
			<< " vOut " << vOut;
	text1 = sstm.str();
	sstm.str("");
	sstm << "Area: " << (pt2Copy.x - pt1Copy.x) << " * " << (pt2Copy.y - pt1Copy.y)
			<<" timeDiff: " << timeDiff;
	text2 = sstm.str();
	sstm.str("");
 	sstm  << "Zoom Value: " << keyZoomNum <<
			" Pan: " << keyPanNum << " Tilt: " << keyTiltNum;
	text3 = sstm.str();
	putText(frame, text0, cvPoint(30, 30),  FONT_HERSHEY_SIMPLEX, 0.8, cvScalar(0, 0, 255), 3, CV_AA);
	putText(frame, text1, cvPoint(30, 100),  FONT_HERSHEY_SIMPLEX, 0.8, cvScalar(0, 0, 255), 3, CV_AA);
	putText(frame, text2, cvPoint(30, 170),  FONT_HERSHEY_SIMPLEX, 0.8, cvScalar(0, 0, 255), 3, CV_AA);
	putText(frame, text3, cvPoint(30, 240),  FONT_HERSHEY_SIMPLEX, 0.8, cvScalar(0, 0, 255), 3, CV_AA);
#endif
	imshow("original", frame);
}
