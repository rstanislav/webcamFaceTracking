#ifndef PTZ_H
#define PTZ_H

#include <opencv2/core/core.hpp>

int PID(cv::Point &pt1, cv::Point &pt2, double &hOut, double &vOut);
void updatePTZ(int keyZoomNum, int keyPanNum, int keyTiltNum);
void manualTracking(char key, int keyZoomNum, int keyPanNum, int keyTiltNum);

#endif
