#pragma once
#include <opencv2\core.hpp>
#include <opencv2\opencv.hpp>

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <vector>
#include <string>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

#include <Leap.h>

#define calibFileNames "calibImages.xml"

using namespace cv;
using namespace std;
using namespace Leap;


bool imgSave(Mat imgLeft, Mat imgRight, int count, int counter);
void StereoCalibration(const vector<string>& imagelist, Size boardSize, bool displayCorners = false, bool useCalibrated = true, bool showRectified = true);
Mat UndistortLeap(Image img, string name, bool show);