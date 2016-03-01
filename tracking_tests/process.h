#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <Leap.h>

using namespace cv;
using namespace std;

void TrackingHoughCircles(const Mat &img);
void TrackingBlobs(const Mat &img);
void TrackingContours(const Mat &img);

//into plugin
Leap::Vector GetTrackedPoint(Leap::Image img);