#include "process.h"

void TrackingHoughCircles(const Mat &img) {
	Mat cimg = img.clone();
	CvSize size = img.size();

	//binary threshold, val = 235
	threshold(img, cimg, 235, 255, 0);
	medianBlur(cimg, cimg, 5);

	//Hough transform	
	vector<Vec3f> circles;
	HoughCircles(cimg, circles, HOUGH_GRADIENT, 1, img.rows / 8, 20, 10, 0, 0);

	cvtColor(cimg, cimg, CV_GRAY2BGR);

	for (size_t i = 0; i < circles.size(); i++)
	{
		Vec3i c = circles[i];
		circle(cimg, Point(c[0], c[1]), c[2], Scalar(0, 0, 255), 1, LINE_AA);
		circle(cimg, Point(c[0], c[1]), 2, Scalar(255, 0, 0), 1, LINE_AA);
		//cout << "Center: " << c[0] << "; " << c[1] << "\n";
	}

	imshow("detected circles", cimg);	
}

void TrackingBlobs(const Mat &img)
{
	Mat cimg = img.clone();
	CvSize size = img.size();	

	//binary threshold, val = 235
	threshold(img, cimg, 235, 255, 0);
	medianBlur(cimg, cimg, 5);

	//blob detection
	// set up the parameters (check the defaults in opencv's code in blobdetector.cpp)
	SimpleBlobDetector::Params params;
	params.minDistBetweenBlobs = 150.0f;
	params.filterByInertia = false;
	params.filterByConvexity = false;
	params.filterByColor = false;
	params.filterByCircularity = false;
	params.filterByArea = false;
	params.minArea = 10.0f;
	params.maxArea = 500.0f;

	// set up and create the detector using the parameters
	Ptr<SimpleBlobDetector> blob_detector = SimpleBlobDetector::create(params);

	// detect!
	vector<KeyPoint> keypoints;
	blob_detector->detect(cimg, keypoints);

	// Draw detected blobs as red circles.
	// DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
	Mat im_with_keypoints;
	drawKeypoints(cimg, keypoints, im_with_keypoints, Scalar(0, 0, 255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

	//blob center
	// extract the x y coordinates of the keypoints: 
	for (int i = 0; i<keypoints.size(); i++) {
		int X = keypoints[i].pt.x;
		int Y = keypoints[i].pt.y;

		//cout << "Center: " << X << "; " << Y << "\n";
		circle(im_with_keypoints, cvPoint(X, Y), 1, Scalar(255, 0, 0), 2);
	}	

	// Show blobs
	imshow("keypoints", im_with_keypoints);
}

void TrackingContours(const Mat &img)
{
	Mat cimg = img.clone();
	CvSize size = img.size();	

	//binary threshold, val = 235
	threshold(img, cimg, 235, 255, 0);
	medianBlur(cimg, cimg, 5);

	//circle detection with contours	
	bool enableRadiusCulling = false;
	int minTargetRadius = 5;
	vector<vector<Point> > contours;
	vector<Vec4i> heirarchy;

	//create copy, so findContours will not change binary image
	Mat cont = cimg.clone();
	findContours(cont, contours, heirarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

	size_t count = contours.size();

	//get circle with largest radius
	float radius = 0;
	Point2i center;

	for (int i = 0; i < count; i++)
	{
		Point2f c;
		float r;
		minEnclosingCircle(contours[i], c, r);

		if (!enableRadiusCulling || r >= minTargetRadius)
		{
			if (r > radius) {
				radius = r;
				center = (Point2i) c;
			}
		}
	}

	cvtColor(cimg, cimg, CV_GRAY2BGR);
	
	Scalar red(0, 0, 255);
	Scalar blue(255, 0, 0);

	if (radius > 0) { //circle was found
		circle(cimg, center, radius, red, 1);
		circle(cimg, center, 1, blue, 2);
		//cout << "Center: " << center.x << "; " << center.y << "\n";
	}

	imshow("detected circles", cimg);	
}

Leap::Vector GetTrackedPoint(Leap::Image image)
{
	Mat img = Mat(image.height(), image.width(), CV_8UC1);
	img.data = (unsigned char*)image.data();

	Mat cimg = img.clone();
	CvSize size = img.size();

	//binary threshold, val = 235
	threshold(img, cimg, 235, 255, 0);
	medianBlur(cimg, cimg, 5);

	//circle detection with contours	
	bool enableRadiusCulling = false;
	int minTargetRadius = 5;
	vector<vector<Point> > contours;
	vector<Vec4i> heirarchy;

	findContours(cimg, contours, heirarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

	size_t count = contours.size();

	//get circle with largest radius
	float radius = 0;
	Point2i center;

	for (int i = 0; i < count; i++)
	{
		Point2f c;
		float r;
		minEnclosingCircle(contours[i], c, r);

		if (!enableRadiusCulling || r >= minTargetRadius)
		{
			if (r > radius) {
				radius = r;
				center = (Point2i)c;
			}
		}
	}

	Leap::Vector res;
	res.x = center.x;
	res.y = center.y;
	res.z = 0;

	/*
	cvtColor(cimg, cimg, CV_GRAY2BGR);

	Scalar red(0, 0, 255);
	Scalar blue(255, 0, 0);

	if (radius > 0) { //circle was found
		circle(cimg, center, radius, red, 1);
		circle(cimg, center, 1, blue, 2);
		//cout << "Center: " << center.x << "; " << center.y << "\n";
	}
	imshow("detected circles", cimg);
	*/
	return res;
}
