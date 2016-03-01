#include <opencv2\highgui.hpp>
#include <opencv2\core.hpp>
#include <opencv2\imgproc.hpp>
#include <time.h>
#include <Leap.h>
#include <iostream>

#include "calib.h"
#include "process.h"

using namespace cv;
using namespace Leap;

Mat cvImgLeft, cvImgRight;

enum Behaviour {
	Show_images,
	Undistort_images,
	Calib_image_recording,
	Stereo_calibration,
	Hough_circle_transform,
	Tracking_blobs,
	Tracking_contours,
	Triangulation,
	Restart,
	Quit
};

Behaviour CheckBehaviour();

int main() {
	//leap motion data
	Controller controller;
	controller.setPolicy(Leap::Controller::POLICY_IMAGES);
	controller.setPolicy(Leap::Controller::POLICY_BACKGROUND_FRAMES);

	//process variables
	int updateRate;
	int frameAmount; //amount of frame to record
	int counter;
	bool done;
	FileStorage fs;
	vector<string> imagelist;
	FileNode n;
	FileNodeIterator it, it_begin, it_end;
	Size boardSize;
	int64 t1, t2;
	double time;
	Vector slopes_left, slopes_right, position;
	float cameraZ, cameraY, cameraX;

	//behavior options	
	Behaviour behaviour;
	
start:
	//initialization
	updateRate = 100;
	frameAmount = 20; //amount of frame to record
	counter = 0;
	done = false;
	imagelist.clear();
	time = 0;

	behaviour = CheckBehaviour();
	if (behaviour == Quit) return 0;
	else {
		system("cls");
		std::cout << "Press 'q' to quit, 'r' to restart (set focus on image window)\n";
	}

	char key = ' ';
	while (key != 'q' && key != 'r') {

		key = waitKey(updateRate); //refresh rate

		//image acquisition
		Frame frame = controller.frame();
		if (!frame.isValid()) {
			//std::cout << "Frame is Invalid" << std::endl;
			continue;
		}

		ImageList images = frame.images();
		if (images.isEmpty() || images.count() == 1) {
			//std::cout << "imageList.isEmpty()" << std::endl;
			continue;
		}

		Image imageLeft = images[0];
		Image imageRight = images[1];

		cvImgLeft = Mat(imageLeft.height(), imageLeft.width(), CV_8UC1);
		cvImgRight = Mat(imageRight.height(), imageRight.width(), CV_8UC1);		

		cvImgLeft.data = (unsigned char*)imageLeft.data();
		cvImgRight.data = (unsigned char*)imageRight.data();

		//image output
		imshow("Left image", cvImgLeft);
		imshow("Right image", cvImgRight);

		//behaviour check
		switch (behaviour) {
		case Show_images:
			//do nothing
			break;
		case Undistort_images:
			//use Leap Motion distortion map
			UndistortLeap(imageLeft, "Undistorted left image", true);
			UndistortLeap(imageRight, "Undistorted right image", true);
			break;
		case Calib_image_recording:
			//record calibration images
			counter++;
			//done = imgSave(cvImgLeft, cvImgRight, frameAmount, counter);
			done = imgSave(UndistortLeap(imageLeft, "Undistorted left image", true), UndistortLeap(imageRight, "Undistorted right image", true), frameAmount, counter);
			if (done) {
				system("cls");
				std::cout << "Images for calibration recorded!" <<
					"\nPress 'q' to quit, 'r' to restart (set focus on image window)";
				behaviour = Show_images;
			}
			break;
		case Stereo_calibration:
			//read calibration names to list			
			fs.open(calibFileNames, FileStorage::READ);
			n = fs["strings"];                         // Read string sequence - Get node
			if (n.type() != FileNode::SEQ)
			{
				cerr << "strings is not a sequence! FAIL" << endl;
				behaviour = Show_images;
				break;
			}
			it = n.begin();
			it_end = n.end(); // Go through the node
			for (; it != it_end; ++it) {
				imagelist.push_back((string)*it);
				cout << (string)*it << endl;
			}
			fs.release();

			//provide board size
			boardSize.width = 9;
			boardSize.height = 6;

			//apply calibration, save parameters
			StereoCalibration(imagelist, boardSize, true, true);

			//exit calibration, revert to showing images
			behaviour = Show_images;
			break;
		case Hough_circle_transform:
			//apply calibration
			//rectify calibrated images
			//apply threshold
			//Hough circle transform detection
			t1 = getTickCount();
			TrackingHoughCircles(cvImgLeft);
			t2 = getTickCount();
			time = (t2 - t1) / getTickFrequency();
			cout << "\nExecution time (ms): " << time * 1000.0f;
			break;
		case Tracking_blobs:
			//uncalibrated
			//threshold
			//simple blob detector			
			t1 = getTickCount();
			TrackingBlobs(cvImgLeft);
			t2 = getTickCount();
			time = (t2 - t1) / getTickFrequency();
			cout << "\nExecution time (ms): " << time * 1000.0f;
			break;
		case Tracking_contours:
			//uncalibrated
			//threshold
			//blur + contours + enclosing circle			
			t1 = getTickCount();
			TrackingContours(cvImgLeft);
			t2 = getTickCount();
			time = (t2 - t1) / getTickFrequency();
			cout << "\nExecution time (ms): " << time * 1000.0f;
			break;
		case Triangulation:
			//triangulation based on two tracked points and Leap Motion rectify() function
			t1 = getTickCount();

			//get the direction to the centere of object from left and right images
			//since there is only one tracked object, points should correspond
			slopes_left = imageLeft.rectify(GetTrackedPoint(imageLeft));
			slopes_right = imageRight.rectify(GetTrackedPoint(imageRight));

			//Do the triangulation from the rectify() slopes
			//40 mm camera separation
			cameraZ = 40 / (slopes_right.x - slopes_left.x);
			cameraY = cameraZ * slopes_right.y;
			cameraX = cameraZ * slopes_right.x - 20;
			position = Vector(cameraX, -cameraZ, cameraY);

			t2 = getTickCount();
			time = (t2 - t1) / getTickFrequency();
			cout << "\nPosition: " << position;
			cout << "\nExecution time (ms): " << time * 1000.0f;
			break;
		default:
			//do nothing
			break;
		}
	}

	//restart
	if (key == 'r') {
		destroyAllWindows();
		goto start;
	}

	return 0;
}

Behaviour CheckBehaviour() {
	for (;;) {
		system("cls");
		std::cout << "Select desired behaviour: \n" <<
			"v: View Leap Motion sensor images; \n" <<
			"u: Apply distortion correction; \n" <<
			"i: Record calibration images; \n" <<
			"s: Stereo calibration; \n" <<
			"h: Tracking Hough circles; \n" <<
			"c: Tracking contours; \n" <<
			"b: Tracking blobs; \n" <<
			"t: Triangulation; \n" <<
			"q: Quit; \n" <<
			"\n[?> ";
		char key = getchar();
		switch (key) {
		case 'v':
			return Show_images;
			break;
		case 'u':
			return Undistort_images;
			break;
		case 'i':
			return Calib_image_recording;
			break;
		case 's':
			return Stereo_calibration;
			break;
		case 'h':
			return Hough_circle_transform;
			break;
		case 'b':
			return Tracking_blobs;
			break;
		case 'c':
			return Tracking_contours;
			break;
		case 't':
			return Triangulation;
			break;
		case 'q':
			return Quit;
			break;
		default:
			//repeat			
			continue;
		}		
		break;
	} 
	return Quit;
}