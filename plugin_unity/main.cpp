#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <Leap.h>
#define DLLExport __declspec (dllexport)

using namespace cv;
using namespace std;

struct ReturnPoint
{
	int x;
	int y;
};

extern "C"
{
	DLLExport bool GetTrackedPoint(unsigned char* data, int h, int w, struct ReturnPoint* res)
	{
		Mat img = Mat(h, w, CV_8UC1);
		img.data = data;

		CvSize size = img.size();

		//binary threshold, val = 235
		threshold(img, img, 235, 255, 0);
		medianBlur(img, img, 5);

		//circle detection with contours	
		bool enableRadiusCulling = false;
		int minTargetRadius = 5;
		vector<vector<Point>> contours;
		vector<Vec4i> heirarchy;

		findContours(img, contours, heirarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

		size_t count = contours.size();

		if (count > 0) { //contours found
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
			res->x = center.x;
			res->y = center.y;
			return true;
		}

		//default: contours not found
		res->x = 0;
		res->y = 0;				

		return false;
	}
}