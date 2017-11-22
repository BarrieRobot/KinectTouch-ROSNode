/*
 * KinectTouch ROS Publisher node
 */
// %Tag(FULLTEXT)%
// %Tag(ROS_HEADER)%
#include "ros/ros.h"
// %EndTag(ROS_HEADER)%
// %Tag(MSG_HEADER)%
#include "std_msgs/String.h"
#include "kt/Cursor.h"
#include "kt/Reset.h"
// %EndTag(MSG_HEADER)%

#include <sstream>
#include <string>

#include <iostream>
#include <fstream>
#include <vector>
#include <map>
using namespace std;

// openCV
#include <opencv/highgui.h>
#include <opencv/cv.h>
using namespace cv;


// openNI
#include <XnOpenNI.h>
#include <XnCppWrapper.h>
using namespace xn;
#define CHECK_RC(rc, what)							\
	if (rc != XN_STATUS_OK)							\
	{									\
		printf("%s failed: %s\n", what, xnGetStatusString(rc));		\
		return rc;							\
	}

//---------------------------------------------------------------------------
// Globals
//---------------------------------------------------------------------------

// OpenNI
xn::Context xnContext;
xn::DepthGenerator xnDepthGenerator;
xn::ImageGenerator xnImgeGenertor;

bool mousePressed = false;

int initOpenNI(const XnChar* fname) {
	XnStatus nRetVal = XN_STATUS_OK;
	ScriptNode scriptNode;

	// initialize context
	nRetVal = xnContext.InitFromXmlFile(fname, scriptNode);
	CHECK_RC(nRetVal, "InitFromXmlFile");

	// initialize depth generator
	nRetVal = xnContext.FindExistingNode(XN_NODE_TYPE_DEPTH, xnDepthGenerator);
	CHECK_RC(nRetVal, "FindExistingNode(XN_NODE_TYPE_DEPTH)");

	// initialize image generator
	nRetVal = xnContext.FindExistingNode(XN_NODE_TYPE_IMAGE, xnImgeGenertor);
	CHECK_RC(nRetVal, "FindExistingNode(XN_NODE_TYPE_IMAGE)");

	return 0;
}

void average(vector<Mat1s>& frames, Mat1s& mean) {
	Mat1d acc(mean.size());
	Mat1d frame(mean.size());

	for (unsigned int i=0; i<frames.size(); i++) {
		frames[i].convertTo(frame, CV_64FC1);
		acc = acc + frame;
	}

	acc = acc / frames.size();

	acc.convertTo(mean, CV_16SC1);
}

int xMin = 0;
int xMax = 0;
int yMin = 0;
int yMax = 0;

void parse_setting (string str) {
	int i;
	string delimiter = "=";
	string token = str.substr(0, str.find(delimiter));
	str.erase(0, str.find(delimiter) + delimiter.length());
	std::istringstream(str) >> i;
	if (token.compare("minx") == 0) {
		xMin = i;
	} else if (token.compare("maxx") == 0) {
		xMax = i;
	} else if (token.compare("miny") == 0) {
		yMin = i;
	} else if (token.compare("maxy") == 0) {
		yMax = i;
	}
}

Mat1s depth(480, 640); // 16 bit depth (in millimeters)
const unsigned int nBackgroundTrain = 30;// How long is the background trained? (frames used in calculation)
const unsigned short touchDepthMin = 15; // How far from the surface a touch needs to be (higher -> touching is not registered + less noise, lower -> more noise)
const unsigned short touchDepthMax = 40; // How far from the surface a touch can be (higher -> touch in air is registered)
const unsigned int touchMinArea = 50;
Mat1b depth8(480, 640); // 8 bit depth
Mat3b rgb(480, 640); // 8 bit depth
Mat3b debug(480, 640); // debug visualization
Mat1s foreground(640, 480);
Mat1b foreground8(640, 480);
Mat1b touch(640, 480); // touch mask
Mat1s background(480, 640);
vector<Mat1s> buffer(nBackgroundTrain);

bool reset_bg(kt::Reset::Request  &req, kt::Reset::Response &res)
{
  ROS_INFO("I heard I need to reset");
	for (unsigned int i=0; i<nBackgroundTrain; i++) {
		xnContext.WaitAndUpdateAll();
		depth.data = (uchar*) xnDepthGenerator.GetDepthMap();
		buffer[i] = depth;
	}
	average(buffer, background);
	return true;
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
	ros::init(argc, argv, "kt");

	std::ifstream file("/home/jonathan/catkin_ws/src/kt/src/settings.txt");
  std::string str;
  while (std::getline(file, str))
  {
		parse_setting(str);
  }
	file.close();

	ROS_INFO("MinX=%d", xMin);
	ROS_INFO("MaxX=%d", xMax);
	ROS_INFO("MinY=%d", yMin);
	ROS_INFO("MaxY=%d", yMax);

	const bool localClientMode = true; 					// connect to a local client

	const double debugFrameMaxDepth = 4000; // maximal distance (in millimeters) for 8 bit debug depth frame quantization
	const char* windowName = "Debug";
	const Scalar debugColor0(0,0,128);
	const Scalar debugColor1(255,0,0);
	const Scalar debugColor2(255,255,255);

	initOpenNI("/home/jonathan/catkin_ws/src/kinect_touch/src/niConfig.xml");

	// create some sliders
	namedWindow(windowName);
	createTrackbar("xMin", windowName, &xMin, 640);
	createTrackbar("xMax", windowName, &xMax, 640);
	createTrackbar("yMin", windowName, &yMin, 480);
	createTrackbar("yMax", windowName, &yMax, 480);

	// create background model (average depth)
	for (unsigned int i=0; i<nBackgroundTrain; i++) {
		xnContext.WaitAndUpdateAll();
		depth.data = (uchar*) xnDepthGenerator.GetDepthMap();
		buffer[i] = depth;
	}
	average(buffer, background);

  ros::NodeHandle n;
	ros::Publisher pub = n.advertise<kt::Cursor>("kinect_touch", 1000);
  ros::ServiceServer service = n.advertiseService("reset_bg", reset_bg);
// %Tag(LOOP_RATE)%
  ros::Rate loop_rate(10);
// %EndTag(LOOP_RATE)%

  while (ros::ok())
  {
	// read available data
		xnContext.WaitAndUpdateAll();

		// update 16 bit depth matrix
		depth.data = (uchar*) xnDepthGenerator.GetDepthMap();
		//xnImgeGenertor.GetGrayscale8ImageMap()

		// update rgb image
		//rgb.data = (uchar*) xnImgeGenertor.GetRGB24ImageMap(); // segmentation fault here
		//cvtColor(rgb, rgb, CV_RGB2BGR);

		// extract foreground by simple subtraction of very basic background model
		foreground = background - depth;

		// find touch mask by thresholding (points that are close to background = touch points)
		touch = (foreground > touchDepthMin) & (foreground < touchDepthMax);

		// extract ROI
		Rect roi(xMin, yMin, xMax - xMin, yMax - yMin);
		Mat touchRoi = touch(roi);

		// find touch points
		vector< vector<Point2i> > contours;
		vector<Point2f> touchPoints;
		findContours(touchRoi, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, Point2i(xMin, yMin));
		for (unsigned int i=0; i<contours.size(); i++) {
			Mat contourMat(contours[i]);
			// find touch points by area thresholding
			if ( contourArea(contourMat) > touchMinArea ) {
				Scalar center = mean(contourMat);
				Point2i touchPoint(center[0], center[1]);
				touchPoints.push_back(touchPoint);
			}
		}

    kt::Cursor list_msg;
		for (unsigned int i=0; i<touchPoints.size(); i++) { // touch points
			float cursorX = (touchPoints[i].x - xMin) / (xMax - xMin);
			float cursorY = 1 - (touchPoints[i].y - yMin)/(yMax - yMin);
			//printf("Touch found at %f, %f\n", cursorX, cursorY);
			// TuioCursor* cursor = tuio->getClosestTuioCursor(cursorX,cursorY);
			kt::point point_msg;
			point_msg.x = cursorX;
			point_msg.y = cursorY;
			list_msg.cursors.push_back(point_msg);
		//	ROS_INFO("%f %f", msg.cursorX, msg.cursorY);

		}
		pub.publish(list_msg);

		// draw debug frame
		depth.convertTo(depth8, CV_8U, 255 / debugFrameMaxDepth); // render depth to debug frame
		cvtColor(depth8, debug, CV_GRAY2BGR);
		debug.setTo(debugColor0, touch);  // touch mask
		rectangle(debug, roi, debugColor1, 2); // surface boundaries
		for (unsigned int i=0; i<touchPoints.size(); i++) { // touch points
			circle(debug, touchPoints[i], 5, debugColor2, CV_FILLED);
		}

		// render debug frame (with sliders)
		imshow(windowName, debug);
		// imshow("image", rgb);
		ros::spinOnce();
		loop_rate.sleep();
  }


  return 0;
}
