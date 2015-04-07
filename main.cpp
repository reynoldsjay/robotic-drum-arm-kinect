#include <OpenNI.h>
#include <opencv2\core\core.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <iostream>
#include "Client.h"
#include <math.h>

#define PI 3.1415926535897

using namespace openni;
using namespace cv;
using namespace std;

void detectColor(Mat img, int minHue, int minSatur, int minValue, int maxHue, int maxSatur, int maxValue, int window);
void CallBackFunc(int event, int x, int y, int flags, void* ptr);
void clickDrums(Mat img);

// Declare variables for x,y locations of drums, servo, and tip
int drum1x, drum1y, drum2x, drum2y;
int servoX, servoY, tipX, tipY;


int main(int argc, char** argv) {
	
	int count = 0;
	OpenNI::initialize();
	puts("Kinect initialization...");
	Device device;
	if (device.open(openni::ANY_DEVICE) != 0)
	{
		puts("Kinect not found !");
		return -1;
	}
	puts("Kinect opened");
	VideoStream depth, color;
	color.create(device, SENSOR_COLOR);
	color.start();
	puts("Camera ok");
	depth.create(device, SENSOR_DEPTH);
	depth.start();
	puts("Depth sensor ok");
	VideoMode paramvideo;
	paramvideo.setResolution(640, 480);
	paramvideo.setFps(30);
	paramvideo.setPixelFormat(PIXEL_FORMAT_DEPTH_100_UM);
	depth.setVideoMode(paramvideo);
	paramvideo.setPixelFormat(PIXEL_FORMAT_RGB888);
	color.setVideoMode(paramvideo);
	puts("Set video modes.");

	// If the depth/color synchronisation is not necessary, start is faster :
	device.setDepthColorSyncEnabled(false);

	// Otherwise, the streams can be synchronized with a reception in the order of our choice :
	//device.setDepthColorSyncEnabled( true );
	//device.setImageRegistrationMode( openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR );

	VideoStream** stream = new VideoStream*[2];
	
	stream[0] = &depth;
	stream[1] = &color;
	puts("Kinect initialization completed");


	// Define HSV for Color1
	namedWindow("Color1", CV_WINDOW_AUTOSIZE); //create a window called "Color1"
	// Can use trackbars to pick calibrate color of interest.
	int minHue = 0;
	int maxHue = 179;

	int minSatur = 0;
	int maxSatur = 255;

	int minValue = 0;
	int maxValue = 255;

	
	//Create trackbars in "Color1" window
	cvCreateTrackbar("minHue", "Color1", &minHue, 179); //Hue (0 - 179)
	cvCreateTrackbar("maxHue", "Color1", &maxHue, 179);

	cvCreateTrackbar("minSatur", "Color1", &minSatur, 255); //Saturation (0 - 255)
	cvCreateTrackbar("maxSatur", "Color1", &maxSatur, 255);

	cvCreateTrackbar("minValue", "Color1", &minValue, 255);//Value (0 - 255)
	cvCreateTrackbar("maxValue", "Color1", &maxValue, 255);

	minHue = 65;
	maxHue = 82;

	minSatur = 45;
	maxSatur = 150;

	minValue = 201;
	maxValue = 255;

	puts("Color1");

	// Define HSV for Color2
	namedWindow("Color2", CV_WINDOW_AUTOSIZE); //create a window called "Color2"
	// Can use trackbars to pick calibrate color of interest.
	int minHue2 = 0;
	int maxHue2 = 179;

	int minSatur2 = 0;
	int maxSatur2 = 255;

	int minValue2 = 0;
	int maxValue2 = 255;


	//Create trackbars in "Color2" window
	cvCreateTrackbar("minHue", "Color2", &minHue2, 179); //Hue (0 - 179)
	cvCreateTrackbar("maxHue", "Color2", &maxHue2, 179);

	cvCreateTrackbar("minSatur", "Color2", &minSatur2, 255); //Saturation (0 - 255)
	cvCreateTrackbar("maxSatur", "Color2", &maxSatur2, 255);

	cvCreateTrackbar("minValue", "Color2", &minValue2, 255);//Value (0 - 255)
	cvCreateTrackbar("maxValue", "Color2", &maxValue2, 255);

	minHue2 = 125;
	maxHue2 = 145;

	minSatur2 = 117;
	maxSatur2 = 255;

	minValue2 = 136;
	maxValue2 = 255;

	puts("Color 2");

	Client client;
	

	if (device.getSensorInfo(SENSOR_DEPTH) != NULL)
	{
		VideoFrameRef depthFrame, colorFrame;
		cv::Mat colorcv(cv::Size(640, 480), CV_8UC3, NULL);
		cv::Mat depthcv(cv::Size(640, 480), CV_16UC1, NULL);
		cv::namedWindow("RGB", CV_WINDOW_AUTOSIZE);
		cv::namedWindow("Depth", CV_WINDOW_AUTOSIZE);

		int changedIndex;
		while (device.isValid())
		{
			OpenNI::waitForAnyStream(stream, 2, &changedIndex);
			switch (changedIndex)
			{
			case 0:
				depth.readFrame(&depthFrame);

				if (depthFrame.isValid())
				{
					depthcv.data = (uchar*)depthFrame.getData();
					cv::imshow("Depth", depthcv);
				}
				break;

			case 1:
				color.readFrame(&colorFrame);

				if (colorFrame.isValid())
				{
					colorcv.data = (uchar*)colorFrame.getData();
					// Detect color depending HSV values thresholded
					detectColor(colorcv, minHue, minSatur, minValue, maxHue, maxSatur, maxValue, 0);
					detectColor(colorcv, minHue2, minSatur2, minValue2, maxHue2, maxSatur2, maxValue2, 1);
					cv::cvtColor(colorcv, colorcv, CV_BGR2RGB);

					// Open the first frame of the window to provide locations of drum 1 and 2 by clicking on the frame
					if (count == 0) {
						clickDrums(colorcv);
						count++;
					}

					cv::imshow("RGB", colorcv);
				}
				break;

			default:
				puts("Error retrieving a stream");
			}
			
			// Law of Cosine Calculation to find angles that will correct the servo's position to hit the center of the drum1 and drum2.
			// Assumes angle from frame and real world will be approx same when mapping from pixels to distances.
			double a = sqrt(pow((servoX - tipX), 2) + pow((servoY - tipY), 2));

			double b1 = sqrt(pow((servoX - drum1x), 2) + pow((servoY - drum1y), 2));
			double c1 = sqrt(pow((tipX - drum1x), 2) + pow((tipY - drum1y), 2));
			double theta1 = acos((pow(c1, 2) - pow(b1,2) - pow(a,2))/(-2*a*b1));

			//printf("color diff: %d \n", servoX - tipX);
			//printf("%f\n", pow((servoX - tipX), 2));
			//printf("color 1:  %d, %d \n", servoX, servoY);
			//printf("color 2:  %d, %d \n", tipX, tipY);
			//printf("srqrt: %f\n", b1);

			double b2 = sqrt(pow((servoX - drum2x), 2) + pow((servoY - drum2y), 2));
			double c2 = sqrt(pow((tipX - drum2x), 2) + pow((tipY - drum2y), 2));
			double theta2 = acos((pow(c2, 2) - pow(b2, 2) - pow(a, 2)) / (-2 * a*b2));
			//printf("srqrt: %f\n", b2);
			//printf("%f \n", theta*(180/PI));

			// Send the correct angles depending on where the tip is relative to the drum in the x axis.
			if (drum1x < tipX) {
				theta1 = theta1 * -1;
			}
			theta2 = theta2 * -1;
			if (drum2x > tipX) {
				theta2 = theta2 * -1;
			}

			printf("angle to drum one: %f\n", theta1*(180/PI));
			printf("angle to drum two: %f\n", theta2*(180/PI));

			// Send the angles in a UDP packet to the specified IP address
			client.sendAngle1(theta1);
			client.sendAngle2(theta2);


			cv::waitKey(1);
		}

		cv::destroyWindow("RGB");
		cv::destroyWindow("Depth");
	}
	depth.stop();
	depth.destroy();
	color.stop();
	color.destroy();
	device.close();
	OpenNI::shutdown();

	
	
} 



void detectColor(Mat img, int minHue, int minSatur, int minValue, int maxHue, int maxSatur, int maxValue, int window) {
		Mat imgHSV;
		Mat imgBin;

		// Convert from BGR to HSV
		cvtColor(img, imgHSV, COLOR_BGR2HSV);
		
		// Threshold the image depending on the HSV values specified
		inRange(imgHSV, Scalar(minHue, minSatur, minValue), Scalar(maxHue, maxSatur, maxValue), imgBin); 

		// Morphological opening (removes noise from the foreground) 
		// Size of filter can be adjusted for size of noise needed to be removed
		erode(imgBin, imgBin, getStructuringElement(MORPH_ELLIPSE, Size(3, 3)));
		dilate(imgBin, imgBin, getStructuringElement(MORPH_ELLIPSE, Size(3, 3)));

		// Morphological closing (fills small holes from the foreground)
		// Size of filter can be adjusted for size of holes needed to be filled
		dilate(imgBin, imgBin, getStructuringElement(MORPH_ELLIPSE, Size(3, 3)));
		erode(imgBin, imgBin, getStructuringElement(MORPH_ELLIPSE, Size(3, 3)));

		// Calculate the moments to estimate the position of the ball
		Moments theMoments = moments(imgBin);

		// The actual moment values
		double moment10 = theMoments.m10;
		double moment01 = theMoments.m01;

		double area = theMoments.m00;

		static int posX = 0;

		static int posY = 0;

		int lastX = posX;

		int lastY = posY;

		posX = moment10 / area;
		posY = moment01 / area;

		// Print it out for debugging purposes
		//printf("position (%d,%d)\n", posX, posY);

		circle(imgBin, Point(posX, posY), 32.0, Scalar(0, 255, 255), 1, 8);

		//Client::sendTheUDP(1.0);
		
		// Send the centroid position of Servo if window is Color1
		if (window == 0) {
			imshow("Threshold Color1", imgBin); //show the thresholded image
			servoX = posX;
			servoY = posY;
		}

		// Send the centroid position of Tip if window is Color2
		else {
			imshow("Threshold Color2", imgBin);
			tipX = posX;
			tipY = posY;
		}
		
	}

void CallBackFunc(int event, int x, int y, int flags, void *ptr) {
	// If CTRL and Left mouse is clicked, then save pixel location of mouse to drum1
	if (flags == (EVENT_FLAG_CTRLKEY + EVENT_FLAG_LBUTTON)) {
		drum1x = x;
		drum1y = y;
		cout << "Left mouse button is clicked while pressing CTRL key - position (" << x << ", " << y << ")" << endl;
	}

	// If Shift and Left mouse is clicked, then save pixel location of mouse to drum2
	else if (flags == (EVENT_FLAG_LBUTTON + EVENT_FLAG_SHIFTKEY)) {
		drum2x = x;
		drum2y= y;
		cout << "Left mouse button is clicked while pressing SHIFT key - position (" << x << ", " << y << ")" << endl;
	}

}

void clickDrums(Mat img) {
	Point p;

	//Create a window
	namedWindow("Drum Locator", 1);

	//set the callback function for any mouse event
	setMouseCallback("Drum Locator", CallBackFunc, &p);

	//Display the image
	imshow("Drum Locator", img);

	cout << "Ctrl + Left click to Store Drum 1's location\r\n";
	cout << "Shift + Left click to Store Drum 2's location\r\n";
	cout << "When done hit any key\r\n";

	// Wait until user press some key
	waitKey(0);

	cout << "X1:" << drum1x << ", Y1:" << drum1y << endl;
	cout << "X2:" << drum2x << ", Y2:" << drum2y << endl;

	destroyWindow("Drum Locator");
}