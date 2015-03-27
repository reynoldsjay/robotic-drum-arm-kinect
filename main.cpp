#include <OpenNI.h>
#include <opencv2\core\core.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <iostream>

using namespace openni;
using namespace cv;

void detectColor(Mat img, int minHue, int minSatur, int minValue, int maxHue, int maxSatur, int maxValue);

int main(int argc, char** argv) {

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

	
	namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"
	// Can use trackbars to pick calibrate color of interest.
	int minHue = 0;
	int maxHue = 179;

	int minSatur = 0;
	int maxSatur = 255;

	int minValue = 0;
	int maxValue = 255;

	
	//Create trackbars in "Control" window
	cvCreateTrackbar("minHue", "Control", &minHue, 179); //Hue (0 - 179)
	cvCreateTrackbar("maxHue", "Control", &maxHue, 179);

	cvCreateTrackbar("minSatur", "Control", &minSatur, 255); //Saturation (0 - 255)
	cvCreateTrackbar("maxSatur", "Control", &maxSatur, 255);

	cvCreateTrackbar("minValue", "Control", &minValue, 255);//Value (0 - 255)
	cvCreateTrackbar("maxValue", "Control", &maxValue, 255);
	

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
					detectColor(colorcv, minHue, minSatur, minValue, maxHue, maxSatur, maxValue);
					cv::cvtColor(colorcv, colorcv, CV_BGR2RGB);
					cv::imshow("RGB", colorcv);
				}
				break;

			default:
				puts("Error retrieving a stream");
			}
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

void detectColor(Mat img, int minHue, int minSatur, int minValue, int maxHue, int maxSatur, int maxValue) {
	
	
		Mat imgHSV;
		Mat imgBin;

		

		// Convert from BGR to HSV
		cvtColor(img, imgHSV, COLOR_BGR2HSV);
		
		inRange(imgHSV, Scalar(minHue, minSatur, minValue), Scalar(maxHue, maxSatur, maxValue), imgBin); //Threshold the image

		//morphological opening (removes noise from the foreground)		
		erode(imgBin, imgBin, getStructuringElement(MORPH_ELLIPSE, Size(3, 3)));
		dilate(imgBin, imgBin, getStructuringElement(MORPH_ELLIPSE, Size(3, 3)));

		//morphological closing (removes small holes from the foreground)
		dilate(imgBin, imgBin, getStructuringElement(MORPH_ELLIPSE, Size(3, 3)));
		erode(imgBin, imgBin, getStructuringElement(MORPH_ELLIPSE, Size(3, 3)));

		imshow("Thresholded Image", imgBin); //show the thresholded image
		//imshow("Original", imgOrig); //show the original image
		
		/*
		if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
		{
			//cout << "esc key is pressed by user" << endl;
			break;
		} 
		*/
	}
