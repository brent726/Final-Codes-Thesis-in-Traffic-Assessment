#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/videoio/videoio.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "Vehicle.h"
#include <iostream>
#include <ctype.h>
#include <fstream>

using namespace cv;
using namespace std;

//global variables
Point2f point;
bool addRemovePt = false;
bool addRefPt = false;

#define ATD at<double>
#define elif else if

const int MIN_OBJECT_AREA = 5 * 5;
int HroadTopX = 0;
int HroadTopY = 0;
int HroadBotX = 0;
int HroadBotY = 0;
int VroadTopX = 0;
int VroadTopY = 0;
int VroadBotX = 0;
int VroadBotY = 0;


//functions
static void onMouse(int event, int x, int y, int /*flags*/, void* /*param*/);
void addPoints(int x, int y);
string intToString(int number);
void drawObject(vector<Vehicle> VehicleCars, Mat &frame);
void searchForVehicle(Mat thresholdImage, vector<Vehicle> &vehicles);
void searchForVehicleV(Mat thresholdImage, vector<Vehicle> &vehicles);
Mat labThresholdingIntersection(Mat labRoadImage);
Mat labThresholdingStraight(Mat labRoadImage);
Rect roadDetectionHorizontal(Mat roadImage, int intersection);
Rect roadDetectionVertical(Mat roadImage);
void secondsToTime(int seconds, int &hr, int &min, int &sec);
Scalar speedSpectrum(double speed);
void setLabel(cv::Mat& im, const std::string label, const cv::Point & pointor);
Point2i vehicleCountTopAndBot(vector<Vehicle> &vehicles, int halfY);

Mat sobelDetection(Mat curFrame);

Mat get_fx(Mat &src1, Mat &src2);
bool isInsideImage(int y, int x, Mat &m);

double get_Sum9(Mat &m, int y, int x);
Mat get_Sum9_Mat(Mat &m);
Mat getLucasKanadeOpticalFlow(Mat &img1, Mat &img2, Mat u, Mat v);
Mat LKDetection(Mat prevFrame, Mat curFrame, Mat u, Mat v);

int main(int argc, char** argv)
{
	//inputs
	string videofile = argv[1];
	//string videofile = "E:/Traffic Videos/DJI_0008(20sec).MP4";
	int intersection = stoi(argv[2]);
	int heading = stoi(argv[3]);
	/*
	E:/Project Alpha/Videos/Take 3(60m).mp4
	E:/zoom60.MP4
	E:/Visual Studio Projects/stabilize30.avi
	D:/Traffic Videos/intersection3-4.MP4
	D:/Traffic Videos/Take 3(60m).mp4
	D:/Traffic Videos/DJI_0008.MP4
	E:/Traffic Videos/intersection.MP4
	D:/Traffic Videos/DJI_0010.MP4
	D:/Traffic Videos/DJI_0009.MP4
	DJI_0011.MP4
	intersection.MP4
	*/

	//variables
	double groundreso = 3.0 / 30;//3.0m/30pixels
	bool pause = false;
	double rightTotal = 0;
	int rightCounter = 0;
	double leftTotal = 0;
	int leftCounter = 0;
	double rightAve, leftAve;
	double refAve = 0;

	//for intersection
	double upAve, downAve;

	//for intersection
	double upTotal = 0;
	int upCounter = 0;
	double downTotal = 0;
	int downCounter = 0;

	//vehicle count
	int numberOfVehicles = 0;
	int numberOfVehiclesV = 0;
	int TopNumberOfCars = 0;
	int BotNumberOfCars = 0;
	int LeftNumberOfCars = 0;
	int RightNumberOfCars = 0;
	double TopRoadDensity = 0;
	double BotRoadDensity = 0;

	

	//for intersection
	double VRoadLength = 0;
	double LeftRoadDensity = 0;
	double RightRoadDensity = 0;

	//opencv variables
	VideoCapture cap;
	TermCriteria termcrit(TermCriteria::COUNT | TermCriteria::EPS, 20, 0.03);
	Size subPixWinSize(10, 10), winSize(31, 31);
	const int MAX_COUNT = 500;
	
	const char *vfilestr = videofile.c_str();

	cap.open(videofile);
	
	if (!cap.isOpened())
	{
		std::cout << "Could not initialize capturing...\n";
		return 0;
	}

	//namedWindow("Output", 1);
	//setMouseCallback("Output", onMouse, 0);

	//video parameters
	double fps = cap.get(CV_CAP_PROP_FPS);
	double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
	double dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video
	/*
	std::cout << "Video file:" << vfilestr << endl;
	std::cout << "Heading:" << heading << endl;
	string typeofroad = (intersection == 0) ? "Straight" : "Intersection";
	std::cout << "Type of road: " << typeofroad << endl;
	std::cout << "Video FPS: " << fps << endl;
	std::cout << "Frame Size = " << dWidth << "x" << dHeight << endl;
	*/
	int framepos, totalframes;
	framepos = 0;
	totalframes = (int)cap.get(CV_CAP_PROP_FRAME_COUNT) - 1;
	//image variables
	Mat gray, prevGray, image, frame, grayImage1, thresholdImage;
	int minDist = 7;// Minimum distance between any two tracking points
	//sobel parameters
	Mat grad;
	Mat grad_x, grad_y;
	Mat abs_grad_x, abs_grad_y;
	Mat sobelImageH, sobelImageV;
	Mat curFrame;
	Mat image2detectionH, image2detectionV;
	int scale = 1;
	int delta = 0;
	int ddepth = CV_16S;

	//road Rectangle
	Rect HroadBorder(0, 0, 0, 0);
	Rect HROAD(0, 0, 0, 0);
	Rect VroadBorder(0, 0, 0, 0);
	Rect VROAD(0, 0, 0, 0);
	int halfY = 0;
	Point HhalfFirstpoint;
	Point HhalfSecondPoint;
	Point VhalfFirstpoint;
	Point VhalfSecondPoint;

	vector<Point2f> refPoints[2];
	vector<Point2f> points[2];
	vector<Vehicle> vehicles;
	vector<Vehicle> tempvehicles;
	//for intersection
	vector<Vehicle> vehiclesV;
	vector<Vehicle> tempvehiclesV;
	vector<Point2f> pointsV[2];
	

	//labels
	
	std::string topRoadName = "";
	std::string botRoadName = "";
	std::string leftRoadName = "";
	std::string rightRoadName = "";


	if ((heading >= 46 && heading <= 134) || (heading >= 226 && heading <= 314))
	{
		topRoadName = "East Bound";
		botRoadName = "West Bound";
		if (intersection)
		{
			leftRoadName = "North Bound";
			rightRoadName = "South Bound";
		}
	}
	else
	{
		topRoadName = "North Bound";
		botRoadName = "South Bound";
		if (intersection)
		{
			leftRoadName = "East Bound";
			rightRoadName = "West Bound";
		}
	}
	//compass
	int angle = heading - 360;
	Point center = Point((dWidth)-60, (dHeight)-60);
	double angleRad = angle*CV_PI / 180.0;
	int length = 50;
	auto direction = cv::Point(length * -cos(angleRad), length * sin(angleRad)); // calculate direction
	double tipLength = .4 + 0.4 * (angle % 180) / 360;
	
	//std::cout << "Total Frames: " << totalframes << endl;
	//std::cout << "Progress" << endl;
	std::string writefile = videofile + "_RESULT.avi";
	//std::cout << topRoadName << " Vehicle Density," << botRoadName << " Vehicle Density," << topRoadName << " Average Speed," << botRoadName << " Average Speed" << endl;
	
	//write video to file initialization
	Size frameSize(static_cast<int>(dWidth), static_cast<int>(dHeight));
	VideoWriter oVideoWriter(writefile, CV_FOURCC('D', 'I', 'V', 'X'), fps, frameSize, true); //initialize the VideoWriter object 
	if (!oVideoWriter.isOpened()) //if not initialize the VideoWriter successfully, exit the program
	{
		cout << "ERROR: Failed to write the video" << endl;
		waitKey(10000);
		return -1;
	}
	

	//logs
	
	std::string logfile = videofile + ".csv";
	ofstream myfile;
	myfile.open(logfile, std::ofstream::out);
	myfile << "Video file: , " << vfilestr;
	myfile << "\nVideo FPS: , " << fps;
	myfile << "\nFrame Size: , " << dWidth << "x" << dHeight;
	myfile << "\nTime(hr:min:sec),"<< "H-Road Total No. of Vehicles,";
	myfile << topRoadName << " No. of Vehicles," << botRoadName << " No. of Vehicles,";

	if (intersection)
	{
		myfile << "V-ROAD Total No. of Vehicles,";
		myfile << leftRoadName << " No. of Vehicles," << rightRoadName << " No. of Vehicles,";
	}

	myfile << topRoadName << " Vehicle Density," << botRoadName << " Vehicle Density,";

	if (intersection)
	{
		myfile << leftRoadName << " Vehicle Density," << rightRoadName << " Vehicle Density,";
	}

	myfile << topRoadName << " Average Speed," << botRoadName << " Average Speed,";

	if (intersection)
	{
		myfile << leftRoadName << " Average Speed," << rightRoadName << " Average Speed";
	}
	

	cv::Point2f RP[4] = {Point2f(170,65) ,Point2f(185,605) ,Point2f(1025,630) ,Point2f(1010,150)};
	addRefPt = false;

	for (;;)
	{
		cap >> frame;
		if (frame.empty())
			break;

		framepos = (int)cap.get(CV_CAP_PROP_POS_FRAMES);

		frame.copyTo(image);
		frame.copyTo(image2detectionH);
		frame.copyTo(image2detectionV);


		//road detection
		if (framepos == 1 || framepos % 30 == 0)
		{
			HroadBorder = roadDetectionHorizontal(image,intersection);
			HroadTopX = HroadBorder.tl().x;
			HroadTopY = HroadBorder.tl().y;
			HroadBotX = HroadBorder.br().x;
			HroadBotY = HroadBorder.br().y;
			HROAD = Rect(HroadTopX, HroadTopY, (HroadBotX - HroadTopX), (HroadBotY - HroadTopY));//horizontal road dimensions

			if(intersection)
			{ 
			VroadBorder = roadDetectionVertical(image);
			VroadTopX = VroadBorder.tl().x;
			VroadTopY = VroadBorder.tl().y;
			VroadBotX = VroadBorder.br().x;
			VroadBotY = VroadBorder.br().y;
			VROAD = Rect(VroadTopX, VroadTopY, (VroadBotX - VroadTopX), (VroadBotY - VroadTopY));//vertical road dimensions
			}

			//horizontal
			halfY = (HroadBorder.br().y + HroadBorder.tl().y) / 2;
			HhalfFirstpoint = Point(HroadBorder.tl().x, halfY);
			HhalfSecondPoint = Point(HroadBorder.br().x, halfY);

			

			//vertical
			if (intersection)
			{
				int halfX = (VroadBorder.br().x + VroadBorder.tl().x) / 2;
				VhalfFirstpoint = Point(halfX, VroadBorder.tl().y);
				VhalfSecondPoint = Point(halfX, VroadBorder.br().y);

				
			}
		}

		//draw road
		//draw rectangular shape of result of road detection
		rectangle(image, HroadBorder.tl(), HroadBorder.br(), Scalar(255, 0, 0), 2, 8, 0);
		//line(image, HhalfFirstpoint, HhalfSecondPoint, Scalar(255, 0, 0), 2, 8, 0);
		if (intersection)
		{
			//draw rectangular shape of result of road detection
			rectangle(image, VroadBorder.tl(), VroadBorder.br(), Scalar(255, 0, 0), 2, 8, 0);
			//line(image, VhalfFirstpoint, VhalfSecondPoint, Scalar(255, 0, 0), 2, 8, 0);
		}

		image2detectionH.copyTo(curFrame);
		cvtColor(curFrame, curFrame, COLOR_BGR2GRAY);
		for (int i = 0; i<image2detectionH.size().width; i++)
		{
			for (int j = 0; j<image2detectionH.size().height; j++)
			{
				if (j < (HroadBorder.tl().y+10) || (j > HroadBorder.br().y))  // these threshold values must be tuned or determined automatically!
				{
					curFrame.at<char>(j, i) = 0;
				}
			}
		}
		curFrame.convertTo(curFrame, CV_64F, 1.0 / 255);
		curFrame.copyTo(image2detectionH);

		if (intersection)
		{
			image2detectionV.copyTo(curFrame);
			cvtColor(curFrame, curFrame, COLOR_BGR2GRAY);
			for (int j = 0; j<image2detectionV.size().height; j++)
			{
				for (int i = 0; i<image2detectionV.size().width; i++)
				{
					if ((i < VroadBorder.tl().x) || (i > VroadBorder.br().x))  // these threshold values must be tuned or determined automatically!
					{
						curFrame.at<char>(j, i) = 0;
					}
				}
			}
			curFrame.convertTo(curFrame, CV_64F, 1.0 / 255);
			curFrame.copyTo(image2detectionV);
			//imshow("image2detectionV", image2detectionV);
		}

		Mat prevFrame= Mat::zeros(curFrame.rows, curFrame.cols, CV_8U); //for LK
		//new vehicle detection
		if (framepos == 2 || framepos % 10 == 0)
		{
			sobelImageH = sobelDetection(image2detectionH);
			tempvehicles = vehicles;
			vehicles.clear();
			points[0].clear();
			points[1].clear();
			
			if (framepos != 1)
			{
				Mat u = Mat::zeros(prevFrame.rows, prevFrame.cols, CV_64FC1);
				Mat v = Mat::zeros(prevFrame.rows, prevFrame.cols, CV_64FC1);
			    //Mat LKImage = LKDetection(prevFrame, curFrame, u, v);
			   // Mat comboResultImage = Mat::zeros(curFrame.rows, curFrame.cols, CV_8U);
				//comboResultImage=LKImage+sobelImageH;
				//imshow("", comboResultImage);
				//search for contours in the image
				searchForVehicle(sobelImageH, vehicles);
			}

			 prevFrame = curFrame;
			
			

			//draw object location on screen
			//drawObject(vehicles, image);

			for (int i = 0; i < vehicles.size(); i++) {
				points[1].insert(points[1].end(), vehicles.at(i).getPoints(1).begin(), vehicles.at(i).getPoints(1).end());
			}

			if (intersection)
			{
				sobelImageV = sobelDetection(image2detectionV);
				tempvehiclesV = vehiclesV;
				vehiclesV.clear();
				pointsV[0].clear();
				pointsV[1].clear();
				//search for contours in the image
				//imshow("sobelImageV", sobelImageV);
				searchForVehicleV(sobelImageV, vehiclesV);

				//draw object location on screen
				//drawObject(vehicles, image);

				for (int i = 0; i < vehiclesV.size(); i++) {
					pointsV[1].insert(pointsV[1].end(), vehiclesV.at(i).getPoints(1).begin(), vehiclesV.at(i).getPoints(1).end());
				}
			}
		}

		//
		////vehicle detection
		//if (framepos == 1 || framepos % 8 == 0)
		//{
		//	//cv::Mat croppedImage(image.row,image.col,image.type);
		//	//cv::Mat destinationROI = croppedImage(HROAD);
		//	//image(HROAD).copyTo(destinationROI);
		//	//cv::imshow("croppedImage", croppedImage);
		//	cvtColor(image(HROAD), grayImage1, COLOR_BGR2GRAY);
		//	//pre-processing
		//	GaussianBlur(grayImage1, grayImage1, Size(15, 15), 0, 0, BORDER_DEFAULT);
		//	//sobel edge detection
		//	Sobel(grayImage1, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT);
		//	convertScaleAbs(grad_x, abs_grad_x);
		//	Sobel(grayImage1, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT);
		//	convertScaleAbs(grad_y, abs_grad_y);
		//	addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad);
		//	//cv::imshow("Sobel Image", grad);
		//	//threshold the edges
		//	threshold(grad, thresholdImage, 25, 255, THRESH_BINARY);
		//	//cv::imshow("Threshold Image", thresholdImage);
		//	//morph the image
		//	morphologyEx(thresholdImage, thresholdImage, MORPH_OPEN, Mat::ones(3, 3, CV_8SC1), Point(1, 1), 2);
		//	//imshow("Morphed Image", thresholdImage);

		//	vehicles.clear();
		//	points[0].clear();
		//	points[1].clear();
		//	//search for contours in the image
		//	searchForVehicle(thresholdImage, vehicles);

		//	//draw object location on screen
		//	//drawObject(vehicles, image);

		//	for (int i = 0; i < vehicles.size(); i++) {
		//		points[1].insert(points[1].end(), vehicles.at(i).getPoints(1).begin(), vehicles.at(i).getPoints(1).end());
		//	}

		//	//add track points
		//	
		//	//if (vehicles.size() > 0)
		//	//{
		//	//	for (int i = 0; i < vehicles.size(); i++) {
		//	//		Point2f centroid = Point2f(vehicles.at(i).getXPos() + roadTopX, vehicles.at(i).getYPos() + roadTopY);
		//	//		points[1].push_back(centroid);
		//	//	}
		//	//}
		//}
		
		//cv::putText(image, "Number of Tracked Vehicles: " + to_string(points[1].size()), cv::Point(800, 60), 1, 1, Scalar(0, 255, 0));
		//drawObject(vehicles, image);
	
		cvtColor(image, gray, COLOR_BGR2GRAY);

		//ref points
		if (addRefPt)
		{
			if (addRefPt == true)
			{
				for (int i = 0; i < 6; i++)
				{
					vector<Point2f> tmp;
					tmp.push_back(RP[i]);
					// Function to refine the location of the corners to subpixel accuracy.
					// Here, 'pixel' refers to the image patch of size 'windowSize' and not the actual image pixel
					cornerSubPix(gray, tmp, winSize, Size(-1, -1), termcrit);
					refPoints[1].push_back(tmp[0]);

				}
				addRefPt = false;
			}

			//add flow points
			if (!refPoints[0].empty())
			{
				vector<uchar> status;
				// Error vector to indicate the error for the corresponding feature
				vector<float> err;
				// Check if previous image is empty
				if (prevGray.empty())
					gray.copyTo(prevGray);

				calcOpticalFlowPyrLK(prevGray, gray, refPoints[0], refPoints[1], status, err, winSize, 3, termcrit, 0, 0.001);

				refAve = 0;
				size_t i, k;
				for (i = k = 0; i < refPoints[1].size(); i++)
				{
					// Check if the status vector is good
					if (!status[i])
						continue;

					refPoints[1][k++] = refPoints[1][i];
					// Draw a filled circle for each of the tracking points
					//cv::circle(image, refPoints[1][i], 3, Scalar(0, 0, 255), -1, 8);
					//Point2f p0 = Point2f(round(points[0][i].x), round(points[0][i].y));
					//Point2f p1 = Point2f(round(points[1][i].x), round(points[1][i].y));
					//line(image, p0, p1, Scalar(0, 0, 255), 2, 8, 0);
					double distance = refPoints[1][i].x - refPoints[0][i].x;
					//speed(in kph) = (distance in pixel/(1frames/48fps))*(3600s/hr)*(4.1m/38pixelx)*(1km/1000m)
					double speed = (distance / (1 / fps)) * 3600 * groundreso / 1000;
					refAve = refAve + speed;
					//cv::putText(image, to_string((int)speed) + " kph", refPoints[0][i], 1, 1, Scalar(0, 0, 255));
				}


				refAve = refAve / 6;
				//cv::putText(image, "Reference Average Speed: " + to_string(refAve), cv::Point(800, 60), 1, 1, Scalar(0, 255, 0));
				refPoints[1].resize(k);
			}

		}

		//vehicle tracking
		if (true)
		{
			if (!points[0].empty())
			{

				// Status vector to indicate whether the flow for the corresponding features has been found
				vector<uchar> status;
				// Error vector to indicate the error for the corresponding feature
				vector<float> err;
				// Check if previous image is empty
				if (prevGray.empty())
					gray.copyTo(prevGray);

				// Calculate the optical flow using Lucas-Kanade algorithm
				calcOpticalFlowPyrLK(prevGray, gray, points[0], points[1], status, err, winSize, 3, termcrit, 0, 0.001);

				size_t i, k;
				rightTotal = rightCounter = leftTotal = leftCounter = 0;
				for (i = k = 0; i < points[1].size(); i++)
				{
					// Check if the status vector is good
					if (!status[i])
						continue;

					points[1][k++] = points[1][i];
					// Draw a filled circle for each of the tracking points
					//cv::circle(image, points[1][i], 3, Scalar(0, 255, 0), -1, 8);
					//Point2f p0 = Point2f(round(points[0][i].x), round(points[0][i].y));
					//Point2f p1 = Point2f(round(points[1][i].x), round(points[1][i].y));
					//line(image, p0, p1, Scalar(0, 0, 255), 2, 8, 0);
					double distance = points[1][i].x - points[0][i].x;
					//speed(in kph) = (distance in pixel/(1frames/48fps))*(3600s/hr)*(4.1m/38pixelx)*(1km/1000m)
					double speed = (distance / (1 / fps)) * 3600 * groundreso / 1000;
					speed = speed - refAve;
					if (speed > 0)
					{
						rightTotal = rightTotal + speed;
						rightCounter++;
					}
					else
					{
						leftTotal = leftTotal + speed;
						leftCounter++;
					}

					vehicles.at(i).setXPos(points[0][i].x);
					vehicles.at(i).setYPos(points[0][i].y);
					vehicles.at(i).setTl(Point(points[0][i].x - vehicles.at(i).getC2BX(), points[0][i].y - vehicles.at(i).getC2BY()));
					vehicles.at(i).setBr(Point(points[0][i].x + vehicles.at(i).getC2BX(), points[0][i].y + vehicles.at(i).getC2BY()));
					vehicles.at(i).setSpeed(std::abs(speed));
					//cv::putText(image, to_string((int)std::abs(speed)) + " kph", Point(points[0][i].x - 20, points[0][i].y+5), 1, 1, speedSpectrum(std::abs(speed)));
				}

				drawObject(vehicles, image);
				
				points[1].resize(k);
			}
			else
			{
				drawObject(tempvehicles, image);
			}

			if (intersection)
			{
				if (!pointsV[0].empty())
				{

					// Status vector to indicate whether the flow for the corresponding features has been found
					vector<uchar> status;
					// Error vector to indicate the error for the corresponding feature
					vector<float> err;
					// Check if previous image is empty
					if (prevGray.empty())
						gray.copyTo(prevGray);

					// Calculate the optical flow using Lucas-Kanade algorithm
					calcOpticalFlowPyrLK(prevGray, gray, pointsV[0], pointsV[1], status, err, winSize, 3, termcrit, 0, 0.001);

					size_t i, k;
					upTotal = upCounter = downTotal = downCounter = 0;
					for (i = k = 0; i < pointsV[1].size(); i++)
					{
						// Check if the status vector is good
						if (!status[i])
							continue;

						pointsV[1][k++] = pointsV[1][i];
						// Draw a filled circle for each of the tracking points
						//cv::circle(image, pointsV[1][i], 3, Scalar(0, 255, 0), -1, 8);
						//Point2f p0 = Point2f(round(points[0][i].x), round(points[0][i].y));
						//Point2f p1 = Point2f(round(points[1][i].x), round(points[1][i].y));
						//line(image, p0, p1, Scalar(0, 0, 255), 2, 8, 0);
						double distance = pointsV[1][i].y - pointsV[0][i].y;
						//speed(in kph) = (distance in pixel/(1frames/48fps))*(3600s/hr)*(4.1m/38pixelx)*(1km/1000m)
						double speed = (distance / (1 / fps)) * 3600 * groundreso / 1000;
						speed = speed - refAve;
						if (speed > 0)
						{
							upTotal = upTotal + speed;
							upCounter++;
						}
						else
						{
							downTotal = downTotal + speed;
							downCounter++;
						}

						vehiclesV.at(i).setXPos(pointsV[0][i].x);
						vehiclesV.at(i).setYPos(pointsV[0][i].y);
						vehiclesV.at(i).setTl(Point(pointsV[0][i].x - vehiclesV.at(i).getC2BX(), pointsV[0][i].y - vehiclesV.at(i).getC2BY()));
						vehiclesV.at(i).setBr(Point(pointsV[0][i].x + vehiclesV.at(i).getC2BX(), pointsV[0][i].y + vehiclesV.at(i).getC2BY()));
						vehiclesV.at(i).setSpeed(std::abs(speed));
						//cv::putText(image, to_string((int)std::abs(speed)) + " kph", Point(points[0][i].x - 20, points[0][i].y+5), 1, 1, speedSpectrum(std::abs(speed)));
					}

					drawObject(vehiclesV, image);
					pointsV[1].resize(k);
				}
				else
				{
					drawObject(tempvehiclesV, image);
				}
			}

			//ORIGNAL TRACKING CODE UNHIDE TO C
			/******************ORIGINAL TRACKING CODE**************
			// Check if there are points to track
			if (!points[0].empty())
			{
				// Status vector to indicate whether the flow for the corresponding features has been found
				vector<uchar> status;
				// Error vector to indicate the error for the corresponding feature
				vector<float> err;
				// Check if previous image is empty
				if (prevGray.empty())
					gray.copyTo(prevGray);

				// Calculate the optical flow using Lucas-Kanade algorithm
				calcOpticalFlowPyrLK(prevGray, gray, points[0], points[1], status, err, winSize, 3, termcrit, 0, 0.001);

				size_t i, k;
				rightTotal = rightCounter = leftTotal = leftCounter = 0;
				for (i = k = 0; i < points[1].size(); i++)
				{
					if (addRemovePt)
					{
						// If the new point is within 'minDist' distance from an existing point, it will not be tracked
						if (norm(point - points[1][i]) <= minDist)
						{
							addRemovePt = false;
							continue;
						}
					}
					// Check if the status vector is good
					if (!status[i])
						continue;

					points[1][k++] = points[1][i];
					// Draw a filled circle for each of the tracking points
					circle(image, points[1][i], 3, Scalar(0, 255, 0), -1, 8);
					//Point2f p0 = Point2f(round(points[0][i].x), round(points[0][i].y));
					//Point2f p1 = Point2f(round(points[1][i].x), round(points[1][i].y));
					//line(image, p0, p1, Scalar(0, 0, 255), 2, 8, 0);
					double distance = points[1][i].x - points[0][i].x;
					//speed(in kph) = (distance in pixel/(1frames/48fps))*(3600s/hr)*(4.1m/38pixelx)*(1km/1000m)
					double speed = (distance / (1 / fps)) * 3600 * groundreso / 1000;
					if (speed > 0)
					{
						rightTotal = rightTotal + speed;
						rightCounter++;
					}
					else
					{
						leftTotal = leftTotal + speed;
						leftCounter++;
					}
					putText(image, to_string((int)std::abs(speed)) + " kph", points[0][i], 1, 1, speedSpectrum(std::abs(speed)));

				}
				points[1].resize(k);
			}
			


			// Refining the location of the feature points
			if (addRemovePt && points[1].size() < (size_t)MAX_COUNT)
			{
				vector<Point2f> tmp;
				tmp.push_back(point);
				// Function to refine the location of the corners to subpixel accuracy.
				// Here, 'pixel' refers to the image patch of size 'windowSize' and not the actual image pixel
				//cornerSubPix(gray, tmp, winSize, Size(-1, -1), termcrit);
				points[1].push_back(tmp[0]);
				addRemovePt = false;
			}
			*/
		}


		

		//average speed computation
		if (framepos % ((int)fps/2) == 0 || framepos == 5 )
		{ 
			//vehicle counting
			Point2i TopAndBotNumOfCars = vehicleCountTopAndBot(vehicles, halfY);
			TopNumberOfCars = TopAndBotNumOfCars.x;
			BotNumberOfCars = TopAndBotNumOfCars.y;
			numberOfVehicles = vehicles.size();
			numberOfVehiclesV = vehiclesV.size();

			//vehicle density computation
			int HRoadLengthInPixel = HroadBotX - HroadTopX;
			double HRoadLength = HRoadLengthInPixel * groundreso;
			TopRoadDensity = rightCounter / HRoadLength;
			BotRoadDensity = leftCounter / HRoadLength;

			//for intersection
			if (intersection)
			{
				int VRoadLengthInPixel = VroadBotY - VroadTopY;
				VRoadLength = VRoadLengthInPixel * groundreso;
				LeftRoadDensity = upCounter / VRoadLength;
				RightRoadDensity = downCounter / VRoadLength;
			}

			//average speed computation
			rightAve = (rightCounter > 0) ? std::abs(rightTotal / rightCounter) : 0;
			leftAve = (leftCounter > 0) ? std::abs(leftTotal / leftCounter) : 0;

			if (intersection)
			{
				upAve = (upCounter > 0) ? std::abs(upTotal / upCounter) : 0;
				downAve = (downCounter > 0) ? std::abs(downTotal / downCounter) : 0;
			}


		}

		//display status
		setLabel(image, "H-Road Number of Vehicles: " + intToString(numberOfVehicles), cv::Point(20, 20));
		setLabel(image, topRoadName + " No. of Vehicles: " + intToString(rightCounter), cv::Point(20, 40));
		setLabel(image, botRoadName + " No. of Vehicles: " + intToString(leftCounter), cv::Point(20, 60));
		setLabel(image, "Frame : " + intToString(framepos) + " / " + intToString(totalframes), cv::Point(20, 680));
		//setLabel(image, "H-Road Length(m): " + to_string(HRoadLength), cv::Point(800, 20));
		setLabel(image, topRoadName + " Vehicle Density: " + to_string(TopRoadDensity), cv::Point(800, 20));
		setLabel(image, botRoadName + " Vehicle Density: " + to_string(BotRoadDensity), cv::Point(800, 40));
		setLabel(image, topRoadName + " Ave. Speed: " + to_string(rightAve), cv::Point(800, 60));
		setLabel(image, botRoadName + " Ave. Speed: " + to_string(leftAve), cv::Point(800, 80));

		if (intersection)
		{
			setLabel(image, "V-Road Number of Vehicles: " + intToString(numberOfVehiclesV), cv::Point(20, 100));
			setLabel(image, leftRoadName + " No. of Vehicles: " + intToString(upCounter), cv::Point(20, 120));
			setLabel(image, rightRoadName + " No. of Vehicles: " + intToString(downCounter), cv::Point(20, 140));

			//setLabel(image, "V-Road Lengt(m): " + to_string(VRoadLength), cv::Point(800, 140));
			setLabel(image, leftRoadName + " Vehicle Density: " + to_string(LeftRoadDensity), cv::Point(800, 140));
			setLabel(image, rightRoadName + " Vehicle Density: " + to_string(RightRoadDensity), cv::Point(800, 160));
			setLabel(image, leftRoadName + " Ave. Speed: " + to_string(upAve), cv::Point(800, 180));
			setLabel(image, rightRoadName + " Ave. Speed: " + to_string(downAve), cv::Point(800, 200));
		}

		//compass
		//draw white background
		cv::rectangle(image, cv::Point((dWidth)-120, (dHeight)-120), cv::Point(dWidth-1, dHeight-1), CV_RGB(255, 255, 255), CV_FILLED);
		//draw arrow
		cv::arrowedLine(image, center + direction*0.5, center + direction, CV_RGB(255, 0, 0), 2, 8, 0, tipLength); // draw arrow!
		//put text "N"
		cv::putText(image, "N", Point(1219, 665), cv::FONT_HERSHEY_TRIPLEX, 0.6, CV_RGB(255, 0, 0), 1, 8);
		//cv::putText(image, "Number of Vehicles: " + intToString(numberOfVehicles), cv::Point(20, 20), 1, 1, Scalar(0, 255, 0));
		//cv::putText(image, "Frame No: " + intToString(framepos), cv::Point(20, 40), 1, 1, Scalar(0, 255, 0));
		//cv::putText(image, "Total Frames: " + intToString(totalframes), cv::Point(20, 60), 1, 1, Scalar(0, 255, 0));
		//cv::putText(image, "Going Right Ave. Speed: " + to_string(rightAve), cv::Point(800, 20), 1, 1, Scalar(0, 255, 0));
		//cv::putText(image, "Going Left Ave. Speed: " + to_string(leftAve), cv::Point(800, 40), 1, 1, Scalar(0, 255, 0));

		//to Logs
		if (framepos % (int)fps == 0)
		{
			int seconds = (framepos / (int)fps);
			int hr, min, sec = 0;
			secondsToTime(seconds, hr, min, sec);
			//std::cout << hr << ":" << min << ":" << sec << endl;
			//fprintf(pFile, "\n %d \t\t\t %d \t\t %lf \t\t %lf", (framepos / (int)fps), numberOfVehicles, rightAve, leftAve);
			myfile << "\n" << hr << ":" << min << ":" << sec << ",";
			myfile << intToString(numberOfVehicles) << ",";
			myfile << intToString(rightCounter) << ",";
			myfile << intToString(leftCounter) << ",";
			if (intersection)
			{
				myfile << intToString(numberOfVehiclesV) << ",";
				myfile << intToString(upCounter) << ",";
				myfile << intToString(downCounter) << ",";
			}
			myfile << to_string(TopRoadDensity) << ",";
			myfile << to_string(BotRoadDensity) << ",";
			if (intersection)
			{
				myfile << to_string(LeftRoadDensity) << ",";
				myfile << to_string(RightRoadDensity) << ",";
			}
			myfile << to_string(rightAve) << ",";
			myfile << to_string(leftAve) << ",";
			if (intersection)
			{
				myfile << to_string(upAve) << ",";
				myfile << to_string(downAve) << ",";
			}
		}
		//result
		//cv::imshow("Output", image);
		double progress = ((double) framepos / totalframes) * 100;
		//cout << "Frame: " << framepos << "/" << totalframes << " -- " << (int)progress << "%" << endl;
		cout << (int)progress << endl;
		oVideoWriter.write(image);

		

		char c = (char)waitKey(10);
		if (c == 27)
			break;
		switch (c)
		{
		case 'c':
			points[0].clear();
			points[1].clear();
			break;
		case 'p': //'p' has been pressed. this will pause/resume the code.
			pause = !pause;
			if (pause == true)
			{
				std::cout << "Code paused, press 'p' again to resume" << endl;
				while (pause == true)
				{
					//stay in this loop until 
					switch (waitKey())
					{
						//a switch statement inside a switch statement? Mind blown.
					case 112:
						//change pause back to false
						pause = false;
						std::cout << "Code Resumed" << endl;
						break;
					case 115:
						//screen capture
						imwrite("D:/capturedimage1.jpg", image);
						std::cout << "Capture Image" << endl;
						break;
					case 27:
						return 0;
					}
				}
			}
		}

		std::swap(points[1], points[0]);
		std::swap(pointsV[1], pointsV[0]);
		std::swap(refPoints[1], refPoints[0]);
		cv::swap(prevGray, gray);

		
	}
	myfile.close();
	//fclose(pFile);
	//cout << "Processing Done!";
	//waitKey(0);
	return 0;
}


void secondsToTime(int seconds, int &hr, int &min, int &sec)
{
	int hours, minutes;
	minutes = seconds / 60;
	hours = minutes / 60;
	hr = (int)hours;
	min = int(minutes % 60);
	sec = int(seconds % 60);
}
void addPoints(int x, int y)
{
	point = Point2f((float)x, (float)y);
	addRemovePt = true;
}

string intToString(int number)
{
	//this function has a number input and string output
	std::stringstream ss;
	ss << number;
	return ss.str();
}

void drawObject(vector<Vehicle> VehicleCars, Mat &frame)
{

	for (int i = 0; i<VehicleCars.size(); i++) {
		//cv::circle(frame,cv::Point(VehicleCars.at(i).getXPos(),VehicleCars.at(i).getYPos()),20,cv::Scalar(0,255,0));
		//rectangle(frame, VehicleCars.at(i).getTl(), VehicleCars.at(i).getBr(), cv::Scalar(0, 255, 0), 1, 8, 0);//REAL
		rectangle(frame,VehicleCars.at(i).getTl(),VehicleCars.at(i).getBr(), speedSpectrum(VehicleCars.at(i).getSpeed()), 2, 8, 0);
		//Tl = top-left point, Br = Bottom-right point
		//location of the object
		//line(frame,cv::Point(VehicleCars.at(i).getXPos(),VehicleCars.at(i).getYPos()),cv::Point(VehicleCars.at(i).getXPos()+10,VehicleCars.at(i).getYPos()),cv::Scalar(0,255,0),2,8,0);
		//cv::putText(frame,intToString(VehicleCars.at(i).getXPos())+ " , " + intToString(VehicleCars.at(i).getYPos()),cv::Point(VehicleCars.at(i).getXPos(),VehicleCars.at(i).getYPos()+20),1,1,Scalar(0,255,0));
		//addPoints(VehicleCars.at(i).getXPos(), VehicleCars.at(i).getYPos());
		//cv::putText(frame, intToString(VehicleCars.at(i).getID()), VehicleCars.at(i).getBr(), 1, 1, Scalar(0, 255, 0));
	}
	
	//cv::putText(frame,"Number of Vehicles: "+intToString(VehicleCars.size()),cv::Point(20,20),1,1,Scalar(0,255,0));
	//printf("Number of Vehicles : %d",VehicleCars.size());
}

void searchForVehicle(Mat thresholdImage, vector<Vehicle> &vehicles) {

	
	int ID = 0;
	//eg. we draw to the cameraFeed to be displayed in the main() function.
	bool objectDetected = false;
	Mat temp;
	thresholdImage.copyTo(temp);
	//these two vectors needed for output of findContours
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	//find contours of filtered image using openCV findContours function
	//findContours(temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE );// retrieves all contours
	//findContours(temp,contours,hierarchy,RETR_TREE,CV_CHAIN_APPROX_SIMPLE );// retrieves all contours tree
	findContours(temp, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);// retrieves external contours
																					  // remove very small contours
	vector<vector<Point> > contours_poly(contours.size());
	vector<Rect> boundRect(contours.size());

	//if contours vector is not empty, we have found some objects
	double refArea = 0;
	if (hierarchy.size() > 0) {

		for (int index = 0; index >= 0; index = hierarchy[index][0]) {

			//cout << "\n Countour area: " << contourArea(contours[index]);

			Moments moment = moments((cv::Mat)contours[index]);
			double area = moment.m00;

			//if the area is less than 20 px by 20px then it is probably just noise
			//if the area is the same as the 3/2 of the image size, probably just a bad filter
			//we only want the object with the largest area so we safe a reference area each
			//iteration and compare it to the area in the next iteration.
			if (area>MIN_OBJECT_AREA) {
				

				approxPolyDP(Mat(contours[index]), contours_poly[index], 3, true);
				boundRect[index] = boundingRect(Mat(contours_poly[index]));

				int width = boundRect[index].width;
				int height = boundRect[index].height;
				
				//std::cout << "ID: " << ID;
				//std::cout << " Width: " << width;
				//std::cout << " Height: " << height << endl;
				
				
				
				//if (true)
				//if (width>15 && width <120 && height>15 && height<50)
				if ((width>30 && width<120) && (height>20 && height<45))
				{
					Vehicle car;
					
					int xPos = moment.m10 / area;
					int yPos = moment.m01 / area;
					//Point bottomRight = Point(boundRect[index].br().x + HroadTopX, boundRect[index].br().y +HroadTopY);
					Point bottomRight = Point(boundRect[index].br().x, boundRect[index].br().y);
					//Point topLeft = Point(boundRect[index].tl().x + HroadTopX, boundRect[index].tl().y + HroadTopY);
					Point topLeft = Point(boundRect[index].tl().x, boundRect[index].tl().y);

					car.setXPos(xPos);
					car.setYPos(yPos);
					car.setID(ID);
					car.setBr(bottomRight);
					car.setTl(topLeft);
					//car.appendPoints(Point2f(xPos + HroadTopX, yPos + HroadTopY),1);
					car.appendPoints(Point2f(xPos, yPos), 1);
					car.setC2BX(xPos- boundRect[index].tl().x);
					car.setC2BY(yPos - boundRect[index].tl().y);
					car.setSpeed(0);
					vehicles.push_back(car);
					ID++;
				}
			}
		}
	}

}
Mat get_fx(Mat &src1, Mat &src2)
{
	Mat fx;
	Mat kernel = Mat::ones(2, 2, CV_64FC1);
	//Mat kernel= Mat::ones(2,2,CV_8U);
	kernel.ATD(0, 0) = -1.0;
	kernel.ATD(1, 0) = -1.0;


	Mat dst1, dst2;
	filter2D(src1, dst1, -1, kernel);
	filter2D(src2, dst2, -1, kernel);

	fx = dst1 + dst2;
	return fx;

}
Mat get_fy(Mat &src1, Mat &src2) {
	Mat fy;
	Mat kernel = Mat::ones(2, 2, CV_64FC1);
	//Mat kernel = Mat::ones(2, 2, CV_8U);
	kernel.ATD(0, 0) = -1.0;
	kernel.ATD(0, 1) = -1.0;

	Mat dst1, dst2;
	filter2D(src1, dst1, -1, kernel);
	filter2D(src2, dst2, -1, kernel);

	fy = dst1 + dst2;

	//imshow("fy dst1",dst1);
	//imshow("fy",fy);
	return fy;
}

Mat get_ft(Mat &src1, Mat &src2) {
	Mat ft;
	Mat kernel = Mat::ones(2, 2, CV_64FC1);
	//Mat kernel = Mat::ones(2, 2, CV_8U);
	kernel = kernel.mul(-1);

	Mat dst1, dst2;
	filter2D(src1, dst1, -1, kernel);
	kernel = kernel.mul(-1);
	filter2D(src2, dst2, -1, kernel);

	ft = dst1 + dst2;
	//imshow("ft dst1",dst1);
	//imshow("ft dst2",dst2);
	//imshow("ft",ft);
	return ft;
}

bool isInsideImage(int y, int x, Mat &m) {
	int width = m.cols;
	int height = m.rows;
	if (x >= 0 && x < width && y >= 0 && y < height) return true;
	else return false;
}
double get_Sum9(Mat &m, int y, int x)
{
	if (x < 0 || x >= m.cols) return 0;
	if (y < 0 || y >= m.rows) return 0;

	double val = 0.0;
	int tmp = 0;
	if (isInsideImage(y - 1, x - 1, m))
	{
		++tmp;
		val += m.ATD(y - 1, x - 1);
	}
	if (isInsideImage(y - 1, x, m)) {
		++tmp;
		val += m.ATD(y - 1, x);
	}
	if (isInsideImage(y - 1, x + 1, m)) {
		++tmp;
		val += m.ATD(y - 1, x + 1);
	}
	if (isInsideImage(y, x - 1, m)) {
		++tmp;
		val += m.ATD(y, x - 1);
	}
	if (isInsideImage(y, x, m)) {
		++tmp;
		val += m.ATD(y, x);
	}
	if (isInsideImage(y, x + 1, m)) {
		++tmp;
		val += m.ATD(y, x + 1);
	}
	if (isInsideImage(y + 1, x - 1, m)) {
		++tmp;
		val += m.ATD(y + 1, x - 1);
	}
	if (isInsideImage(y + 1, x, m)) {
		++tmp;
		val += m.ATD(y + 1, x);
	}
	if (isInsideImage(y + 1, x + 1, m)) {
		++tmp;
		val += m.ATD(y + 1, x + 1);
	}
	if (tmp == 9) return val;
	else return m.ATD(y, x) * 9;
}
Mat get_Sum9_Mat(Mat &m) {
	Mat res = Mat::zeros(m.rows, m.cols, CV_64FC1);
	//Mat res = Mat::zeros(m.rows, m.cols, CV_8U);
	for (int i = 1; i < m.rows - 1; i++) {
		for (int j = 1; j < m.cols - 1; j++) {
			res.ATD(i, j) = get_Sum9(m, i, j);
		}
	}
	return res;
}
Mat getLucasKanadeOpticalFlow(Mat &img1, Mat &img2, Mat u, Mat v) {

	Mat fx = get_fx(img1, img2);
	Mat ft = get_ft(img1, img2);
	Mat fy = get_fy(img1, img2);

	Mat fx2 = fx.mul(fx);
	Mat fy2 = fy.mul(fy);
	Mat fxfy = fx.mul(fy);
	Mat fxft = fx.mul(ft);
	//imshow("fxft",fxft);
	Mat fyft = fy.mul(ft);

	Mat sumfx2 = get_Sum9_Mat(fx2);
    Mat sumfy2 = get_Sum9_Mat(fy2);
	//imshow("sumfy2",sumfy2);
	Mat sumfxft = get_Sum9_Mat(fxft);
	Mat sumfxfy = get_Sum9_Mat(fxfy);
	//imshow("sumfxfy",sumfxfy);
	Mat sumfyft = get_Sum9_Mat(fyft);
	
	Mat tmp = sumfx2.mul(sumfy2) - sumfxfy.mul(sumfxfy);
	//imshow("tmp",tmp);
    u = sumfxfy.mul(sumfyft) - sumfy2.mul(sumfxft);
	v = sumfxft.mul(sumfxfy) - sumfx2.mul(sumfyft);
	
	divide(u, tmp, u);
	divide(v, tmp, v);
	//imshow("u",u);
	//imshow("v",v);
	return sumfx2;
}
Mat LKDetection(Mat prevFrame, Mat curFrame, Mat u, Mat v)
{
	int i;
	Mat thresholdImage;
	Mat frameCurOrig;
	Size img_sz = curFrame.size();
	frameCurOrig = curFrame.clone();
	//imshow("curFrame",curFrame);
	//Mat u = Mat::zeros(prevFrame.rows, prevFrame.cols, CV_64FC1);
	//Mat v = Mat::zeros(prevFrame.rows, prevFrame.cols, CV_64FC1);
	//Mat u = Mat::zeros(prevFrame.rows, prevFrame.cols, CV_8U);
	//Mat v = Mat::zeros(prevFrame.rows, prevFrame.cols, CV_8U);

	/***************LK**********************/
	Mat grayLK;
	double  minVal, maxVal;
	curFrame.convertTo(curFrame, CV_64F, 1.0 / 255);
	prevFrame.convertTo(prevFrame, CV_64F, 1.0 / 255);
	GaussianBlur(prevFrame, prevFrame, Size(3, 3), 0, 0, BORDER_DEFAULT);
	GaussianBlur(curFrame, curFrame, Size(3, 3), 0, 0, BORDER_DEFAULT);
	//imshow("curFrame", curFrame);
	Mat LKResultImage = getLucasKanadeOpticalFlow(prevFrame, curFrame, u, v);
	minMaxLoc(LKResultImage,  &minVal,  &maxVal);  //find  minimum  and  maximum  intensities
	LKResultImage.convertTo(grayLK,  CV_8U,  255.0/(maxVal  -  minVal),  -minVal);
	cv::threshold(grayLK, thresholdImage, 18, 255, THRESH_BINARY);
	return thresholdImage;
}

void searchForVehicleV(Mat thresholdImage, vector<Vehicle> &vehicles) {


	int ID = 0;
	//eg. we draw to the cameraFeed to be displayed in the main() function.
	bool objectDetected = false;
	Mat temp;
	thresholdImage.copyTo(temp);
	//these two vectors needed for output of findContours
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	//find contours of filtered image using openCV findContours function
	//findContours(temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE );// retrieves all contours
	//findContours(temp,contours,hierarchy,RETR_TREE,CV_CHAIN_APPROX_SIMPLE );// retrieves all contours tree
	findContours(temp, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);// retrieves external contours
																					  // remove very small contours
	vector<vector<Point> > contours_poly(contours.size());
	vector<Rect> boundRect(contours.size());

	//if contours vector is not empty, we have found some objects
	double refArea = 0;
	if (hierarchy.size() > 0) {

		for (int index = 0; index >= 0; index = hierarchy[index][0]) {

			//cout << "\n Countour area: " << contourArea(contours[index]);

			Moments moment = moments((cv::Mat)contours[index]);
			double area = moment.m00;

			//if the area is less than 20 px by 20px then it is probably just noise
			//if the area is the same as the 3/2 of the image size, probably just a bad filter
			//we only want the object with the largest area so we safe a reference area each
			//iteration and compare it to the area in the next iteration.
			if (area>MIN_OBJECT_AREA) {


				approxPolyDP(Mat(contours[index]), contours_poly[index], 3, true);
				boundRect[index] = boundingRect(Mat(contours_poly[index]));

				int width = boundRect[index].width;
				int height = boundRect[index].height;

				//std::cout << "ID: " << ID;
				//std::cout << " Width: " << width;
				//std::cout << " Height: " << height << endl;



				//if (true)
				//if (width>15 && width <120 && height>15 && height<50)
				if ((height>30 && height<120) && (width>20 && width<45))
				{
					Vehicle car;

					int xPos = moment.m10 / area;
					int yPos = moment.m01 / area;
					//Point bottomRight = Point(boundRect[index].br().x + HroadTopX, boundRect[index].br().y +HroadTopY);
					Point bottomRight = Point(boundRect[index].br().x, boundRect[index].br().y);
					//Point topLeft = Point(boundRect[index].tl().x + HroadTopX, boundRect[index].tl().y + HroadTopY);
					Point topLeft = Point(boundRect[index].tl().x, boundRect[index].tl().y);

					car.setXPos(xPos);
					car.setYPos(yPos);
					car.setID(ID);
					car.setBr(bottomRight);
					car.setTl(topLeft);
					//car.appendPoints(Point2f(xPos + HroadTopX, yPos + HroadTopY),1);
					car.appendPoints(Point2f(xPos, yPos), 1);
					car.setC2BX(xPos - boundRect[index].tl().x);
					car.setC2BY(yPos - boundRect[index].tl().y);
					car.setSpeed(0);
					vehicles.push_back(car);
					ID++;
				}
			}
		}
	}

}

Mat labThresholdingIntersection(Mat labRoadImage)
{
	Mat im(labRoadImage.size().height, labRoadImage.size().width, CV_8UC1, Scalar(0));
	for (int x = 0; x<labRoadImage.size().width; x++)
	{
		for (int y = 0; y<labRoadImage.size().height; y++)
		{
			//if ((labRoadImage.at<Vec3b>(y, x)[1]>118) && (labRoadImage.at<Vec3b>(y, x)[1]<135))
			//if ((labRoadImage.at<Vec3b>(y, x)[1]>122) && (labRoadImage.at<Vec3b>(y, x)[1]<131))
			if ((labRoadImage.at<Vec3b>(y, x)[1]>125) && (labRoadImage.at<Vec3b>(y, x)[1]<131))  // these threshold values must be tuned or determined automatically!
			{
				if ((labRoadImage.at<Vec3b>(y, x)[2]>120) && (labRoadImage.at<Vec3b>(y, x)[2]<128)) //these threshold values must be tuned or determined automatically!
				//if ((labRoadImage.at<Vec3b>(y, x)[2]>118) && (labRoadImage.at<Vec3b>(y, x)[2]<135))
				//if ((labRoadImage.at<Vec3b>(y, x)[2]>118) && (labRoadImage.at<Vec3b>(y, x)[2]<129))
				{
					//changing the pixel intensity to white
					im.at<uchar>(y, x) = 255;
				}
			}
		}
	}
	return im;
}

Mat labThresholdingStraight(Mat labRoadImage)
{
	int x, y;
	Mat im(labRoadImage.size().height, labRoadImage.size().width, CV_8UC1, Scalar(0));
	for (x = 0; x<labRoadImage.size().width; x++)
	{
		for (y = 0; y<labRoadImage.size().height; y++)
		{
			//if ((labRoadImage.at<Vec3b>(y, x)[1]>122) && (labRoadImage.at<Vec3b>(y, x)[1]<131))  // straight these threshold values must be tuned or determined automatically!
			//{
				//if ((labRoadImage.at<Vec3b>(y, x)[2]>118) && (labRoadImage.at<Vec3b>(y, x)[2]<129)) //straight these threshold values must be tuned or determined automatically!
				//{
			if ((labRoadImage.at<Vec3b>(y, x)[1]>125) && (labRoadImage.at<Vec3b>(y, x)[1]<131))  // these threshold values must be tuned or determined automatically!
			{
				if ((labRoadImage.at<Vec3b>(y, x)[2]>120) && (labRoadImage.at<Vec3b>(y, x)[2]<128)) //these threshold values must be tuned or determined automatically!
					//changing the pixel intensity to white
					im.at<uchar>(y, x) = 255;
				}
			
		}
	}
	return im;

}

Rect roadDetectionHorizontal(Mat roadImage, int intersection)
{
	int x, y;
	double area;
	Rect bounding_rect;
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;

	Mat dst;
	Mat labRoadImage;
	Mat im(roadImage.size().height, roadImage.size().width, CV_8UC1, Scalar(0));
	Mat dstImg(roadImage.size().height, roadImage.size().width, CV_8UC1, Scalar(0));

	Mat origFrame = roadImage.clone();
	Mat gradImage;
	Mat roadImageBlur;
	GaussianBlur(roadImage, roadImageBlur, Size(21, 21), 0, 0, BORDER_DEFAULT);
	/// Total Gradient (approximate)
	//addWeighted(roadImage, 1, roadBlurImage, 1, 0, gradImage);

	//imshow( "Road Image Sobel", roadImage );
	cvtColor(roadImageBlur, labRoadImage, COLOR_BGR2Lab);
	//imshow("lab uimage", labRoadImage);

	if (intersection == 0)
	{
		im = labThresholdingStraight(labRoadImage);
	}
	else if (intersection == 1)
	{
		im = labThresholdingIntersection(labRoadImage);
	}

	//imshow("threshold", im);
	// Create a structuring element (SE)
	int morph_size = 2;
	Mat element = getStructuringElement(MORPH_RECT, Size(2 * morph_size + 1, 2 * morph_size + 1), Point(morph_size, morph_size));

	for (int i = 0; i<2; i++)
	{
		morphologyEx(im, im, 2, element, Point(-1, -1), i);
	}

	//Canny(im,im,50,150);
	//imshow("Canny Image", im);
	//imshow("Image", im);
	int counter = 0;
	int arrayPercentH[1280];//only 720 height is needed
							//int arrayPercentV[720];
	float percentage;
	x = 0;
	//FILE *f;
	//f = fopen("frequency.txt", "w");
	/*****for horizontal*****/

	for (int y = 0; y<im.size().height; y++)
	{
		for (x = 0, counter = 0; x<im.size().width; x++)
		{
			if (im.at<uchar>(y, x) == 255)  // these threshold values must be tuned or determined automatically!
			{
				counter++;
			}
		}
		percentage = (float)counter / im.size().width;
		//printf("y:%d, percentage: %f\n",y,percentage);
		//fprintf(f, " %d, %f\n", y, percentage);

		if (percentage >= 0.60)
		{
			arrayPercentH[y] = 255;
		}
		else
		{
			arrayPercentH[y] = 0;
		}


		//imshow("im",im);

	}
	/*****for horizontal*****/


	/***for horizontal**/
	for (int y = 0; y<im.size().height; y++)
	{
		for (x = 0; x<im.size().width; x++)
		{
			im.at<uchar>(y, x) = arrayPercentH[y];  // these threshold values must be tuned or determined automatically!
		}
	}
	/***for horizontal**/


	//imshow("bin",im);

	int dilate_size = 2;
	Mat dilateElement = getStructuringElement(cv::MORPH_RECT, Size(2 * dilate_size + 1, 2 * dilate_size + 1), Point(dilate_size, dilate_size));

	for (int i = 0; i<6; i++)
	{
		//Apply erosion or dilation on the image
		dilate(im, im, dilateElement);
	}
	//imshow("Result Road Dilate",im);

	double largest_area = 0;
	double a;
	int largest_contour_index;


	findContours(im, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
	for (int i = 0; i< contours.size(); i++)
	{
		a = contourArea(contours[i], false);  //  Find the area of contour
		if (a>largest_area)
		{
			largest_area = a;
			largest_contour_index = i;                //Store the index of largest contour
			bounding_rect = boundingRect(contours[i]); // Find the bounding rectangle for biggest contour
		}
	}
	//drawContours(origFrame, contours, largest_contour_index, Scalar(255), 1, 8, hierarchy);
	//imshow("largest area horizontal", origFrame);
	return bounding_rect;
}

Rect roadDetectionVertical(Mat roadImage)
{
	int x, y;
	double area;
	Rect bounding_rect;
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	Mat origFrame = roadImage.clone();
	//imshow("origFrame", origFrame);
	Mat dst;
	Mat labRoadImage;
	Mat im(roadImage.size().height, roadImage.size().width, CV_8UC1, Scalar(0));
	Mat dstImg(roadImage.size().height, roadImage.size().width, CV_8UC1, Scalar(0));

	Mat roadImageBlur;
	GaussianBlur(roadImage, roadImageBlur, Size(21, 21), 0, 0, BORDER_DEFAULT);

	cvtColor(roadImageBlur, labRoadImage, COLOR_BGR2Lab);
	im = labThresholdingIntersection(labRoadImage);
	// Create a structuring element (SE)
	int morph_size = 2;
	Mat element = getStructuringElement(MORPH_RECT, Size(2 * morph_size + 1, 2 * morph_size + 1), Point(morph_size, morph_size));

	for (int i = 0; i<2; i++)
	{
		morphologyEx(im, im, 2, element, Point(-1, -1), i);
	}

	//Canny(im,im,50,150);
	//imshow("Canny Image", im);
	//imshow("Image", im);
	int counter = 0;
	//int arrayPercentH[1280];
	int arrayPercentV[1280];
	float percentage;
	x = 0; y = 0;
	//FILE *f;
	//f = fopen("frequency.txt", "w");
	/*****for vertical*****/

	for (int x = 0; x<im.size().width; x++)
	{
		for (y = 0, counter = 0; y<im.size().height; y++)
		{
			if (im.at<uchar>(y, x) == 255)  // these threshold values must be tuned or determined automatically!
			{
				counter++;
			}
		}
		percentage = (float)counter / im.size().height;

		if (percentage >= 0.40)
		{
			arrayPercentV[x] = 255;
		}
		else
		{
			arrayPercentV[x] = 0;
		}


		//imshow("im",im);

	}
	/*****for vertical*****/


	/***for vertical**/
	for (int x = 0; x<im.size().width; x++)
	{
		for (y = 0; y<im.size().height; y++)
		{
			im.at<uchar>(y, x) = arrayPercentV[x];  // these threshold values must be tuned or determined automatically!
		}
	}
	/***for verticaltal**/


	//imshow("bin",im);

	int dilate_size = 2;
	Mat dilateElement = getStructuringElement(cv::MORPH_RECT, Size(2 * dilate_size + 1, 2 * dilate_size + 1), Point(dilate_size, dilate_size));

	for (int i = 0; i<6; i++)
	{
		//Apply erosion or dilation on the image
		dilate(im, im, dilateElement);
	}
	//imshow("Result Road Dilate",im);

	double largest_area = 0;
	double a;
	int largest_contour_index;


	findContours(im, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
	for (int i = 0; i< contours.size(); i++)
	{
		a = contourArea(contours[i], false);  //  Find the area of contour
		if (a>largest_area)
		{
			largest_area = a;
			largest_contour_index = i;                //Store the index of largest contour
			bounding_rect = boundingRect(contours[i]); // Find the bounding rectangle for biggest contour
		}
	}
	//drawContours(origFrame, contours, largest_contour_index, Scalar(255), 1, 8, hierarchy);
	//imshow("largest area vertical", origFrame);
	//imshow("largest area", dstImg);
	return bounding_rect;
}


Scalar speedSpectrum(double speed)
{
	double R = 0.05 * (40 - speed);
	double G = 0.05 * speed;
	R = (R > 1) ? 1 : (R < 0.06) ? 0 : R;
	G = (G > 1) ? 1 : (G < 0.06) ? 0 : G;

	return Scalar(0.0, G * 255, R * 255);
}

static void onMouse(int event, int x, int y, int /*flags*/, void* /*param*/)
{
	if (event == EVENT_LBUTTONDOWN)
	{
		point = Point2f((float)x, (float)y);
		std::cout << "x: " << x << " y: " << y << endl;
		//addRemovePt = true;
	}
}

void setLabel(cv::Mat& im, const std::string label, const cv::Point & pointor )
{
	int fontface = cv::FONT_HERSHEY_TRIPLEX;
	double scale = 0.6;
	int thickness = 1;
	int baseline = 0;

	cv::Size text = cv::getTextSize(label, fontface, scale, thickness, &baseline);
	cv::rectangle(im, pointor +cv::Point(0, baseline), pointor +cv::Point(text.width, -text.height), CV_RGB(255, 255, 255), CV_FILLED);
	cv::putText(im, label, pointor, fontface, scale, CV_RGB(0, 0, 0), thickness, 8);
}

Point2i vehicleCountTopAndBot(vector<Vehicle> &vehicles, int halfY)
{
	int Top = 0;
	int Bot = 0;
	for (int i = 0; i < vehicles.size(); i++) {
		if (vehicles.at(i).getYPos() < halfY)
		{
			Top++;
		}
		else
		{
			Bot++;
		}
	}

	return Point2i(Top, Bot);
}

Mat sobelDetection(Mat curFrame)
{
	//sobel parameters
	Mat gray;
	int i, j;
	Mat grad;
	Mat grad_x, grad_y;
	Mat abs_grad_x, abs_grad_y;
	int scale = 1;
	int delta = 0;
	int ddepth = CV_16S;
	Mat curFrameCpy = curFrame.clone();
	Mat sobelResult = curFrame.clone();
	Mat thresholdImage;
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	//find contours of filtered image using openCV findContours function
	vector<vector<Point> > contours_poly(contours.size());
	vector<Rect> boundRect(contours.size());
	double minVal, maxVal;
	minMaxLoc(curFrameCpy, &minVal, &maxVal);  //find  minimum  and  maximum  intensities
	curFrameCpy.convertTo(gray, CV_8U, 255.0 / (maxVal - minVal), -minVal);
	GaussianBlur(gray, gray, Size(15, 15), 0, 0, 1);
	//imshow("gray",gray);
	Sobel(gray, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT);
    convertScaleAbs(grad_x, abs_grad_x);
	Sobel(gray, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT);
	convertScaleAbs(grad_y, abs_grad_y);
	addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad);
	//cv::imshow("Sobel Image", grad);
	//cv::threshold(abs_grad_y, thresholdImage, 30, 255, THRESH_BINARY);
	cv::threshold(grad, thresholdImage, 20, 255, THRESH_BINARY);
	//cv::imshow("Threshold Image", thresholdImage);
	morphologyEx(thresholdImage, thresholdImage, MORPH_OPEN, Mat::ones(2, 2, CV_8SC1), Point(1, 1), 2);
	//cv::imshow("Sobel Morphed Image", thresholdImage);
	
	return thresholdImage;

}