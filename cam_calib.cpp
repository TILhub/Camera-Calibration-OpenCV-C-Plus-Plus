#include <vector>
#include <iostream>
#include <fstream>
#include <opencv2/core/core.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/highgui/highgui.hpp>
using namespace cv;
using namespace std;

inline bool fexists(const std::string& name) {
	ifstream f(name.c_str());
	return f.good();
}

class CameraCalibrator {
	std::vector<std::vector<cv::Point3f>> objectPoints;
	std::vector<std::vector<cv::Point2f>> imagePoints;
	cv::Mat cameraMatrix;
	cv::Mat distCoeffs;
	int flag;
	cv::Mat map1, map2;
	bool mustInitUndistort;

public:
	CameraCalibrator() : flag(0), mustInitUndistort(true) {};
	int addChessboardPoints(const std::vector<std::string>& filelist, cv::Size & boardSize);
	void addPoints(const std::vector<cv::Point2f>& imageCorners, const std::vector<cv::Point3f>& objectCorners);
	double calibrate(cv::Size &imageSize);
	void setCalibrationFlag(bool radial8CoeffEnabled = false, bool tangentialParamEnabled = false);
	Mat CameraCalibrator::remap(const cv::Mat &image);
	Mat getCameraMatrix() { return cameraMatrix; }
	Mat getDistCoeffs() { return distCoeffs; }
};

int CameraCalibrator::addChessboardPoints(
	const std::vector<std::string>& filelist, cv::Size & boardSize) {

	// the points on the chessboard
	std::vector<cv::Point2f> imageCorners;
	std::vector<cv::Point3f> objectCorners;

	// 3D Scene Points:
	// Initialize the chessboard corners 
	// in the chessboard reference frame
	// The corners are at 3D location (X,Y,Z)= (i,j,0)
	for (int i = 0; i<boardSize.height; i++) {
		for (int j = 0; j<boardSize.width; j++) {

			objectCorners.push_back(cv::Point3f(float(i), float(j), 0.0f));
		}
	}
	cv::Mat image;
	int successes = 0;

	for (int i = 0; i<filelist.size(); i++) {
		image = cv::imread(filelist[i], 0);
		bool found = cv::findChessboardCorners(image, boardSize, imageCorners);
		// Get subpixel accuracy on the corners
		cv::cornerSubPix(image, imageCorners,cv::Size(5, 5),cv::Size(-1, -1),cv::TermCriteria(cv::TermCriteria::MAX_ITER +cv::TermCriteria::EPS,30,	0.1));   
		// If we have a good board, add it to our data
		if (imageCorners.size() == boardSize.area()) {
			// Add image and scene points from one view
			addPoints(imageCorners, objectCorners);
			successes++;
		}

		//Draw the corners
		cv::drawChessboardCorners(image, boardSize, imageCorners, found);
		cv::imshow("Corners on Chessboard", image);
		//cv::waitKey(100);
	}

	return successes;
}

void CameraCalibrator::addPoints(const std::vector<cv::Point2f>& imageCorners, const std::vector<cv::Point3f>& objectCorners) {
	imagePoints.push_back(imageCorners);
	objectPoints.push_back(objectCorners);
}

double CameraCalibrator::calibrate(cv::Size &imageSize)
{
	mustInitUndistort = true;
	std::vector<cv::Mat> rvecs, tvecs;
	return	calibrateCamera(objectPoints, // the 3D points
			imagePoints,  // the image points
			imageSize,    // image size
			cameraMatrix, // output camera matrix
			distCoeffs,   // output distortion matrix
			rvecs, tvecs, // Rs, Ts 
			flag);        // set options
						  //					,CV_CALIB_USE_INTRINSIC_GUESS);

}

cv::Mat CameraCalibrator::remap(const cv::Mat &image) {

	cv::Mat undistorted;

	if (mustInitUndistort) { // called once per calibration

		cv::initUndistortRectifyMap(
			cameraMatrix,  // computed camera matrix
			distCoeffs,    // computed distortion matrix
			cv::Mat(),     // optional rectification (none) 
			cv::Mat(),     // camera matrix to generate undistorted
			cv::Size(640, 480),
			//            image.size(),  // size of undistorted
			CV_32FC1,      // type of output map
			map1, map2);   // the x and y mapping functions
		imwrite("map1.jpg",map1);
		imwrite("map2.jpg",map2);
		mustInitUndistort = false;
	}

	// Apply mapping functions
	cv::remap(image, undistorted, map1, map2,
		cv::INTER_LINEAR); // interpolation type

	return undistorted;
}


// Set the calibration options
// 8radialCoeffEnabled should be true if 8 radial coefficients are required (5 is default)
// tangentialParamEnabled should be true if tangeantial distortion is present
void CameraCalibrator::setCalibrationFlag(bool radial8CoeffEnabled, bool tangentialParamEnabled) {

	// Set the flag used in cv::calibrateCamera()
	flag = 0;
	if (!tangentialParamEnabled) flag += CV_CALIB_ZERO_TANGENT_DIST;
	if (radial8CoeffEnabled) flag += CV_CALIB_RATIONAL_MODEL;
}

int main()
{
	Mat camera_m=imread("cam_mat.jpg");
	if (!camera_m.empty()) {
		Mat undistorted,m1, m2,image;
		m1 = imread("map1.jpg");
		m2 = imread("map2.jpg");
		image = imread("E:/chessboards/chess (4).jpg");
		remap(image, undistorted, m1, m2,cv::INTER_LINEAR); // interpolation type
		imshow("Original Image", image);
		imshow("Undistorted Image", undistorted);
		waitKey(0);
		return 0;
	}
	else {
		
		//cv::namedWindow("Image");
		cv::Mat image;
		std::vector<std::string> filelist;
		for (int i = 1; i <= 10; i++) {

			std::stringstream str;
			str << "e:/chessboards/chess (" << to_string(i) << ").jpg";
			std::cout << str.str() << std::endl;

			filelist.push_back(str.str());
			image = cv::imread(str.str(), 0);
			//cv::imshow("Image", image);
			//cv::waitKey(10);
		}
		CameraCalibrator cameraCalibrator;
		cv::Size boardSize(6, 4);
		cameraCalibrator.addChessboardPoints(filelist, boardSize);
		cameraCalibrator.calibrate(image.size());
		image = cv::imread(filelist[9]);
		cv::Mat uImage = cameraCalibrator.remap(image);
		cv::Mat cameraMatrix = cameraCalibrator.getCameraMatrix();
		imwrite("cam_mat.jpg",cameraMatrix);
		cout << " Camera intrinsic: " << cameraMatrix.rows << "x" << cameraMatrix.cols << std::endl;
		cout << cameraMatrix.at<double>(0, 0) << " " << cameraMatrix.at<double>(0, 1) << " " << cameraMatrix.at<double>(0, 2) << std::endl;
		cout << cameraMatrix.at<double>(1, 0) << " " << cameraMatrix.at<double>(1, 1) << " " << cameraMatrix.at<double>(1, 2) << std::endl;
		cout << cameraMatrix.at<double>(2, 0) << " " << cameraMatrix.at<double>(2, 1) << " " << cameraMatrix.at<double>(2, 2) << std::endl;

	}
	return 0;
}
