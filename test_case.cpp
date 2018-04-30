#include <opencv2/opencv.hpp>
#include <string>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#define X 6			//number of vertices in a row of white blocks or number of white blocks in a rows*2
#define Y 4			//number of vertics in a column of white blocks or number of white blocks in a column*2
using namespace std;
using namespace cv;

Size boardSize(X,Y);
int main()
{
	
	for (int i = 1;i <= 43;i++) {
		string path_chessboards = "e:/chessboards/chess (";
		path_chessboards += to_string(i);
		path_chessboards += ").jpg";
		Mat image = imread(path_chessboards);
		vector<Point2f> imageCorners;
		bool found = findChessboardCorners(image, boardSize, imageCorners);
		drawChessboardCorners(image, boardSize, imageCorners, found);
		imageCorners.clear();
		imshow("Test", image);
		waitKey(10);
	}
	return 0;
}