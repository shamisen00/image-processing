#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

int main() {

    String path = "images/1.jpg";
    Mat im = imread(path);
    Size boardSize = Size(7,7);

    vector<Point2f> pointBuf;
    bool found = findChessboardCorners( im, boardSize, pointBuf);

    if ( found)
    {
        drawChessboardCorners( im, boardSize, Mat(pointBuf), found);
    }

    imwrite("images/output.jpg", im);

    return 0;
}