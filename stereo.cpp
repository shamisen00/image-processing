#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <vector>
#include <filesystem>

using namespace cv;
using namespace std;


// ファイル名読み込み
bool readStringList( const string& filename, vector<string>& l )
{
    l.clear();
    FileStorage fs(filename, FileStorage::READ);
    if( !fs.isOpened() )
        return false;
    FileNode n = fs.getFirstTopLevelNode();
    if( n.type() != FileNode::SEQ )
        return false;
    FileNodeIterator it = n.begin(), it_end = n.end();
    for( ; it != it_end; ++it )
        l.push_back((string)*it);
    return true;
}

void calcBoardCornerPositions(vector<Point3f>& corners, Size boardsize);

// パラータを保存
void saveCameraParams(Mat& cameraMatrix, Mat& distCoeffs);

#define squareSize (50)

int main()
{
    vector<string> imageListR, imageListL;
    string inputR = "images/left.xml";
    string inputL = "images/right.xml";
    cv::Size boardsize = cv::Size(7, 10);
    float grid_width = squareSize * (boardsize.width - 1);
    vector<vector<cv::Point2f>> pointBufsR, pointBufsL;
    Mat cameraMatrix, distCoeffs;
    Size imageSize;
    vector<cv::Mat> imagesR,imagesL;
    vector<int> imgNumber;

    readStringList(inputR, imageListR);
    readStringList(inputL, imageListL);

    // 1.
    for(size_t i = 0; i < imageListR.size(); i++ )
    {
        imagesR.push_back(imread(imageListR[i], IMREAD_COLOR));
        imagesL.push_back(imread(imageListL[i], IMREAD_COLOR));
    }

        for(size_t i = 0; i < imageListR.size(); i++ )
    {
        vector<Point2f> pointBufR, pointBufL;
        
        bool foundR = cv::findChessboardCorners(imagesR[i], boardsize, pointBufR);
        bool foundL = cv::findChessboardCorners(imagesL[i], boardsize, pointBufL);
        if( foundR && foundL)
        {
            pointBufsR.push_back(pointBufR);
            pointBufsL.push_back(pointBufL);
            imgNumber.push_back(i);
        }
    }

    cv::imwrite("01.jpg", imagesL[1]);
    imageSize = imagesR[0].size();

    cv::Mat cam_matL, cam_matR;
    cv::Mat dist_coefsL, dist_coefsR;

    // vector<vector<Point3f> > objectPoints(1);
    // calcBoardCornerPositions(objectPoints[0], boardsize);
    // objectPoints.resize(pointBufsR.size(), objectPoints[0]);

    vector<cv::Point3f> object;
    for (int j = 0; j < boardsize.height; j++) {
        for (int k = 0; k < boardsize.width; k++) {

            cv::Point3f p(j * squareSize, k * squareSize, 0.0);
            object.push_back(p);
        }
    }

    vector<vector<cv::Point3f>> objectPoints;
    for (int i = 0; i < imgNumber.size(); i++)
    {
        objectPoints.push_back(object);
    }

    cam_matR = cv::initCameraMatrix2D(objectPoints, pointBufsR, imageSize);
    cam_matL = cv::initCameraMatrix2D(objectPoints, pointBufsL, imageSize);

    cv::Mat R, T, E, F;

    double rsm = cv::stereoCalibrate(objectPoints, pointBufsL, pointBufsR, cam_matL, dist_coefsL, cam_matR, dist_coefsR, imageSize, R, T, E, F, cv::CALIB_USE_INTRINSIC_GUESS , cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 1e-6));
    
    cv::Mat R1, R2, P1, P2, Q;
    cv::stereoRectify(cam_matL, dist_coefsL, cam_matR, dist_coefsR, imageSize, R, T, R1,  R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY, -1,  cv::Size(),0,  0);

    cv::Mat map1L, map2L,map1R,map2R;
    cv::initUndistortRectifyMap(cam_matL, dist_coefsL, R1, P1, imageSize, CV_32FC1, map1L, map2L);
    cv::initUndistortRectifyMap(cam_matR, dist_coefsR, R2, P2, imageSize, CV_32FC1, map1R, map2R);

    cv::Mat outL,outR;
    cv::remap(imagesL[imgNumber[0]], outL, map1L, map2L, cv::INTER_LINEAR);
    cv::remap(imagesR[imgNumber[0]], outR, map1R, map2R, cv::INTER_LINEAR);
    
    cv::imwrite("1.jpg", outL);
    cv::imwrite("2.jpg", outR);

    return 0;
}

void calcBoardCornerPositions(vector<Point3f>& corners, Size boardsize)
{
    corners.clear();

    for( int i = 0; i < boardsize.height; ++i )
        for( int j = 0; j <boardsize.width; ++j )
            corners.push_back(Point3f(j*squareSize, i*squareSize, 0));
}