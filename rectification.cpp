#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <vector>
#include <filesystem>

using namespace cv;
using namespace std;
using namespace std::filesystem;

void calcBoardCornerPositions(vector<Point3f>& corners, Size boardsize);

// パラータを保存
void saveCameraParams(Mat& cameraMatrix, Mat& distCoeffs);

#define squareSize (50)
#define Num (5)

int main()
{
    cv::Size boardsize = cv::Size(7, 10);
    float grid_width = squareSize * (boardsize.width - 1);
    vector<vector<cv::Point2f>> pointBufsR, pointBufsL;
    Mat cameraMatrix, distCoeffs;
    Size imageSize;
    vector<cv::Mat> imagesR,imagesL;
    vector<int> imgNumber;

    // 1.画像読み込み
    cv::Mat imgR, imgL;
    for (const auto & file : directory_iterator("images/stereo/right")){
        imgR = cv::imread(file.path());
        imagesR.push_back(imgR);
    }

    for (const auto & file : directory_iterator("images/stereo/left")){
        imgL = cv::imread(file.path());
        imagesL.push_back(imgL);
    }

    // 2.コーナー検出
    for(size_t i = 0; i < imagesR.size(); i++ )
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

    imageSize = imagesR[0].size();

    cv::Mat cam_matL, cam_matR;
    cv::Mat dist_coefsL, dist_coefsR;

    // 3.カメラ校正とパラメータ推定
    vector<vector<Point3f> > objectPoints(1);
    calcBoardCornerPositions(objectPoints[0], boardsize);
    objectPoints.resize(pointBufsR.size(), objectPoints[0]);

    cam_matR = cv::initCameraMatrix2D(objectPoints, pointBufsR, imageSize);
    cam_matL = cv::initCameraMatrix2D(objectPoints, pointBufsL, imageSize);

    cv::Mat R, T, E, F;

    double rsm = cv::stereoCalibrate(objectPoints, pointBufsL, pointBufsR, cam_matL, dist_coefsL, cam_matR, dist_coefsR, imageSize, R, T, E, F, cv::CALIB_USE_INTRINSIC_GUESS , cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 1e-6));
    
    // 4.平衡化変換を求める
    cv::Mat R1, R2, P1, P2, Q;
    cv::stereoRectify(cam_matL, dist_coefsL, cam_matR, dist_coefsR, imageSize, R, T, R1,  R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY, -1,  cv::Size(),0,  0);

    cv::Mat map1L, map2L,map1R,map2R;
    cv::initUndistortRectifyMap(cam_matL, dist_coefsL, R1, P1, imageSize, CV_32FC1, map1L, map2L);
    cv::initUndistortRectifyMap(cam_matR, dist_coefsR, R2, P2, imageSize, CV_32FC1, map1R, map2R);

    cv::Mat outL,outR;
    cv::remap(imagesL[imgNumber[Num]], outL, map1L, map2L, cv::INTER_LINEAR);
    cv::remap(imagesR[imgNumber[Num]], outR, map1R, map2R, cv::INTER_LINEAR);

    // 5.画像を並べ、平行線のガイドラインを表示
    cv::Mat img_out;
    cv::hconcat( outL, outR,img_out);
    int width = img_out.size().width;
    cv::resize(img_out, img_out, cv::Size(), 0.5, 0.5);
    for (int i = 0; i < img_out.size().height; i++) {
        if (i % 10 == 0) {
            cv::line(img_out, cv::Point2f(0, i), cv::Point2f(width, i), (0, 0, 255), 1);
        }  
    }
    
    cv::imwrite("stereo_scopic.jpg", img_out);

    // 6.パラメータを保存
    cv::FileStorage fs("params.yaml", cv::FileStorage::WRITE);
    fs << "Left_Camera Matrix" << cam_matL;
    fs << "Right_Camera Matrix" << cam_matR;
    fs << "Left_Distortion" << dist_coefsL;
    fs << "Right_Distortion" << dist_coefsR;
    fs << "R1" << R1;
    fs << "R2" << R2;
    fs << "P1" << P1;
    fs << "P2" << P2;
    fs << "Q" << Q;
    fs.release();

    return 0;
}

void calcBoardCornerPositions(vector<Point3f>& corners, Size boardsize)
{
    corners.clear();

    for( int i = 0; i < boardsize.height; ++i )
        for( int j = 0; j <boardsize.width; ++j )
            corners.push_back(Point3f(j*squareSize, i*squareSize, 0));
}
