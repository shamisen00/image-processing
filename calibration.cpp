#include <iostream>
#include <sstream>
#include <string>
#include <ctime>
#include <cstdio>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

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

// キャリブレーションを行い、再投影誤差を計算
double runCalibration(Size& imageSize, Mat& cameraMatrix, Mat& distCoeffs,
                            vector<vector<Point2f> > imagePoints, vector<Mat>& rvecs, vector<Mat>& tvecs, float grid_width, Size boardsize);

// パラータを保存
void saveCameraParams(Mat& cameraMatrix, Mat& distCoeffs);

#define squareSize (50)

int main()
{   
    vector<string> imageList;
    string input = "images/VID5.xml";
    cv::Size boardsize = cv::Size(7, 10);
    float grid_width = squareSize * (boardsize.width - 1);
    vector<vector<Point2f> > imagePoints;
    Mat cameraMatrix, distCoeffs;
    Size imageSize;
    vector<cv::Mat> images;


    readStringList(input, imageList);
    
    // 1.画像読み込み
    for(size_t i = 0; i < imageList.size(); i++ )
    {
        Mat view = imread(imageList[i], IMREAD_COLOR);
        images.push_back(view);
    }

    // 2.コーナー検出
    for(size_t i = 0; i < imageList.size(); i++ )
    {   
        vector<Point2f> pointBuf;
        
        bool found = cv::findChessboardCorners(images[i], boardsize, pointBuf);
        if( !pointBuf.empty() )
        {
            imagePoints.push_back(pointBuf);
        }
    }

    // 3.キャリブレーションを行い、再投影誤差を計算
    imageSize = images[0].size();

    vector<Mat> rvecs, tvecs;
    double rms = runCalibration(imageSize, cameraMatrix, distCoeffs, imagePoints, rvecs, tvecs, grid_width, boardsize);

    // 4.再投影誤差を表示
    std::cout << "reprojection error : " << rms << std::endl;

    // 5.校正前後の画像を並べて表示
    if( !cameraMatrix.empty())
        {
            Mat temp = images[4];

            //歪み補正
            Mat rview, map1, map2;

            initUndistortRectifyMap(
                cameraMatrix, distCoeffs, Mat(),
                cameraMatrix, imageSize,
                CV_16SC2, map1, map2);

            remap(temp, rview, map1, map2, INTER_LINEAR); // コーナー検出できなかったため検出は行っていない

            vector<Point2f> pointBuf;
            // コーナー検出
            bool found = cv::findChessboardCorners(temp, boardsize, pointBuf);
            if (found)
            {
                cv::drawChessboardCorners(temp, boardsize, Mat(pointBuf), found);
            }

            cv::Mat output;
            cv::hconcat(temp, rview, output);
            cv::resize(output, output, cv::Size(), 0.5, 0.5);
            cv::imwrite("enshuu03_01.png", output);

        }
    
    // 6.パラメタをyamlで保存する
    saveCameraParams(cameraMatrix, distCoeffs);

    return 0;
}


static void calcBoardCornerPositions(vector<Point3f>& corners, Size boardsize)
{
    corners.clear();

    for( int i = 0; i < boardsize.height; ++i )
        for( int j = 0; j <boardsize.width; ++j )
            corners.push_back(Point3f(j*squareSize, i*squareSize, 0));
}

double runCalibration(Size& imageSize, Mat& cameraMatrix, Mat& distCoeffs,
                            vector<vector<Point2f> > imagePoints, vector<Mat>& rvecs, vector<Mat>& tvecs, float grid_width, Size boardsize)
{
    cameraMatrix = Mat::eye(3, 3, CV_64F);

    vector<vector<Point3f> > objectPoints(1);
    calcBoardCornerPositions(objectPoints[0], boardsize);

    objectPoints.resize(imagePoints.size(),objectPoints[0]);

    double rms;

    rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs);

    return rms;
}

// yamlでパラメタを保存
void saveCameraParams(Mat& cameraMatrix, Mat& distCoeffs)
{
    FileStorage fs( "out_camera_data.yml", FileStorage::WRITE );

    fs << "camera_matrix" << cameraMatrix;
    fs << "distortion_coefficients" << distCoeffs;
}
