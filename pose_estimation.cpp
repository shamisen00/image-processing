#include <iostream>
#include <string>
#include <sstream>
#include <iomanip>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <vector>

#define ROW (7)
#define COL (10)
#define CHESS_SIZE (0.05) 

int main()
{
    using namespace std;
    cv::Mat img;
    cv::Mat cam_mat, dist_coefs;
    vector<cv::Point2f> corner;
    cv::Size pattern_size = cv::Size(10, 7);

    // 1.画像の読み込み
    img = cv::imread("/workspace/Enshu3/images/00.png");

    // 2.yamlファイルから内部パラメタを読み込む
    cv::FileStorage fs("/workspace/Enshu3/out_camera_data.yml", cv::FileStorage::READ);

    fs["camera_matrix"] >> cam_mat;
    fs["distortion_coefficients"] >> dist_coefs;
    fs.release();

    // 3.コーナー検出
    bool found = cv::findChessboardCorners(img, pattern_size, corner);

    vector<cv::Point3f> object;
    for (int i = 0; i < pattern_size.height; i++) {
        for (int j = 0; j < pattern_size.width; j++) {
            cv::Point3f p(j * CHESS_SIZE, i * CHESS_SIZE, 0.0);
            object.push_back(p);
        }
    }

    // 4.5 コーナー座標(2D)、世界座標系上のコーナー座標、内部パラメータからカメラの位置・姿勢を求め6DoFの位置・姿勢に分解する
    cv::Mat rvec, tvec;
    cv::solvePnP(object, corner, cam_mat, dist_coefs, rvec, tvec);

    cv::Mat rMat;
    cv::Rodrigues(rvec, rMat);

    cv::Mat R, Q;
    // オイラー角を求める
    cv::Vec3d eulerAngles = cv::RQDecomp3x3(rMat, R, Q, cv::noArray(), cv::noArray(), cv::noArray());

    // 6. 6DoFの位置・姿勢を描画し保存する
    ostringstream ss;

    ss.str("");
    ss.clear(stringstream::goodbit);
    ss << "tvec(mm) =  (" << tvec.at<double>(0) << "," << tvec.at<double>(1) << "," << tvec.at<double>(2) << ")";
    cv::putText(img, ss.str(), cv::Point(10, 85), 1, cv::FONT_HERSHEY_DUPLEX, cv::Scalar(255, 255, 255), 1);

    ss.str("");
    ss.clear(stringstream::goodbit);
    ss << "eulerAngles = (" << eulerAngles[0] << "," << eulerAngles[1] << "," << eulerAngles[2] << ")";
    cv::putText(img, ss.str(), cv::Point(25, 55), 1, cv::FONT_HERSHEY_DUPLEX, cv::Scalar(255, 255, 255), 1);

    ss.str("");
    ss.clear(stringstream::goodbit);
    ss << "rvec = (" << rvec.at<double>(0) << "," << rvec.at<double>(1) << "," << rvec.at<double>(2) << ")";
    cv::putText(img, ss.str(), cv::Point(10,115), 1, cv::FONT_HERSHEY_DUPLEX, cv::Scalar(255, 255, 255), 1);

    cv::imwrite("enshuu03_03.png", img);

    return 0;
}