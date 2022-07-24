#include <iostream>
#include <string>
#include <sstream>
#include <iomanip>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <vector>


#define CHESS_SIZE (50.0) 

int main()
{
    using namespace std;
    cv::Mat img; // �摜�i�[�p
    cv::Mat cam_mat, dist_coefs; // �J�����p�����[�^�A�c�݌W�����i�[
    vector<cv::Point2f> corner;
    cv::Size pattern_size = cv::Size(7, 7);

    
    int iterationsCount = 500;
    float reprojectionError = 2.0;
    float confidence = 0.95; 

    img = cv::imread("/workspace/images/5.jpg");

    cv::FileStorage fs("/workspace/out_camera_data.yml", cv::FileStorage::READ);

    fs["camera_matrix"] >> cam_mat;
    fs["distortion_coefficients"] >> dist_coefs;
    fs.release();

    bool found = cv::findChessboardCorners(img, pattern_size, corner);

    vector<cv::Point3f> object;
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            cv::Point3f p(j * 50, i * 50, 0.0);
            object.push_back(p);
        }
    }

    bool useExtrinsicGuess = false;
    
    cv::Mat rvec, tvec;
    cv::solvePnPRansac(object, corner, cam_mat, dist_coefs, rvec, tvec);

    cv::Mat  rMat;
    cv::Rodrigues(rvec, rMat);

    cv::Mat R, Q;
    cv::Vec3d eulerAngles = cv::RQDecomp3x3(rMat, R, Q, cv::noArray(), cv::noArray(), cv::noArray());

    cv::Mat pos;

    pos = -rMat.t() * tvec;
    cout << pos << endl;

    ostringstream ss;
    string text1, text2;
    ss << "pos = (" << pos.at<double>(0) << "," << pos.at<double>(1) << "," << pos.at<double>(2) << ")[m]";
    cv::putText(img, ss.str(), cv::Point(25, 25), 1, cv::FONT_HERSHEY_DUPLEX, cv::Scalar(255, 255, 255), 1);
    ss.str("");
    ss.clear(stringstream::goodbit);
    ss << "deg = (" << eulerAngles[0] << "," << eulerAngles[1] << "," << eulerAngles[2] << ")";
    cv::putText(img, ss.str(), cv::Point(25, 55), 1, cv::FONT_HERSHEY_DUPLEX, cv::Scalar(255, 255, 255), 1);

    ss.str("");
    ss.clear(stringstream::goodbit);
    ss << "tvec =  (" << tvec.at<double>(0) << "," << tvec.at<double>(1) << "," << tvec.at<double>(2) << ")";
    cv::putText(img, ss.str(), cv::Point(10, 85), 1, cv::FONT_HERSHEY_DUPLEX, cv::Scalar(255, 255, 255), 1);

    ss.str("");
    ss.clear(stringstream::goodbit);
    ss << "rvec = (" << rvec.at<double>(0) << "," << rvec.at<double>(1) << "," << rvec.at<double>(2) << ")";
    cv::putText(img, ss.str(), cv::Point(10,115), 1, cv::FONT_HERSHEY_DUPLEX, cv::Scalar(255, 255, 255), 1);

    cv::imwrite("pos.png", img);
}