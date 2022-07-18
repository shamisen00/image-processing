#include <iostream>
#include <string>
#include <sstream>
#include <iomanip>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <vector>
#include <filesystem>

int main()
{
    cv::Mat R, T, E, F;

    double rsm = cv::stereoCalibrate(obj_points, img_pointsL, img_pointsR, cam_matL, dist_coefsL, cam_matR, dist_coefsR, imgL.size(),
        R, T, E, F, cv::CALIB_USE_INTRINSIC_GUESS , cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 1e-6));
}