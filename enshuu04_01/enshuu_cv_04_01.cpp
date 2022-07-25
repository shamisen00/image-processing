/**********************************************************************
                                Created on 2021 / 09 / 03 by Haiki Ritsuto


�@�\ �F�X�e���I�J�����Z��
�߂�l�F0(����), -1(�ُ�)
���� �F�Ȃ�
* *********************************************************************/

#include <iostream>
#include <string>
#include <sstream>
#include <iomanip>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <vector>
#include <filesystem>

#define ROW (7)     // �񐔁i�^�C���̌�������_�j
#define COL (10)    // �s���i�^�C���̌�������_�j
#define SIZE   (ROW * COL)  // 1���̌����_�̐�
#define CHESS_SIZE (0.05) // 1�}�X�̒���

int main()
{
    using namespace std;
    using namespace std::filesystem;

    vector<cv::Mat> imgLeft, imgRight;
    vector<vector<cv::Point2f>> Lcorners,Rcorners;
    cv::Size pattern_size = cv::Size(COL, ROW);
    vector<vector<cv::Point2f>> img_pointsL, img_pointsR;
    int img_num = 0;


    //pl = "D:\\pic\\Enshuu4\\Enshuu4\\Enshuu_cv_04_01\\left";
    //pr = "D:\\pic\\Enshuu4\\Enshuu4\\Enshuu_cv_04_01\\right";

    // �L�����u���[�V�����{�[�h�摜�~N(���E�y�A)���t�@�C������ǂݍ���
    // �e�摜����R�[�i�[�����o����
    // ���E�̂����ꂩ�ŃR�[�i�[�̌��o�Ɏ��s�����ꍇ�̓T���v������O��
    cv::Mat imgR, imgL;
    vector<cv::Point2f > l_corner,r_corner;
    for (const auto & file : directory_iterator("D:\\pic\\Enshuu4\\Enshuu_cv_04_01\\left")){
        ostringstream ss;
        ss << file.path();

        string left = ss.str().substr(1,47);
        string right = ss.str().replace(36,4,"right").substr(1, 48);

        //cv::Mat i = cv::imread("D:\\pic\\Enshuu4\\Enshuu_cv_04_01\\left\\00.png");
        imgL = cv::imread(left);
        imgR = cv::imread(right);
        //cv::imshow("1", imgL);
        //cv::waitKey(10000);
        //cout << imgL.size().height << endl;

        bool foundL = cv::findChessboardCorners(imgL, pattern_size, l_corner);
        bool foundR = cv::findChessboardCorners(imgR, pattern_size, r_corner);
        if (foundL && foundR) {
            img_num += 1;
            imgLeft.push_back(imgL);
            imgRight.push_back(imgR);
            Lcorners.push_back(l_corner);
            Rcorners.push_back(r_corner);
        }
        else {
            std::cout << "�摜�ŃR�[�i�[���o�����s���܂���" << std::endl;
            continue;
        }
    }

    for (int i = 0; i < img_num; i++) {

        cv::Mat src_grayL = cv::Mat(imgL.size(), CV_8UC1);
        cv::Mat src_grayR = cv::Mat(imgR.size(), CV_8UC1);
        cv::cvtColor(imgL, src_grayL, cv::COLOR_BGR2GRAY);
        cv::cvtColor(imgR, src_grayR, cv::COLOR_BGR2GRAY);
        cv::find4QuadCornerSubpix(src_grayL, Lcorners[i], cv::Size(3, 3));
        cv::find4QuadCornerSubpix(src_grayR, Rcorners[i], cv::Size(3, 3));
        img_pointsL.push_back(Lcorners[i]);
        img_pointsR.push_back(Rcorners[i]);

        /*cv::drawChessboardCorners(imgLeft[i], pattern_size, Lcorners[i], true);
        cv::drawChessboardCorners(imgRight[i], pattern_size, Rcorners[i], true);
        cv::imshow("cornerL", imgLeft[i]);
        cv::imshow("cornerR", imgRight[i]);
        cv::waitKey(1000);*/
    }
    // 3�������W�n�̐ݒ�
    vector<cv::Point3f> object;
    for (int j = 0; j < ROW; j++) {
        for (int k = 0; k < COL; k++) {

            cv::Point3f p(j * CHESS_SIZE, k * CHESS_SIZE, 0.0);
            object.push_back(p);
        }
    }

    vector<vector<cv::Point3f>> obj_points;
    for (int i = 0; i < img_num; i++)
    {
        obj_points.push_back(object);
    }

    // ���o�����R�[�i�[�̍��W���g�p���ăX�e���I�J�����Z�����s���A�J�����̓����p�����^�y�ъO���p�����^�𓾂�
    cv::Mat cam_matL, cam_matR;                   // �J���������p�����[�^�s��
    cv::Mat dist_coefsL, dist_coefsR;                // �c�݌W��
    vector<cv::Mat> rvecsL, tvecsL, rvecsR, tvecsR;      // �e�r���[�̉�]�x�N�g���ƕ��i�x�N�g��
    //auto rsmL = cv::calibrateCamera(obj_points, img_pointsL, imgL.size(), cam_matL, dist_coefsL, rvecsL, tvecsL);
    cam_matL = cv::initCameraMatrix2D(obj_points, img_pointsL, imgL.size());
    //auto rsmR = cv::calibrateCamera(obj_points, img_pointsR, imgR.size(), cam_matR, dist_coefsR, rvecsR, tvecsR);
    cam_matR = cv::initCameraMatrix2D(obj_points, img_pointsR, imgR.size());

    cv::Mat R, T, E, F;

    double rsm = cv::stereoCalibrate(obj_points, img_pointsL, img_pointsR, cam_matL, dist_coefsL, cam_matR, dist_coefsR, imgL.size(),
        R, T, E, F, cv::CALIB_USE_INTRINSIC_GUESS , cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 1e-6));



    // �J�����̓����p�����^�y�ъO���p�����^���g�p���ăX�e���I�J�����̕��s���ϊ������߂�
    cv::Mat R1, R2, P1, P2, Q;
    cv::stereoRectify(cam_matL,dist_coefsL,cam_matR,dist_coefsR,imgL.size(), R, T, R1, R2,P1, P2,Q,  cv::CALIB_ZERO_DISPARITY, -1,  cv::Size(),0,  0);

    //cout << P1 << endl;

    cv::Mat map1L, map2L,map1R,map2R;
    cv::initUndistortRectifyMap(cam_matL, dist_coefsL, R1, P1,imgL.size(), CV_32FC1, map1L,  map2L);
    cv::initUndistortRectifyMap(cam_matR, dist_coefsR, R2, P2, imgL.size(), CV_32FC1, map1R, map2R);
    cv::Mat outL,outR;
    cv::remap(imgLeft[0], outL, map1L, map2L, cv::INTER_LINEAR);
    cv::remap(imgRight[0], outR, map1R, map2R, cv::INTER_LINEAR);

    cv::imshow("1", outL);
    cv::imshow("2", outR);
    //cv::waitKey(100000);
    
    // �L�����u���[�V�����{�[�h�摜�����E�ɕ��ׂĕ\������(���摜����ѕ��s����������)�B
    // ���s���̋���m�F���邽�߂̃K�C�h���C���𕹂��ĕ`�悷�邱�ƁB
   
    cv::Mat img_out;
    cv::hconcat( outL, outR,img_out);
    int width = img_out.size().width;
    cv::resize(img_out, img_out, cv::Size(), 0.5, 0.5);
    for (int i = 0; i < img_out.size().height; i++) {
        if (i % 10 == 0) {
            cv::line(img_out, cv::Point2f(0, i), cv::Point2f(width, i), (0, 0, 255), 1);
        }  
    }
    if (!cv::imwrite("D:\\pic\\enshuu04_01.jpg", img_out)) {
        std::cout << "�摜�̕ۑ��Ɏ��s���܂���" << std::endl;
        return -1;
    }

    cv::imshow("111", img_out);
    cv::waitKey(100000);
    // �J�����p�����^(�����A�O���A���s���ϊ�)��yaml�`���Ńt�@�C���ۑ�����
    cv::FileStorage fs("D:\\pic\\enshuu04_01_outcome.yaml", cv::FileStorage::WRITE);
    if (!fs.isOpened()) {
        std::cout << "File can not be opened." << std::endl;
        return -1;
    }
    //fs << "rsm" << rsm;
    fs << "Left Camera Matrix" << cam_matL;
    fs << "Right Camera Matrix" << cam_matR;
    fs << "Left Distortion" << dist_coefsL;
    fs << "Right Distortion" << dist_coefsR;
    fs << "R1" << R1;
    fs << "R2" << R2;
    fs << "P1" << P1;
    fs << "P2" << P2;
    fs.release();

    return 0;
}
