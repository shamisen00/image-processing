#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <vector>
#include <filesystem>

int SavePlyFormat(std::string outputPath, cv::Mat image, cv::Mat image3D) {

    std::ofstream ifs(outputPath);
    ifs << "ply" << std::endl;
    ifs << "format ascii 1.0" << std::endl;
    ifs << "comment PCL generated" << std::endl;
    ifs << "element vertex " << image3D.size().area() << std::endl;
    ifs << "property float x" << std::endl;
    ifs << "property float y" << std::endl;
    ifs << "property float z" << std::endl;
    ifs << "property uchar red" << std::endl;
    ifs << "property uchar green" << std::endl;
    ifs << "property uchar blue" << std::endl;
    ifs << "end_header" << std::endl;

    for (int i = 0; i < image3D.rows; i++) {
        for (int j = 0; j < image3D.cols; j++) {
            ifs << image3D.at<cv::Point3f>(i, j).x << " " << image3D.at<cv::Point3f>(i, j).y << " " << image3D.at<cv::Point3f>(i, j).z << " "
                << (int)(image.at<cv::Vec3b>(i, j)[2]) << " " << (int)(image.at<cv::Vec3b>(i, j)[1]) << " " << (int)(image.at<cv::Vec3b>(i, j)[0]) << std::endl;
        }
    }

    ifs.close();

    return 0;
}

int main()
{
    using namespace std;
    using namespace std::filesystem;

    vector<cv::Mat> imagesR,imagesL;
    vector<vector<cv::Point2f>> img_pointsL, img_pointsR;
    cv::Mat cam_matL, cam_matR;         
    cv::Mat dist_coefsL, dist_coefsR;   
    cv::Mat R1, R2, P1, P2,Q;

    // 1.ファイルを読み込む
    cv::Mat imgR, imgL;
    for (const auto & file : directory_iterator("images/test/right")){
        imgR = cv::imread(file.path());
        imagesR.push_back(imgR);
    }

    for (const auto & file : directory_iterator("images/test/left")){
        imgL = cv::imread(file.path());
        imagesL.push_back(imgL);
    }

    // 2.カメラパラメーターを読み込む
    cv::FileStorage fs("params.yaml", cv::FileStorage::READ);

    fs["Left_Camera Matrix"] >> cam_matL;
    fs["Right_Camera Matrix"] >> cam_matR;
    fs["Left_Distortion"] >> dist_coefsL;
    fs["Right_Distortion"] >> dist_coefsR;
    fs["R1"] >> R1;
    fs["R2"] >> R2;
    fs["P1"] >> P1;
    fs["P2"] >> P2;
    fs["Q"] >> Q;
    fs.release();

    cv::Mat map1L, map2L, map1R, map2R;
    cv::Mat inputL, inputR;
    int rows = imgL.size().height;
    int cols = imgL.size().width;
    int ch= imgL.channels();

    // 3.歪み補正、ステレオ並行化
    cv::initUndistortRectifyMap(cam_matL, dist_coefsL, R1, P1, imgL.size(), CV_32FC1, map1L, map2L);
    cv::initUndistortRectifyMap(cam_matR, dist_coefsR, R2, P2, imgL.size(), CV_32FC1, map1R, map2R);

    for (int i = 0; i < imagesR.size(); i++) {
        cv::remap(imagesL[i], inputL, map1L, map2L, cv::INTER_LINEAR);
        cv::remap(imagesR[i], inputR, map1R, map2R, cv::INTER_LINEAR);

        int window_size = 3;
        int minDisparity = 12;
        int numDisparities = 128;
        int blockSize = 3;
        int P1 = 8 * ch * window_size * window_size;
        int P2 = 32 * ch * window_size * window_size;
        int disp12MaxDiff = 1;
        int preFilterCap = 0;
        int uniquenessRatio = 10;
        int speckleWindowSize = 100;
        int speckleRange = 32;

        // 4.ステレオ計測(密) 全画素についてマッチングを行い、視差画像を生成
        cv::Ptr < cv::StereoSGBM > ssgbm = cv::StereoSGBM::create(
            minDisparity,
            numDisparities,
            blockSize,
            P1,
            P2, disp12MaxDiff,
            preFilterCap,
            uniquenessRatio,
            speckleWindowSize, speckleRange, cv::StereoSGBM::MODE_SGBM
        );

        cv::Mat disparity;
        ssgbm->compute(inputL, inputR, disparity);
        cv::imwrite("stereoscopic.png", disparity);

        // 5.視差画像を濃淡画像としてファイルに保存
        cv::Mat disparity_map;
        double min, max;
        cv::minMaxLoc(disparity, &min, &max);
        disparity.convertTo(disparity_map, CV_8UC1, 255.0 / (max - min), -255.0 * min / (max - min));

        cv::Mat abc(rows,cols,CV_8UC3);
        for (int s = 0; s < rows; s++)
        {
            for (int t = 0; t < cols; t++)
            {
                unsigned char vv = (unsigned char)(disparity.at<short>(s, t));
                if ((int)(vv)< 177){
                    continue;
                }
                else {
                    abc.at<cv::Vec3b>(s, t) = cv::Vec3b(vv, vv, vv);
                }
            }
        }
        
        cv::imwrite("grayscale.png", abc);

        // 6.カメラパラメタ及び視差画像から3D点群を生成
        cv::Mat img3d,disp;
        disparity.convertTo(disp, CV_32F, 1 / 16.0);
        cv::reprojectImageTo3D(disp, img3d, Q);
        cv::imwrite("point_cloud.png", disparity_map);

        // 7.3D点群を.ply形式でファイルに保存する
        ostringstream ss;
        ss << std::setfill('0') << std::right << std::setw(2) << i;

        string path = ss.str() + ".ply";
        SavePlyFormat(path, inputL, img3d);

        // 8.3D点群のZ座標を抽出して距離画像(実数)を生成する
        vector<cv::Mat> dst;
        cv::split(img3d, dst);
        cv::Mat zmap = dst[2];

        // 9.距離画像(実数)を256諧調の濃淡値に変換して可視化し、ファイルに保存する。
        cv::Mat zmap_gray;
        double minz, maxz;
        cv::minMaxLoc(zmap_gray, &minz, &maxz);
        disparity.convertTo(zmap_gray, CV_8UC1, 255.0 / (max - min), -255.0 * min / (max - min));
        fs << "zmap" << zmap_gray;
        fs.release();
    }

    return 0;
}