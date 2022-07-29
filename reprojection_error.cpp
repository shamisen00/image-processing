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
#define SIZE   (ROW * COL)
#define CHESS_SIZE (50.0)
// Zはデータを取得した環境で計測された値を用いた
#define Z (850.0)

int main()
{

    using namespace std;
    cv::Mat img;
    cv::Mat cameraMatrix;
    cv::Size pattern_size = cv::Size(COL, ROW);
    cv::Mat distCoeffs;
    vector<cv::Point2f > corner;

    // 1.ファイルの読み込み
    img = cv::imread("/workspace/Enshu3/images/00.png");

    cv::FileStorage fs("/workspace/Enshu3/out_camera_data.yml", cv::FileStorage::READ);

    // 2.パラメータ読み込み
    fs["camera_matrix"] >> cameraMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    fs.release();


    double fx = cameraMatrix.at<double>(0);
    double fy = cameraMatrix.at<double>(4);
    double cx = cameraMatrix.at<double>(2);
    double cy = cameraMatrix.at<double>(5);

    // 3.コーナー検出
    bool found = cv::findChessboardCorners(img, pattern_size, corner);
    if (!found) {
        cv::drawChessboardCorners(img, pattern_size, corner, found);
    }

    // 4.コーナー座標(2d)からカメラ座標(3d)に変換
    vector<cv::Point2f > dist;
    cv::Mat R;
    cv::undistortPoints(corner, dist, cameraMatrix, distCoeffs, R, cameraMatrix);

    vector<cv::Point3f> imagePoints3;
    for (int i = 0; i < SIZE; i++) {
        double X, Y;

        X = (Z * (dist[i].x - cx)) / fx;
        Y = (Z * (dist[i].y - cy)) / fy;
        cv::Point3f p(X, Y, Z);
        imagePoints3.push_back(p);
    }

    // 5. コーナー座標(3d)から2d座標に変換
    vector<double> rvecs(3);
    vector<double> tvecs(3);
    vector<cv::Point2f> imagePoints2;
    size_t totalPoints = 0;
    double totalErr = 0, err;
    projectPoints(imagePoints3, rvecs, tvecs, cameraMatrix, distCoeffs, imagePoints2);
    err = norm(corner, imagePoints2, cv::NORM_L2); // 歪み補正前に射影

    size_t n = imagePoints3.size();
    totalErr        += err*err;
    totalPoints     += n;

    double totalAvgErr = std::sqrt(totalErr/totalPoints);

    // 6. 4で求めたコーナー座標(3D)の間隔をチェックし、実際の距離との誤差の平均を確認する

    double sum = 0;
    int count = 0;
    for (int i = 0; i < ROW - 1; i++) {
        for (int j = 0; j < COL; j++) {
            int index = i * COL + j;
            int index_y = (i + 1) * COL + j;

            double h = sqrt(pow((double)(imagePoints3[index_y].y) - (double)(imagePoints3[index].y), 2) + pow((double)(imagePoints3[index_y].x) - (double)(imagePoints3[index].x), 2));
            sum += h;
            count++;
        }
    }

    for (int i = 0; i < ROW ; i++) {
        for (int j = 0; j < COL-1; j++) {
            int index = i * COL + j;
            int index_x = i * COL + (j + 1);

            double w = sqrt(pow((double)(imagePoints3[index_x].x) - (double)(imagePoints3[index].x),2) + pow((double)(imagePoints3[index_x].y) - (double)(imagePoints3[index].y), 2));
            sum += w;
            count++;
        }
    }
    double delta = abs(CHESS_SIZE - sum / count);

    // 8.再投影誤差を描画して保存する
    ostringstream ss;
    string text1,text2,text3;

    // 再投影誤差を表示
    ss << totalAvgErr;
    text1 = "Reprojection Error:" + ss.str();
    ss.str(""); 
    ss.clear(stringstream::goodbit);
    // コーナー座標(3D)の間隔と、実際の距離との誤差の平均を表示
    ss << delta;
    text2 = "BetweenCorner Distance Error:" + ss.str();
    ss.str(""); 
    ss.clear(stringstream::goodbit);
    // チェスの１マスの大きさを表示
    ss << CHESS_SIZE;
    text3 = "chess size:" + ss.str();

    cv::putText(img, "z = 850", cv::Point(25, 25), 1, cv::FONT_HERSHEY_DUPLEX, cv::Scalar(255, 255, 255), 1);
    cv::putText(img, text1, cv::Point(25, 45), 1, cv::FONT_HERSHEY_DUPLEX, cv::Scalar(255, 255, 255), 1);
    cv::putText(img, text2, cv::Point(25, 65), 1, cv::FONT_HERSHEY_DUPLEX, cv::Scalar(255, 255, 255), 1);
    cv::putText(img, text3, cv::Point(25, 85), 1, cv::FONT_HERSHEY_DUPLEX, cv::Scalar(255, 255, 255),1);
    cv::imwrite("enshuu03_02.png", img);

    return 0;
}
