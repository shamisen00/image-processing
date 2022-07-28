#include <opencv2/opencv.hpp>
#include <iostream>
#include "opencv2/core.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
using namespace cv;
using namespace cv::xfeatures2d;
using std::cout;
using std::endl;
 
int main(void)
{
	cv::Mat src1, src2, dst;
 
	src1 = cv::imread("B.png");
	src2 = cv::imread("C.png");
 
	// 特徴点検出アルゴリズムの選択
    cv::Ptr<cv::FeatureDetector> akaze = cv::AKAZE::create();
 
	std::vector<cv::KeyPoint> key1, key2;
 
	// キーポイントの検出
    akaze->detect(src1, key1);
    akaze->detect(src2, key2);
 
	// 特徴量記述の計算
    cv::Ptr<cv::ORB>  orb = cv::ORB::create();

	cv::Mat des1, des2;
	orb->compute(src1, key1, des1);
	orb->compute(src2, key2, des2);
 
	// 特徴点マッチングアルゴリズム BF
	cv::Ptr<cv::DescriptorMatcher> hamming = cv::DescriptorMatcher::create("BruteForce-Hamming");
 
	// 特徴点マッチング
	std::vector<cv::DMatch> match12, match21;
	hamming->match(des1, des2, match12);
    hamming->match(des2, des1, match21);
    
    // クロスチェック
    std::vector<cv::DMatch> match;

    for (size_t i = 0; i < match12.size(); ++i)
    {
        cv::DMatch m12 = match12[i];
        cv::DMatch m21 = match21[m12.trainIdx];

        if (m21.trainIdx == m12.queryIdx)
        match.push_back( m12 );
    }

    cv::drawMatches(src1, key1, src2, key2, match, dst);
    cv::imwrite("matched.png", dst);

    std::vector<Point2f> obj;
    std::vector<Point2f> scene;

    for( size_t i = 0; i < match.size(); i++ )
    {
        //-- Get the keypoints from the good matches
        obj.push_back( key1[ match[i].queryIdx ].pt );
        scene.push_back( key2[ match[i].trainIdx ].pt );
    }

    // ホモグラフィー変換
    Mat H = findHomography(scene, obj, RANSAC );

    Mat im_out;

    //ホモグラフィの通り回転の必要な画像を回転する

    warpPerspective(src2, im_out, H, Size(static_cast<int>(src2.cols * 2), static_cast<int>(src2.rows * 1.5)));

    cv::imwrite("WarpedImage.png", im_out);
    
    // 画像重畳
    for (int y = 0; y < src1.rows; y++){
        for (int x = 0; x < src1.cols; x++){
            im_out.at<Vec3b>(y, x) = 0.5 * src1.at<Vec3b>(y, x) + 0.5 * im_out.at<Vec3b>(y, x);
        }
    }

    cv::imwrite("panorama.png", im_out);
    
    // std::vector<Point2f> obj_corners(4);
    // obj_corners[0] = Point2f(0, 0);
    // obj_corners[1] = Point2f( (float)src1.cols, 0 );
    // obj_corners[2] = Point2f( (float)src1.cols, (float)src1.rows );
    // obj_corners[3] = Point2f( 0, (float)src1.rows );
    // std::vector<Point2f> scene_corners(4);

    // perspectiveTransform( obj_corners, scene_corners, H);

    // //-- Draw lines between the corners (the mapped object in the scene - image_2 )
    // Mat img_matches;
    // cv::drawMatches(src1, key1, src2, key2, match, img_matches, Scalar::all(-1),
    //             Scalar::all(-1), std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

    // cv::line( img_matches, scene_corners[0] + Point2f((float)src1.cols, 0),
    //       scene_corners[1] + Point2f((float)src1.cols, 0), Scalar(0, 255, 0), 4 );
    // cv::line( img_matches, scene_corners[1] + Point2f((float)src1.cols, 0),
    //       scene_corners[2] + Point2f((float)src1.cols, 0), Scalar( 0, 255, 0), 4 );
    // cv::line( img_matches, scene_corners[2] + Point2f((float)src1.cols, 0),
    //       scene_corners[3] + Point2f((float)src1.cols, 0), Scalar( 0, 255, 0), 4 );
    // cv::line( img_matches, scene_corners[3] + Point2f((float)src1.cols, 0),
    //       scene_corners[0] + Point2f((float)src1.cols, 0), Scalar( 0, 255, 0), 4 );
    // //-- Show detected matches
    // //cv::imshow("Good Matches & Object detection", img_matches );
    // cv::imwrite("test2.png", img_matches);
	// //cv::drawMatches(src1, key1, src2, key2, match, dst);
 
	return 0;
}