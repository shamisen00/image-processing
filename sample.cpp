#include <opencv2/opencv.hpp>

int main(int argc, char* argv[]) {

  cv::Mat image(256, 256, CV_8UC3);

  cv::String text = "Hello, world";
  cv::Point org(0, 100);
  int fontFace = cv::FONT_HERSHEY_SIMPLEX;
  double fontScale = 1.0;
  cv::Scalar color(0, 255, 127);
  cv::putText(image, text, org, fontFace, fontScale, color);

  cv::String path = "hello-world.png";
  cv::imwrite(path, image);

  return 0;
}