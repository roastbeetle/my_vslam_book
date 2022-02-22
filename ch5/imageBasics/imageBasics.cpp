#include <iostream>
#include <chrono>

using namespace std;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

/// Opencv 이미지 기초 예제

int main(int argc, char **argv) {
  cv::Mat image;
  image = cv::imread(argv[1]);

  /// Read Fail
  if (image.data == nullptr) { 
    cerr << "Image read fail" << endl;
    return 0;
  }

  /// Read Success
  cout << "Width: " << image.cols << ", Height: " << image.rows << ", Channel: " << image.channels() << endl;
  cv::imshow("image", image);
  cv::waitKey(0);

  /// Check Image type
  if (image.type() != CV_8UC1 && image.type() != CV_8UC3) {
    cout << "Wrong Image type: Image must be CV_8UC1 or CV_8UC3. " << endl;
    return 0;
  }

  /// 픽셀 데이터 접근
  chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
  for (size_t y = 0; y < image.rows; y++) {
    unsigned char *row_ptr = image.ptr<unsigned char>(y);
    for (size_t x = 0; x < image.cols; x++) {
      unsigned char *data_ptr = &row_ptr[x * image.channels()];
      for (int c = 0; c != image.channels(); c++) {
        unsigned char data = data_ptr[c];
      }
    }
  }

  chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
  chrono::duration<double> time_used = chrono::duration_cast < chrono::duration < double >> (t2 - t1);
  cout << "Check Time ：" << time_used.count() << " seconds." << endl;

  /// 이미지의 포인터를 복사 -> 원본 변형 O
  cv::Mat image_another = image;
  cv::imshow("image1", image);
  image_another(cv::Rect(0, 0, 100, 100)).setTo(0); // 100*100 검은색 추가
  cv::imshow("image2", image); 
  cv::waitKey(0);

  /// 이미지 복사 -> 원본 변형 X
  cv::Mat image_clone = image.clone();
  image_clone(cv::Rect(0, 0, 100, 100)).setTo(255); // 100*100* 흰색 추가
  cv::imshow("image_origin", image);
  cv::imshow("image_clone", image_clone);
  cv::waitKey(0);

  cv::destroyAllWindows();
  return 0;
}
