#include <iostream>
#include <creekQrCodeDetector.h>

int main()
{
  cv::VideoCapture video(0);
  cv::Mat src, finder, vertices;

  creek::creekQrCodeDetector dec;
  //dec.debug(true);
  //dec.set(5);

  cv::namedWindow("src", CV_WINDOW_AUTOSIZE|CV_WINDOW_FREERATIO);
  cv::namedWindow("finder", CV_WINDOW_AUTOSIZE|CV_WINDOW_FREERATIO);
  cv::namedWindow("vertices", CV_WINDOW_AUTOSIZE|CV_WINDOW_FREERATIO);
  cv::namedWindow("QR", CV_WINDOW_AUTOSIZE|CV_WINDOW_FREERATIO);

  int key(0);
  while(key != 'q') {
    video >> src;
    if( src.empty() )
      return 0;
    finder = src.clone();
    //vertices = cv::Mat::zeros(src.rows, src.cols, CV_8U);
    vertices = src.clone();

    if( dec.detectQrCode(src, 0.7) ) {
      dec.drawFinderPattern(finder);
      dec.drawVertices(vertices);

      std::cout << "center : " << dec.getCenter().x << ", " << dec.getCenter().y << std::endl;
    }
 

    cv::imshow("src", src);
    cv::imshow("finder", finder);
    cv::imshow("vertices", vertices);
    cv::imshow("QR", dec.getQrImage(src, 0));

    key = cv::waitKey(100) & 255;
  }
  return 0;
}
