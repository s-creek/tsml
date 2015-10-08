#include <iostream>
#include <creekQrCodeDetector.h>

int main( int argc, char **argv )
{
  if( argc < 2 )
    return 0;

  cv::Mat src = cv::imread(argv[1]);
  cv::Mat tmp = src.clone();

  creek::creekQrCodeDetector dec;
  dec.debug(true);  
  if( !dec.detectQrCode(src) )
    return 0;

  cv::namedWindow("src", CV_WINDOW_AUTOSIZE|CV_WINDOW_FREERATIO);
  cv::namedWindow("finder", CV_WINDOW_AUTOSIZE|CV_WINDOW_FREERATIO);
  cv::namedWindow("QR pers", CV_WINDOW_AUTOSIZE|CV_WINDOW_FREERATIO);
  cv::namedWindow("QR affin", CV_WINDOW_AUTOSIZE|CV_WINDOW_FREERATIO);

  int key(0), num(0);
  while(key != 'q') {
    num++;
    tmp = src.clone();
    dec.drawFinderPattern(tmp, num);
    dec.drawVertices(tmp);
    std::cout << "finder index ( 0 : " << (num-1) << " )" << std::endl;
    num = num%3;

    cv::imshow("src", src);
    cv::imshow("finder", tmp);
    cv::imshow("QR pers", dec.getQrImage());
    cv::imshow("QR affin", dec.getQrImage(src, 1));

    key = cv::waitKey(0) & 255;
  }
  return 0;
}
