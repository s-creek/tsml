#include <iostream>
#include <creekQrCodeDetector.h>
#include <zbar.h>

int main()
{
  cv::VideoCapture video(0);
  cv::Mat src, finder, gray, qr0, qr1;

  creek::creekQrCodeDetector dec;
  //dec.debug(true);
  //dec.set(5);

  zbar::Image m_zbar(256, 256, "Y800", dec.getQrImage().data, 256*256);
  zbar::ImageScanner m_scanner;
  m_scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);


  cv::namedWindow("src", CV_WINDOW_AUTOSIZE|CV_WINDOW_FREERATIO);
  cv::namedWindow("finder", CV_WINDOW_AUTOSIZE|CV_WINDOW_FREERATIO);
  cv::namedWindow("QR pers", CV_WINDOW_AUTOSIZE|CV_WINDOW_FREERATIO);
  cv::namedWindow("QR affin", CV_WINDOW_AUTOSIZE|CV_WINDOW_FREERATIO);


  std::string orgstr, qrstr;
  int key(0);
  while(key != 'q') {
    video >> src;
    if( src.empty() )
      return 0;
    //finder = src.clone();
    finder = cv::Mat::zeros(src.rows, src.cols, CV_8U);


    if( dec.detectQrCode(src, 0.7, 0) ) {
      dec.drawFinderPattern(finder);
      dec.drawVertices(finder);
    }


    cv::cvtColor(src, gray, CV_BGR2GRAY);
    m_zbar.set_size(gray.cols, gray.rows);
    m_zbar.set_data(gray.data, gray.total());
    if( m_scanner.scan(m_zbar) != 0 ) {
      for(zbar::Image::SymbolIterator symbol = m_zbar.symbol_begin(); symbol != m_zbar.symbol_end(); ++symbol) {
	//std::cout << "org : type = " << symbol->get_type_name() << ",  type = " << symbol->get_type() << ",  data = " << symbol->get_data() << std::endl;
	orgstr = symbol->get_data();
      }
    }


    qr0 = dec.getQrImage(src, 0);
    m_zbar.set_size(qr0.cols, qr0.rows);
    m_zbar.set_data(qr0.data, qr0.total());
    if( m_scanner.scan(m_zbar) != 0 ) {
      for(zbar::Image::SymbolIterator symbol = m_zbar.symbol_begin(); symbol != m_zbar.symbol_end(); ++symbol) {
	//std::cout << "QR0 : type = " << symbol->get_type_name() << ",  type = " << symbol->get_type() << ",  data = " << symbol->get_data() << std::endl;
	qrstr = symbol->get_data();
      }
    }

 
    qr1 = dec.getQrImage(src, 1);
    m_zbar.set_size(qr1.cols, qr1.rows);
    m_zbar.set_data(qr1.data, qr1.total());
    if( m_scanner.scan(m_zbar) != 0 ) {
      for(zbar::Image::SymbolIterator symbol = m_zbar.symbol_begin(); symbol != m_zbar.symbol_end(); ++symbol) {
	//std::cout << "QR1 : type = " << symbol->get_type_name() << ",  type = " << symbol->get_type() << ",  data = " << symbol->get_data() << std::endl;
	
      }
    }
    if( orgstr != qrstr ) {
      std::cout << "error" << std::endl;
      std::cout << "  org = " << orgstr << std::endl;
      std::cout << "  qr0 = " << qrstr << std::endl;
    }
    else {
      std::cout << "ok" << std::endl;
      std::cout << "  org = " << orgstr << std::endl;
      std::cout << "  qr0 = " << qrstr << std::endl;
    }


    cv::imshow("src", src);
    cv::imshow("finder", finder);
    cv::imshow("QR pers", qr0);
    cv::imshow("QR affin", qr1);

    key = cv::waitKey(100) & 255;
  }
  return 0;
}
