// -*- C++ -*-

#include "CameraPort.h"

CameraPort::CameraPort(const char *name)
  : RTC::InPort<Img::TimedCameraImage>(name, m_image),
    m_frame( cv::Size(640,480), CV_8UC3)
{
  m_zbar.set_format("Y800");
}


bool CameraPort::read()
{
  if( !RTC::InPort<Img::TimedCameraImage>::read() )
    return false;

  // size check
  int a = m_frame.total()*m_frame.channels();
  int b = m_image.data.image.raw_data.length();
  if( a != b ) {
    switch(m_image.data.image.format)
      {
      case Img::CF_GRAY:
	m_frame = cv::Mat::zeros(m_image.data.image.height, m_image.data.image.width, CV_8UC1);
	break;
      case Img::CF_RGB:
	m_frame = cv::Mat::zeros(m_image.data.image.height, m_image.data.image.width, CV_8UC3);
	break;
      default:
	return false;
      }
    std::cout << "CameraPort : resize image data (opecv)" << std::endl;
  }


  // copy original image
  memcpy(m_frame.data,
	 m_image.data.image.raw_data.get_buffer(),
	 m_image.data.image.raw_data.length());


  // convert image
  switch(m_image.data.image.format)
    {
    case Img::CF_GRAY:
      m_gray = m_frame;
      break;
    case Img::CF_RGB:
      cv::cvtColor(m_frame, m_frame, CV_RGB2BGR);
      cv::cvtColor(m_frame, m_gray, CV_BGR2GRAY);
      break;
    default:
      return false;
    }


  // size check (zbar)
  if( m_gray.total() != m_zbar.get_data_length() ) {
    m_zbar.set_size(m_gray.size().width, m_gray.size().height);
    m_zbar.set_data(m_gray.data, m_gray.total());
    std::cout << "CameraPort : resize image data (zbar)" << std::endl;
  }


  return true;
}
