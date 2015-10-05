// -*- C++ -*-

#include "creekCameraViewer.h"

// Module specification
static const char* creekcameraviewer_spec[] =
  {
    "implementation_id", "creekCameraViewer",
    "type_name",         "creekCameraViewer",
    "description",       "creekCameraViewer",
    "version",           "1.0",
    "vendor",            "s-creek",
    "category",          "example",
    "activity_type",     "SPORADIC",
    "kind",              "creekCameraViewer",
    "max_instance",      "0",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    ""
  };


template <class T> double toSec(T t)
{
  return t.sec + t.nsec / 1000000000.0;
}


creekCameraViewer::creekCameraViewer(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    m_creekCameraViewerServicePort("creekCameraViewerService")
{
  m_service0.setComponent(this);
}

creekCameraViewer::~creekCameraViewer()
{
}


RTC::ReturnCode_t creekCameraViewer::onInitialize()
{
  std::cout << "creekCameraViewer : onInitialize" << std::endl;

  m_creekCameraViewerServicePort.registerProvider("service0", "creekCameraViewerService", m_service0);
  addPort(m_creekCameraViewerServicePort);


  RTC::Properties& prop = getProperties();
  coil::vstring camera_name_list = coil::split( prop["CAMERA_NAME_LIST"], ",");
  int n = camera_name_list.size();
  m_nameToIndex.clear();
  m_searchFlag.resize(n);

  m_maxSeqNum = 35;
  m_tcsSeq.resize(n);


  // image port
  m_ports.resize(n);
  for(int i=0; i<n; i++) {
    m_ports[i] = new CameraPort(camera_name_list[i].c_str());
    addInPort(camera_name_list[i].c_str(), *m_ports[i]);

    m_nameToIndex[ camera_name_list[i] ] = i;
    m_searchFlag[i] = false;
  }

  // pose port
  m_cameraPose.resize(n);
  m_cameraPoseIn.resize(n);
  for(int i=0; i<n; i++) {
    std::string name = camera_name_list[i] + "Pose";
    m_cameraPoseIn[i] = new InPort<TimedPose3D>(name.c_str(), m_cameraPose[i]);
    addInPort(name.c_str(), *m_cameraPoseIn[i]);
  }


  // init zbar
  m_scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);


  return RTC::RTC_OK;
}


RTC::ReturnCode_t creekCameraViewer::onActivated(RTC::UniqueId ec_id)
{
  std::cout << "creekCameraViewer : onActivated" << std::endl;

  for(int i=0; i<m_tcsSeq.size(); i++) {
    m_tcsSeq[i].clear();
  }
  m_qrDataSet.clear();

  return RTC::RTC_OK;
}


RTC::ReturnCode_t creekCameraViewer::onDeactivated(RTC::UniqueId ec_id)
{
  std::cout << "creekCameraViewer : onDeactivated" << std::endl;
  return RTC::RTC_OK;
}


RTC::ReturnCode_t creekCameraViewer::onExecute(RTC::UniqueId ec_id)
{
  // get camera pose
  for(int i=0; i<m_cameraPoseIn.size(); i++) {
    if( m_cameraPoseIn[i]->isNew() ) {
      m_cameraPoseIn[i]->read();

      TimedCoordinateSystem tmp;
      tmp.tm = toSec(m_cameraPose[i].tm);
      tmp.p << m_cameraPose[i].data.position.x, m_cameraPose[i].data.position.y, m_cameraPose[i].data.position.z;
      tmp.R = cnoid::rotFromRpy(m_cameraPose[i].data.orientation.r, m_cameraPose[i].data.orientation.p, m_cameraPose[i].data.orientation.y);

      // logging
      m_tcsSeq[i].push_back(tmp);
      if( m_tcsSeq[i].size() > m_maxSeqNum ) {
	m_tcsSeq[i].pop_front();
      }
    }
  }


  // get image data
  for(int i=0; i<m_ports.size(); i++) {
    if( m_ports[i]->isNew() ) {
      m_ports[i]->read();

      if( m_scanner.scan(m_ports[i]->m_zbar) != 0 ) {
	bool closeFlag(false);
	for(zbar::Image::SymbolIterator symbol = m_ports[i]->m_zbar.symbol_begin(); symbol != m_ports[i]->m_zbar.symbol_end(); ++symbol) {

	  if( symbol->get_type() == zbar::ZBAR_QRCODE ) {

	    // get center of QrCode
	    cv::Point2f center;  center.x = 0.0;  center.y = 0.0;
	    int n = symbol->get_location_size();
	    for(int j=0; j<n; j++) {
	      center.x += symbol->get_location_x(j);
	      center.y += symbol->get_location_y(j);
	    }
	    center.x = center.x / (float)n;
	    center.y = center.y / (float)n;

	    // draw QrCode
	    if( m_service0.draw() ) {
	      std::vector<cv::Point2f> points(n);
	      for(int j=0; j<n; j++) {
		points[j].x = symbol->get_location_x(j);
		points[j].y = symbol->get_location_y(j);
	      }
	      drawFrame(m_ports[i]->m_frame, points, center);
	    }


	    if( m_searchFlag[i] ) {

	      // pixel to vector
	      cnoid::Vector3 e = pixelToVector(center.x, center.y);
	      cnoid::Vector3 cp, ce;  cnoid::Matrix3 cR;
	      getCameraPose( toSec(m_ports[i]->m_image.tm), i, cp, cR);
	      ce = cR * e;

	      // calc QrCode position
	      std::map< std::string, QrCodeData >::iterator qrDataIt = m_qrDataSet.find( symbol->get_data() );
	      if( qrDataIt == m_qrDataSet.end() ) {
		QrCodeData qrdata;
		qrdata.pos  << 0,0,0;
		qrdata.data = symbol->get_data();
		qrdata.calc = false;
		qrdata.p1 = cp;
		qrdata.e1 = ce;
		qrdata.p2 << 0,0,0;
		qrdata.e2 << 0,0,0;
		qrdata.name1 = m_ports[i]->name();
		qrdata.name2 = "";
		m_qrDataSet[ symbol->get_data() ] = qrdata;

		std::cout << m_ports[i]->name() << " find QrCode" << std::endl;

		closeFlag = true;
	      }
	      else if( !qrDataIt->second.calc ) {
		cnoid::Vector3 qrposition(0, 0, 0);
		if( intersectLines( qrDataIt->second.p1, qrDataIt->second.e1, cp, ce, qrposition ) ) {
		  qrDataIt->second.pos  = qrposition;
		  qrDataIt->second.calc = true;
		  qrDataIt->second.p2   = cp;
		  qrDataIt->second.e2   = ce;
		  qrDataIt->second.name2 = m_ports[i]->name();

		  closeFlag = true;

		  std::cout << "success (" << qrDataIt->second.name1 << ", " << m_ports[i]->name() << " )" << std::endl;
		  std::cout << "  data = " << qrDataIt->first << std::endl;
		  std::cout << "  pos = " << qrposition.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << std::endl;
		}
		// std::cout << m_ports[i]->name() << " find QrCode" << std::endl;
		// Eigen::IOFormat IO(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]");
		// std::cout << qrDataIt->second.p1.format(IO) << std::endl;
		// std::cout << qrDataIt->second.e1.format(IO) << std::endl;
		// std::cout << cp.format(IO) << std::endl;
		// std::cout << ce.format(IO) << std::endl << std::endl;
	      }
	    }
	  }
	}  // end of for loop (zbar symbol iterator)
	if( closeFlag )
	  m_searchFlag[i] = false;
      }
    }
  }

  combineImage();
  cv::imshow("all", m_allImage);
  //cv::waitKey(1);

  return RTC::RTC_OK;
}


//
//  y              +---------x
//  |              |
//  | cnoid        |  OpenCV
//  |              |
//  +---------x    y
//
cnoid::Vector3 creekCameraViewer::pixelToVector(double x, double y)
{
  double width(640), height(480), fieldOfView(1.0);

  double dist = height / 2.0 / tan(fieldOfView/2.0);
  double cx = (width-1) / 2.0;   // center x
  double cy = (height-1) / 2.0;  // center y

  double vx =  x - cx;
  double vy = -y + cy;

  cnoid::Vector3 vec(vx, vy, -dist);
  vec.normalize();
  return vec;
}


bool creekCameraViewer::intersectLines(const cnoid::Vector3 &p1, const cnoid::Vector3 &e1, const cnoid::Vector3 &p2, const cnoid::Vector3 &e2, cnoid::Vector3 &out)
{
  double limitRate(0.0005);
  double limitErr(0.1);

  cnoid::Vector3 n1(e1), n2(e2);
  n1.normalize();
  n2.normalize();

  double a = n1.dot(n2);
  double b = 1 - a*a;

  if( b < 1.0e-12 )
    return false;

  cnoid::Vector3 d(p2-p1);
  
  double d1 = ( d.dot(n1) - a*d.dot(n2) ) / b;
  double d2 = ( a*d.dot(n1) - d.dot(n2) ) / b;

  cnoid::Vector3 r1 = p1 + d1 * n1;
  cnoid::Vector3 r2 = p2 + d2 * n2;

  out = (r1+r2)/2.0;


  if( b < limitRate ) {
    //std::cout << "b = " << b << std::endl;
    return false;
  }
  if( (r1-r2).norm() > limitErr ) {
    //std::cout << "err = " << (r1-r2).norm() << std::endl;
    return false;
  }
}


bool creekCameraViewer::getCameraPose(double tm, int index, cnoid::Vector3 &p, cnoid::Matrix3 &R)
{
  bool ret(false);
  for( std::deque<TimedCoordinateSystem>::reverse_iterator it = m_tcsSeq[index].rbegin(); it != m_tcsSeq[index].rend(); it++) {
    if( it->tm <= tm ) {
      p = it->p;
      R = it->R;
      m_tcsSeq[index].erase(m_tcsSeq[index].begin(), it.base());
      ret = true;
      break;
    }
  }
  return ret;
}


void creekCameraViewer::drawFrame( cv::Mat &in_src, std::vector<cv::Point2f> &points, cv::Point2f &center )
{
  for(int i=0; i<points.size(); i++) {
    int j = i+1;
    if( j >= points.size())
      j = 0;
    cv::line( in_src,points[i], points[j], cv::Scalar(150,0,255), 1, 8, 0 );
  }
  cv::circle( in_src, center, 4, cv::Scalar(0,0,255), -1, 8, 0 );
}


void creekCameraViewer::combineImage()
{
  int n = m_ports.size();
  int nx = ceil( sqrt(n) );
  int ny = ceil( double(n)/double(nx) );

  int frameX = m_ports[0]->m_frame.cols;
  int frameY = m_ports[0]->m_frame.rows;
  m_allImage = cv::Mat::zeros( cv::Size(frameX*nx + (nx-1), frameY*ny + (ny-1) ), CV_8UC3);

  //std::cout << "all image" << m_allImage.cols << "x" << m_allImage.rows << std::endl;
  cv::Rect roi_rect;
  int i=0;
  for(int y=0; y<ny; y++) {
    for(int x=0; x<nx; x++) {
      if( i < n ) {
	roi_rect.width  = m_ports[i]->m_frame.cols;
	roi_rect.height = m_ports[i]->m_frame.rows;
	cv::Mat roi(m_allImage, roi_rect);
	m_ports[i]->m_frame.copyTo(roi);
	roi_rect.x += m_ports[i]->m_frame.cols + 1;
      }
      i++;
    }
    if( i < n ) {
      roi_rect.x  = 0;
      roi_rect.y += m_ports[i]->m_frame.rows + 1;
    }
  }
}


//--------------------------------------------------------------------------------------------


void creekCameraViewer::setSearchFlag(std::string &list)
{
  for( int j=0; j<m_searchFlag.size(); j++) {
    m_searchFlag[j] = false;
  }
  

  coil::vstring camera_name_list = coil::split( list, ",");
  for(int i=0; i<camera_name_list.size(); i++) {
    if( camera_name_list[i] == "all" ) {
      for( int j=0; j<m_searchFlag.size(); j++) {
	m_searchFlag[j] = true;
      }
      break;
    }

    std::map<std::string, int>::const_iterator p = m_nameToIndex.find(camera_name_list[i]);
    if( p != m_nameToIndex.end() ) {
      m_searchFlag[p->second] = true;
    }
  }
}


void creekCameraViewer::show()
{
  for( std::map< std::string, QrCodeData >::iterator it = m_qrDataSet.begin(); it != m_qrDataSet.end(); it++ ) {
    std::cout << it->second.data << ", " 
	      << it->second.calc << ", " 
	      << it->second.pos.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << std::endl;
  }
}


void creekCameraViewer::saveData(const char *path)
{
  std::ofstream ofs(path);
  if( ofs ) {
    for( std::map< std::string, QrCodeData >::iterator it = m_qrDataSet.begin(); it != m_qrDataSet.end(); it++ ) {
      ofs << it->second.data << "," << it->second.pos(0) << "," << it->second.pos(1) << "," << it->second.pos(2) << std::endl;
    }
  }
  ofs.close();
}


extern "C"
{
 
  void creekCameraViewerInit(RTC::Manager* manager)
  {
    coil::Properties profile(creekcameraviewer_spec);
    manager->registerFactory(profile,
                             RTC::Create<creekCameraViewer>,
                             RTC::Delete<creekCameraViewer>);
  }
  
};



