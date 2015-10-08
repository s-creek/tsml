#include "creekQrCodeDetector.h"
#include <stdexcept>

using namespace creek;
enum { QR_NE, QR_SE, QR_SW, QR_NW, QR_CENTER };

creekQrCodeDetector::creekQrCodeDetector(int in_requiredChildNum)
  : m_requiredChildNum(in_requiredChildNum),
    m_out(256, 256, CV_8UC1, cv::Scalar(0)),
    m_intersection(-1.0, -1.0),
    m_debug(false)
{
}


void creekQrCodeDetector::drawFinderPattern(cv::Mat &in_src, int num)
{
  static std::vector< cv::Scalar > colors = {cv::Scalar(0, 255, 0),
					     cv::Scalar(0, 0, 255),
					     cv::Scalar(200, 0, 200),
					     cv::Scalar(255, 0, 0),
					     cv::Scalar(0, 200, 200),
					     cv::Scalar(200, 200, 0),
					     cv::Scalar(0, 0, 100),
					     cv::Scalar(0, 100, 0),
					     cv::Scalar(100, 0, 0)};

  int size = m_index.size();
  if( num > 0 ) {
    size = std::min(size, num);
  }
  for(int i=0; i<size; i++) {
    std::vector<int> tmp = m_index[i];
    for(int j=0; j<tmp.size(); j++) {
      cv::drawContours(in_src, m_contours, tmp[j], colors[j], 2, CV_AA, m_hierarchy, 0);
    }
  }
}


void creekQrCodeDetector::drawVertices(cv::Mat &in_src, int num)
{
  int size = 3;
  if( num > 0 ) {
    size = std::min(size, num);
  }
  for(int i=0; i<size; i++) {
    for(int j=0; j<4; j++) {
      cv::circle( in_src, m_vertices[i][j], 3, cv::Scalar(200,200,0), -1, 8, 0 );
    }
  }
  cv::circle( in_src, m_intersection, 3, cv::Scalar(255,0,0), -1, 8, 0 );

  cv::line( in_src, m_vertices[1][1], m_intersection, cv::Scalar(150,0,255), 1, 8, 0 );
  cv::line( in_src, m_vertices[2][3], m_intersection, cv::Scalar(150,0,255), 1, 8, 0 );
  
  cv::circle( in_src, m_center, 6, cv::Scalar(0,0,255), -1, 8, 0 );
}


//-------------------------------------------------------------------------------------------------


bool creekQrCodeDetector::detectQrCode(cv::Mat &in_src, double in_th, int in_method)
{
  cv::cvtColor(in_src, m_gray, CV_BGR2GRAY);
  cv::Canny(m_gray, m_edge, 100, 200, 3, true);
  m_intersection = cv::Point2f(-1.0, -1.0);  // temporary

  // detect finder-pattern ( corner square )  
  if( !detectFinderPattern(in_th) ) {
    if(m_debug) drawFinderPattern(in_src);
    return false;
  }

  // align finder-pattern index
  // * orientation is old value, now in use for debug.
  int orientation = align();
  if( orientation == QR_CENTER ) {
    if(m_debug) std::clog << "[detectQrCode] error : align()" << std::endl;
    return false;
  }

  // get finder-pattern vertices and align index
  getVertices(orientation);

  // get last corner
  if( !getIntersectionPoint() ) {
    if(m_debug) std::clog << "[detectQrCode] error : getIntersectionPoint()" << std::endl;
    return false;
  }

  // get QR-code image
  if( in_method == 0 ) {
    if( !warpImage(in_src) ) {
      if(m_debug) std::clog << "[detectQrCode] error : warpImage()" << std::endl;
      return false;
    }
  }
  else {
    if( !cropImage(in_src) ) {
      if(m_debug) std::clog << "[detectQrCode] error : cropImage()" << std::endl;
      return false;
    }
  }
  return true;
}


cv::Mat creekQrCodeDetector::getQrImage(cv::Mat &in_src, int in_method)
{
  // get QR-code image
  if( in_method == 0 ) {
    warpImage(in_src);
  }
  else {
    cropImage(in_src);
  }
  return m_out.clone();
}


bool creekQrCodeDetector::detectFinderPattern(double in_th)
{
  m_contours.clear();
  m_hierarchy.clear();
  cv::findContours( m_edge, m_contours, m_hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE );

  m_index.clear();
  for(int i=0; i<m_hierarchy.size(); i++) {
    int k=i, c=0;
    
    // find child end
    while(m_hierarchy[k][2] != -1) {
      k = m_hierarchy[k][2];
      c++;
    }
    if( m_hierarchy[k][0] != -1 )
      c = 0;

    // check child num
    if( c >= m_requiredChildNum ) {
      // create index (temporary)
      std::vector<int> tmp;
      for(int j=0; j<m_requiredChildNum; j++) {
	tmp.push_back(k);
	k = m_hierarchy[k][3];
      }

      // check duplication
      if( std::find(m_index.begin(), m_index.end(), tmp) == m_index.end() ) {
	// check rectangle
	if( minAreaRatio(tmp) > in_th )
	  m_index.push_back(tmp);
      }
    }
  }
  if(m_debug) std::clog << "[detectFinderPattern] find num = " << m_index.size() << std::endl;
  return (m_index.size() == 3);
}


int creekQrCodeDetector::align()
{
  // calc center of mass
  std::vector<cv::Moments> mu(m_index.size());
  std::vector<cv::Point2f> mc(m_index.size());

  for(int i=0; i<m_index.size(); i++) {
    int index = m_index[i].back();
    //mu[i] = cv::moments(m_contours[index]);
    mu[i] = cv::moments( cv::Mat(m_contours[index]) );
    mc[i] = cv::Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 );
  }


  // each distance
  enum {A, B, C};
  double dAB, dBC, dCA;
  dAB = distance(mc[A], mc[B]);
  dBC = distance(mc[B], mc[C]);
  dCA = distance(mc[C], mc[A]);
  if(m_debug) std::clog << "[align] AB = " << dAB << ",  BC = " << dBC << ",  CA = " << dCA << std::endl;


  // finder index
  //
  //  0------1
  //  |
  //  |
  //  2
  //
  // align
  std::vector< std::vector<int> > old_index(m_index.begin(), m_index.end());
  if ( dAB > dBC && dAB > dCA )
    {
      cv::Point2f CA, CB;
      CA = mc[A] - mc[C];
      CB = mc[B] - mc[C];

      m_index[0] = old_index[C];
      if( cross(CA, CB) > 0 ) {
	m_index[1] = old_index[A];
	m_index[2] = old_index[B];
	m_top = mc[A] - mc[C];
      }
      else {
	m_index[1] = old_index[B];
	m_index[2] = old_index[A];
	m_top = mc[B] - mc[C];
      }
    }
  else if ( dCA > dAB && dCA > dBC )
    {
      cv::Point2f BA, BC;
      BA = mc[A] - mc[B];
      BC = mc[C] - mc[B];

      m_index[0] = old_index[B];
      if( cross(BA, BC) > 0 ) {
	m_index[1] = old_index[A];
	m_index[2] = old_index[C];
	m_top = mc[A] - mc[B];
      }
      else {
	m_index[1] = old_index[C];
	m_index[2] = old_index[A];
	m_top = mc[C] - mc[B];
      }
    }
  //else if ( dBC > dAB && dBC > dCA )
  else
    {
      cv::Point2f AB, AC;
      AB = mc[B] - mc[A];
      AC = mc[C] - mc[A];

      m_index[0] = old_index[A];
      if( cross(AB, AC) > 0 ) {
	m_index[1] = old_index[B];
	m_index[2] = old_index[C];
	m_top = mc[B] - mc[A];
      }
      else {
	m_index[1] = old_index[C];
	m_index[2] = old_index[B];
	m_top = mc[C] - mc[A];
      }
    }
  return getOrientation();
}


void creekQrCodeDetector::getVertices(int in_orientation)
{
  for(int i=0; i<3; i++) {
    int index = m_index[i].back();
    cv::Point2f tmp[4];

    std::vector<cv::Point> approx;
    cv::approxPolyDP(cv::Mat(m_contours[index]), approx, 5.0, true);
    if(m_debug) {
      std::clog << "[getVertices] aaprox size = " << approx.size() << std::endl;
      for(int j=0; j<approx.size(); j++) {
	std::clog << "  " << approx[j] << std::endl;
      }
    }
    if( approx.size() == 4 && true ) {
      for(int j=0; j<4; j++) {
	tmp[j] = approx[j];
      }
    }
    else {
      cv::RotatedRect rect = cv::minAreaRect( cv::Mat(m_contours[index]) );
      rect.points(tmp);
      std::clog << "[getVertices] aaprox size = " << approx.size() << std::endl;
    }


    int start(0);
    float dot(-1.0);
    for( int j=0; j<4; j++) {
      cv::Point2f d( tmp[(j+1)%4] - tmp[j] );
      if( d.dot(m_top) > dot ) {
	start = j;
	dot = d.dot(m_top);
      }
    }
    for(int j=0; j<4; j++) {
      m_vertices[i][j] = tmp[start%4];
      start++;
    }

    if(m_debug) {
      std::clog << "[getVertices] finder = " << i << ",  orientation = " << in_orientation << ",  vertices = [";
      for(int j=0; j<4; j++)
	std::clog << " " << m_vertices[i][j] << " ";
      std::clog << "]" << std::endl;
    }
  }
}


bool creekQrCodeDetector::getIntersectionPoint()
{
  cv::Point2f p( m_vertices[1][1] );
  cv::Point2f q( m_vertices[2][3] );
  cv::Point2f r(m_vertices[1][2]-m_vertices[1][1]);
  cv::Point2f s(m_vertices[2][2]-m_vertices[2][3]);

  if(cross(r, s) == 0) {return false;}
  float t = cross( q-p, s)/cross(r, s);
  m_intersection = p + t*r;

  m_center.x = (p.x + q.x) / 2.0;
  m_center.y = (p.y + q.y) / 2.0;

  return true;
}


bool creekQrCodeDetector::cropImage(cv::Mat &in_src)
{
  if( m_intersection.x < 0 || m_intersection.y < 0 || m_intersection.x > in_src.cols || m_intersection.y > in_src.rows )
    return false;
  
  std::vector<cv::Point> contour;
  try {
    contour.push_back(m_intersection);
    for(int i=0; i<3; i++) {
      int index = m_index[i].back();
      contour.insert(contour.end(), m_contours[index].begin(), m_contours[index].end());
    }
  }
  catch (const std::length_error& le) {
    std::cerr << "length error: " << le.what() << std::endl;
    std::cerr << "length = " << contour.size() << std::endl;
    for(int i=0; i<3; i++) {
      int index = m_index[i].back();
      std::cerr << " " << m_contours[index].size();
    }
    std::cerr << std::endl;
    return false;
  }
  

  float x, y, max;
  cv::Mat crop, tmp;
  tmp = in_src.clone();


  int mode=0;
  float scale=1.1;
  if( mode==0 ) {
    //cv::RotatedRect rect = cv::minAreaRect( contour );
    cv::RotatedRect rect = cv::minAreaRect( cv::Mat(contour) );
    
    cv::Mat rot = cv::getRotationMatrix2D( rect.center, rect.angle, 1.0 );
    cv::warpAffine(in_src, tmp, rot, in_src.size());

    max = std::max(rect.size.width, rect.size.height)*scale;
    x = rect.center.x - max/2.0;  if(x<0) x = 0;
    y = rect.center.y - max/2.0;  if(y<0) y = 0;
  }
  else {
    //cv::Rect rect = cv::boundingRect( contour );
    cv::Rect rect = cv::boundingRect( cv::Mat(contour) );
    
    cv::Point2f center;
    center.x = rect.x + rect.width/2.0;
    center.y = rect.y + rect.height/2.0;

    max = std::max(rect.width, rect.height)*scale;
    x = center.x - max/2.0;  if(x<0) x = 0;
    y = center.y - max/2.0;  if(y<0) y = 0;
  }


  try {
    crop = cv::Mat(tmp, cv::Rect(x, y, max, max));
  
    cv::Mat crop_resize, crop_gray;
    cv::resize(crop, crop_resize, m_out.size(), 0, 0, cv::INTER_LINEAR);
    cv::cvtColor(crop_resize, crop_gray, CV_BGR2GRAY);
    //cv::threshold(crop_gray, m_out, 128, 255, CV_THRESH_BINARY);
    cv::adaptiveThreshold(crop_gray, m_out, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, 35, 10);
  }
  catch(cv::Exception) {
    return false;
  }
  
  return true;
}


bool creekQrCodeDetector::warpImage(cv::Mat &in_src)
{
  if( m_intersection.x < 0 || m_intersection.y < 0 || m_intersection.x > in_src.cols || m_intersection.y > in_src.rows )
    return false;

  std::vector<cv::Point2f> org, dst;
  float bw=10.0;  // border width
  org.push_back(m_vertices[0][0]);
  org.push_back(m_vertices[1][1]);
  org.push_back(m_intersection);
  org.push_back(m_vertices[2][3]);

  dst.push_back(cv::Point2f(0, 0));
  dst.push_back(cv::Point2f(m_out.cols-2*bw, 0));
  dst.push_back(cv::Point2f(m_out.cols-2*bw, m_out.rows-2*bw));
  dst.push_back(cv::Point2f(0, m_out.rows-2*bw));

  cv::Mat warpMat;
  cv::Mat warp, warp_raw, warp_gray, tmp;
  tmp = in_src.clone();
  warpMat = cv::getPerspectiveTransform(org, dst);
  cv::warpPerspective(tmp, warp_raw, warpMat, cv::Size(m_out.cols-2*bw, m_out.rows-2*bw));
  cv::cvtColor(warp_raw, warp_gray, CV_BGR2GRAY);
  cv::adaptiveThreshold(warp_gray, warp, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, 35, 10);
  cv::copyMakeBorder( warp, m_out, bw, bw, bw, bw, cv::BORDER_CONSTANT, cv::Scalar(255,255,255) );
 
  return true;
}


double creekQrCodeDetector::minAreaRatio(std::vector<int> &in_contIndex)
{
  double min=1.0;

  for(int i=0; i<in_contIndex.size(); i++) {
    int index=in_contIndex[i];
    //cv::RotatedRect rect = cv::minAreaRect( m_contours[index] );
    cv::RotatedRect rect = cv::minAreaRect( cv::Mat(m_contours[index]) );
    double rect_area = rect.size.width * rect.size.height;
    //double cont_area = cv::contourArea( m_contours[index] );
    double cont_area = cv::contourArea( cv::Mat(m_contours[index]) );

    double ratio=cont_area/rect_area;
    if( min>ratio ) min=ratio;
  }
  return min;
}


float creekQrCodeDetector::distance(cv::Point2f &P, cv::Point2f &Q)
{
  return sqrt(pow(abs(P.x - Q.x),2) + pow(abs(P.y - Q.y),2)); 
}


float creekQrCodeDetector::cross(cv::Point2f v1, cv::Point2f v2)
{
  return v1.x*v2.y - v1.y*v2.x;
}


int creekQrCodeDetector::getOrientation()
{
  int orientation(QR_CENTER);

  float x = m_top.x;
  float y = m_top.y;

  if( x > 0  &&  y >=0 )
    orientation = QR_NE;
  else if( x <= 0  &&  y > 0 )
    orientation = QR_SE;
  else if( x < 0  &&  y <= 0 )
    orientation = QR_SW;
  else if( x >= 0  &&  y < 0 )
    orientation = QR_NW;

  return orientation;
}



/*
void creekQrCodeDetector::getVertices(int in_orientation)
{
  for(int i=0; i<3; i++) {
    int index = m_index[i].back();
    cv::RotatedRect rect = cv::minAreaRect( cv::Mat(m_contours[index]) );

    cv::Point2f tmp[4];
    rect.points(tmp);

    if( in_orientation == QR_NE ) {
      m_vertices[i][0] = tmp[2];
      m_vertices[i][1] = tmp[3];
      m_vertices[i][2] = tmp[0];
      m_vertices[i][3] = tmp[1];
    }
    else if( in_orientation == QR_SE ) {
      m_vertices[i][0] = tmp[3];
      m_vertices[i][1] = tmp[0];
      m_vertices[i][2] = tmp[1];
      m_vertices[i][3] = tmp[2];
    }
    else if( in_orientation == QR_SW ) {
      m_vertices[i][0] = tmp[0];
      m_vertices[i][1] = tmp[1];
      m_vertices[i][2] = tmp[2];
      m_vertices[i][3] = tmp[3];
    }
    else if( in_orientation == QR_NW ) {
      m_vertices[i][0] = tmp[1];
      m_vertices[i][1] = tmp[2];
      m_vertices[i][2] = tmp[3];
      m_vertices[i][3] = tmp[0];
    }
    else
      rect.points(m_vertices[i]);  // error


    if(m_debug) {
      std::clog << "[getVertices] finder = " << i << ",  orientation = " << in_orientation << ",  angle = " << rect.angle << ",  vertices = [";
      for(int j=0; j<4; j++)
	std::clog << " " << m_vertices[i][j] << " ";
      std::clog << "]" << std::endl;
    }
  }
}
*/

/*
int creekQrCodeDetector::getOrientation(cv::Point2f &in_slope)
{
  int orientation(QR_CENTER);

  float x = in_slope.x;
  float y = in_slope.y;

  if( x < 0 && -x > std::fabs(y) )
    orientation = QR_NE;
  else if( y < 0 && std::fabs(x) <= -y )
    orientation = QR_SE;
  else if( x > 0 && x > std::fabs(y) )
    orientation = QR_SW;
  else
    orientation = QR_NW;

  return orientation;
}
*/
