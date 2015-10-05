// -*- C++ -*-

#include "creekPointCloudViewer.h"
#include <stdexcept>

#include <cnoid/BodyLoader>
#include <cnoid/EigenUtil>

#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>  
#include <pcl/sample_consensus/model_types.h>  
#include <pcl/segmentation/sac_segmentation.h> 
#include <pcl/filters/extract_indices.h>  
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/octree/octree.h>
#include <pcl/registration/icp.h>

#include <VectorConvert.h>
#include "creekTypesEigen.h"

static const char* creekpointcloudviewer_spec[] =
  {
    "implementation_id", "creekPointCloudViewer",
    "type_name",         "creekPointCloudViewer",
    "description",       "creekPointCloudViewer",
    "version",           "1.0",
    "vendor",            "s-creek",
    "category",          "example",
    "activity_type",     "SPORADIC",
    "kind",              "creekPointCloudViewer",
    "max_instance",      "0",
    "language",          "C++",
    "lang_type",         "compile",
    ""
  };

template <class T> double toSec(T t)
{
  return t.sec + t.nsec / 1000000000.0;
}

double toDeg(double rad)
{
  return rad / M_PI * 180.0;
}


creekPointCloudViewer::creekPointCloudViewer(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    m_rangerIn("ranger", m_ranger),
    m_qCurIn("qCur", m_qCur),
    m_basePosIn("basePos", m_basePos),
    m_baseRpyIn("baseRpy", m_baseRpy),
    m_baseRpyActIn("baseRpyAct", m_baseRpyAct),
    m_basePosOut("basePosOut", m_basePosIcp),
    m_baseRpyOut("baseRpyOut", m_baseRpyIcp),
    m_axesIn("axes", m_axes),
    m_buttonsIn("buttons", m_buttons),
    m_creekPointCloudViewerServicePort("creekPointCloudViewerService"),
    m_cloud(new pcl::PointCloud<pcl::PointXYZRGB>),
    m_world(new pcl::PointCloud<pcl::PointXYZRGB>),
    //m_viewer(new pcl::visualization::CloudViewer("Cloud Viewer"))
    //m_viewer(new pcl::visualization::PCLVisualizer("Cloud Viewer"))
    m_immediately(true),
    m_rfootT(vtkSmartPointer<vtkTransform>::New()),
    m_lfootT(vtkSmartPointer<vtkTransform>::New()),
    m_samplingSize(0.01),
    m_planeThreshold(0.005),
    m_octSearchSize(0.04)
{
  m_service0.setComponent(this);
}


creekPointCloudViewer::~creekPointCloudViewer()
{
}


RTC::ReturnCode_t creekPointCloudViewer::onInitialize()
{
  std::cout << "creekPointCloudViewer : onInitialize" << std::endl;

  // setup port
  addInPort("ranger", m_rangerIn);
  addInPort("qCur", m_qCurIn);
  addInPort("basePos", m_basePosIn);
  addInPort("baseRpy", m_baseRpyIn);
  addInPort("baseRpyAct", m_baseRpyActIn);

  addInPort("axes", m_axesIn);
  addInPort("buttons", m_buttonsIn);

  addOutPort("basePosOut", m_basePosOut);
  addOutPort("baseRpyOut", m_baseRpyOut);

  m_creekPointCloudViewerServicePort.registerProvider("service0", "creekPointCloudViewerService", m_service0);
  addPort(m_creekPointCloudViewerServicePort);


  // setup model
  RTC::Properties& prop = getProperties();
  cnoid::BodyLoader bl;
  m_robot=bl.load( prop["model"].c_str() );

  std::vector<double> tmp;
  coil::stringTo(tmp, prop["initBasePos"].c_str());
  m_robot->rootLink()->p() << tmp[0], tmp[1], tmp[2];
  
  tmp.clear();
  coil::stringTo(tmp, prop["initBaseRpy"].c_str());
  m_robot->rootLink()->R() = cnoid::rotFromRpy(tmp[0], tmp[1], tmp[2]);

  m_robot->calcForwardKinematics();

  m_sensor = m_robot->findDevice<cnoid::RangeSensor>( prop["RANGE_SENSOR_NAME"] );
  m_rfoot = m_robot->link(prop["RLEG_END"]);
  m_lfoot = m_robot->link(prop["LLEG_END"]);
  coil::stringTo(m_ankleHeight, prop["ankle_height"].c_str());
  coil::stringTo(m_footSize, prop["FOOT_SIZE"].c_str());

  std::cout << "creekPointCloudViewer : robot name = " << m_robot->name() << std::endl;
  std::cout << "creekPointCloudViewer : sesor name = " << m_sensor->name() << std::endl;


  // setup pcl (cloud)
  m_cloud->reserve(5000000);
  m_world->reserve(5000000);


  // max buffer size
  double dt;
  coil::stringTo(dt, prop["pdservo.dt"].c_str());
  m_maxSeqNum = 5 / dt / m_sensor->frameRate();
  if( m_maxSeqNum < 10 )
    m_maxSeqNum = 10;


  m_qCur.data.length(m_robot->numJoints());
  m_axes.data.length(29);
  m_buttons.data.length(17);

  return RTC::RTC_OK;
}


RTC::ReturnCode_t creekPointCloudViewer::onActivated(RTC::UniqueId ec_id)
{
  std::cout << "creekPointCloudViewer : onActivated" << std::endl;


  m_active = false;


  // init data port
  m_basePos.data.x = 0.0;
  m_basePos.data.y = 0.0;
  m_basePos.data.z = 0.0;

  m_baseRpy.data.r = 0.0;
  m_baseRpy.data.p = 0.0;
  m_baseRpy.data.y = 0.0;

  m_baseRpyAct.data.r = 0.0;
  m_baseRpyAct.data.p = 0.0;
  m_baseRpyAct.data.y = 0.0;


  // log for range sensor coordinate system
  m_tcsSeq.clear();


  // setup pcl (viewer)
  if( !m_viewer ) {
    m_viewer.reset(new pcl::visualization::PCLVisualizer("Cloud Viewer")); 
    m_viewer->addPointCloud(m_cloud, "cloud");
    m_viewer->addPointCloud(m_world, "world");
    m_viewer->initCameraParameters();
    m_viewer->setSize(1280, 720);
    m_viewer->setBackgroundColor(0, 0, 0);
    m_viewer->addCoordinateSystem(0.1);
    {
      cnoid::Vector3 camera = m_robot->rootLink()->p() + m_robot->rootLink()->R() * cnoid::Vector3(-2,0,1);
      cnoid::Vector3 view = m_robot->rootLink()->p() + m_robot->rootLink()->R() * cnoid::Vector3(2,0,1);
      cnoid::Vector3 top = m_robot->rootLink()->R() * cnoid::Vector3(0,0,1);
      m_viewer->setCameraPosition(camera[0], camera[1], camera[2],
				  view[0], view[1], view[2],
				  top[0], top[1], top[2]);
    }

    // test
    if( false ) {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr floor(new pcl::PointCloud<pcl::PointXYZRGB>);
      for(int i=-50; i<=50; i++) {
	for(int j=-50; j<=50; j++) {
	  pcl::PointXYZRGB point;
	  point.x = i*0.1;
	  point.y = j*0.1;
	  point.z = 0.0;
	  point.b = 255;
	  point.g = 120;
	  floor->push_back(point);
	}
      }
      m_viewer->addPointCloud(floor, "floor");
    }

    RTC::Properties& prop = getProperties();
    m_viewer->addModelFromPLYFile(prop["RFOOT_MODEL_NAME"], m_rfootT, "rfoot");
    m_viewer->addModelFromPLYFile(prop["LFOOT_MODEL_NAME"], m_lfootT, "lfoot");

    m_cloud->clear();
    m_world->clear(); 
  }


  m_detectMode = false;
  m_autoFitting = true;
  return RTC::RTC_OK;
}


RTC::ReturnCode_t creekPointCloudViewer::onDeactivated(RTC::UniqueId ec_id)
{
  std::cout << "creekPointCloudViewer : onDeactivated" << std::endl;


  m_active = false;
  m_tcsSeq.clear();

  if( m_viewer ) {
    //m_viewer->close();  // no effect in linux
    m_viewer.reset();
  }
  return RTC::RTC_OK;
}


RTC::ReturnCode_t creekPointCloudViewer::onExecute(RTC::UniqueId ec_id)
{
  if(m_basePosIn.isNew()) m_basePosIn.read();
  if(m_baseRpyIn.isNew()) m_baseRpyIn.read();
  if(m_baseRpyActIn.isNew()) m_baseRpyActIn.read();


  if(m_qCurIn.isNew()) {
    m_qCurIn.read();

    // update model
    for(int i=0; i<m_robot->numJoints(); i++) {
      m_robot->joint(i)->q() = m_qCur.data[i];
    }
    m_robot->rootLink()->p() << m_basePos.data.x, m_basePos.data.y, m_basePos.data.z;
    //m_robot->rootLink()->R() = cnoid::rotFromRpy(m_baseRpyAct.data.r, m_baseRpyAct.data.p, m_baseRpy.data.y);
    m_robot->rootLink()->R() = cnoid::rotFromRpy(m_baseRpyAct.data.r, m_baseRpyAct.data.p, m_baseRpyAct.data.y);
    m_robot->calcForwardKinematics();

    // calc sensor p,R (on world)
    TimedCoordinateSystem tmp;
    tmp.tm = toSec(m_qCur.tm);
    tmp.p  = m_sensor->link()->p() + m_sensor->link()->R() * m_sensor->p_local();
    tmp.R  = m_sensor->link()->R() * m_sensor->R_local();

    // logging
    m_tcsSeq.push_back(tmp);
    if( m_tcsSeq.size() > m_maxSeqNum ) {
      m_tcsSeq.pop_front();
    }
  }


  // detect mode
  if( m_buttonsIn.isNew() )  m_buttonsIn.read();
  if(m_axesIn.isNew()){
    m_axesIn.read();

    if( m_detectMode ) {
      // RFOOT
      {
	cnoid::Vector3 vec(-m_axes.data[3], -m_axes.data[2], 0.0);
	cnoid::Vector3 omg(0.0, 0.0, m_buttons.data[9]-m_buttons.data[11]);
	//std::cout << "RFOOT : " << vec.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << std::endl;
	if( vec.norm() > 0.001 ) {
	  m_rsole.translation() += 0.005 * m_robot->rootLink()->R() * vec;
	}
	if( omg.norm() > 0.001 ) {
	  m_rsole.linear() = cnoid::rotFromRpy( 0.01*omg ) * m_rsole.linear();
	}
      }
      // LFOOT
      {
	cnoid::Vector3 vec(-m_axes.data[1], -m_axes.data[0], 0.0);
	cnoid::Vector3 omg(0.0, 0.0, m_buttons.data[8]-m_buttons.data[10]);
	//std::cout << "LFOOT : " << vec.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << std::endl;
	if( vec.norm() > 0.001 ) {
	  m_lsole.translation() += 0.005 * m_robot->rootLink()->R() * vec;
	}
	if( omg.norm() > 0.001 ) {
	  m_lsole.linear() = cnoid::rotFromRpy( 0.01*omg ) * m_lsole.linear();
	}
      } 
    }
  }


  if( m_detectMode )
    setModelToReference();
  else
    setModelToCurrent();


  if(m_rangerIn.isNew()) {
    m_rangerIn.read();
    //std::cout << "range : " << toSec(m_ranger.tm) << ",  q : " << toSec(m_qCur.tm) << std::endl;

    if(m_active) {
      cnoid::Vector3 sp;
      cnoid::Matrix3 sR;


      // detect coordinate system
      bool detect(false);
      double tm = toSec(m_ranger.tm);
      for( std::deque<TimedCoordinateSystem>::reverse_iterator it = m_tcsSeq.rbegin(); it != m_tcsSeq.rend(); it++) {
	if( it->tm <= tm ) {
	  sp = it->p;
	  sR = it->R;
	  m_tcsSeq.erase(m_tcsSeq.begin(), it.base());
	  detect = true;
	  break;
	}
      }
      if( !detect )
	return RTC::RTC_OK;


      // calc point (on world)
      int num = m_ranger.ranges.length();
      double *data    = m_ranger.ranges.get_buffer();
      double step     = m_ranger.config.angularRes;
      double minAngle = m_ranger.config.minAngle;
      double minRange = m_ranger.config.minRange;
      double maxRange = m_ranger.config.maxRange;
      for(int i=0; i<num; i++) {
	if( data[i] >= minRange && data[i] <= maxRange) {
	  double angle = minAngle + i*step;
	  cnoid::Vector3 p_local( data[i]*sin(-angle), 0.0, -data[i]*cos(angle));
	  cnoid::Vector3 p_world = sp + sR * p_local;

	  cnoid::Vector3 dXY(p_world-m_robot->rootLink()->p());
	  dXY(2) = 0.0;

	  if( dXY.norm() > 0.3 ) {
	    pcl::PointXYZRGB point;
	    point.x = p_world[0];
	    point.y = p_world[1];
	    point.z = p_world[2];
	    point.r = 255;
	    point.g = 255;
	    point.b = 255;
	    m_cloud->push_back(point);
	  }
	}
      }
      if( m_immediately ) {
	//m_viewer->showCloud(m_cloud->makeShared());
	m_viewer->updatePointCloud(m_cloud, "cloud");
      }
    }
  }
  m_viewer->spinOnce();


  return RTC::RTC_OK;
}


void creekPointCloudViewer::start()
{
  m_active = true;
  m_cloud->clear();

  //std::cout << m_robot->rootLink()->p().format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << std::endl;
  //std::cout << m_robot->rootLink()->R().format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", "]\n", "", "", "[", "]")) << std::endl;

  /* 
  cnoid::Vector3 camera = m_robot->rootLink()->p() + m_robot->rootLink()->R() * cnoid::Vector3(-2,0,1);
  cnoid::Vector3 view = m_robot->rootLink()->p() + m_robot->rootLink()->R() * cnoid::Vector3(2,0,1);
  cnoid::Vector3 top = m_robot->rootLink()->R() * cnoid::Vector3(0,0,1);
  //std::cout << "set camera = " << camera.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << std::endl;
  //std::cout << "set view = " << view.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << std::endl;
  
  m_viewer->setCameraPosition(camera[0], camera[1], camera[2],
			      view[0], view[1], view[2],
			      top[0], top[1], top[2]);
  */
}


void creekPointCloudViewer::stop()
{
  m_active = false;


  // add sole
  for(double x= -m_footSize[1]; x<= m_footSize[0]; x+=m_samplingSize) {
    for(double y=-m_footSize[2]; y<=m_footSize[3]; y+=m_samplingSize) {
      cnoid::Vector3 r = m_rfoot->p() + m_rfoot->R() * cnoid::Vector3( x, y, -m_ankleHeight);
      cnoid::Vector3 l = m_lfoot->p() + m_lfoot->R() * cnoid::Vector3( x,-y, -m_ankleHeight);

      pcl::PointXYZRGB rp, lp;
      rp.x = r[0];  rp.y = r[1];  rp.z = r[2];
      rp.r = 255;   rp.g = 255;   rp.b = 255;
      lp.x = l[0];  lp.y = l[1];  lp.z = l[2];
      lp.r = 255;   lp.g = 255;   lp.b = 255;
      m_cloud->push_back(rp);
      m_cloud->push_back(lp);
    }
  }


  // downsampling
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr org(new pcl::PointCloud<pcl::PointXYZRGB>());
  org = m_cloud->makeShared();  // deep copy
  pcl::VoxelGrid<pcl::PointXYZRGB> filter;
  filter.setInputCloud(org);
  filter.setLeafSize (m_samplingSize, m_samplingSize, m_samplingSize);
  filter.filter(*m_cloud);


  if( !m_world->empty() ) {
    if( matchingMap() ) {
      updateMap();
    }
    else
      std::cout << "creekPointCloudViewer : matching map error" << std::endl;
  }
  else {
    updateMap();
  }

  //m_viewer->showCloud(m_cloud->makeShared());
  m_viewer->updatePointCloud(m_cloud, "cloud");
  m_viewer->updatePointCloud(m_world, "world");
  //std::cout << "creekPointCloudViewer : cloud size = " << org->size() << " (original size)" << std::endl;
  //std::cout << "creekPointCloudViewer : cloud size = " << m_cloud->size() << " (down size)" << std::endl;
}


bool creekPointCloudViewer::detectLandingPoint(double x, double y, double w, int ft)
{
  double range(0.3), minRange(0.05);
  double bankLimit(30.0); // degree
  int minReqNum(10);


  // set detect area
  std::vector<int> indices;
  double rangeSquare = range*range;
  indices.reserve(m_world->size());
  for( int i=0; i<m_world->size(); i++) {
    double dist = (m_world->points[i].x-x)*(m_world->points[i].x-x) + (m_world->points[i].y-y)*(m_world->points[i].y-y);
    if( dist < rangeSquare ) {
      indices.push_back(i);
      m_world->points[i].r = 0;
      m_world->points[i].g = 0;
      m_world->points[i].b = 255;
    }
    else {
      m_world->points[i].r = 255;
      m_world->points[i].g = 255;
      m_world->points[i].b = 255;
    }
  }
  if( indices.size() < minReqNum ) {
    std::cout << "creekPointCloudViewer : too little of point (set area)" << std::endl;
    return false;
  }
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr area(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::copyPointCloud(*m_world, indices, *area);


  // detect plane
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  // Optional
  seg.setOptimizeCoefficients(true);
  // Mandatory
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(m_planeThreshold);
  
  seg.setInputCloud( area->makeShared() );
  seg.segment(*inliers, *coefficients);

  if( inliers->indices.size() < minReqNum ) {
    std::cout << "creekPointCloudViewer : too little of point (detect plane)" << std::endl;
    return false;
  }
  std::vector<int> plane_indices( inliers->indices.size() );
  for(int i=0; i<inliers->indices.size(); i++) {
    int id = indices[inliers->indices[i]];
    m_world->points[id].r = 255;
    m_world->points[id].g = 0;
    m_world->points[id].b = 0;
    plane_indices[i] = id;
  }


  // calc landing point
  cnoid::Vector3 ez(0,0,1);
  cnoid::Vector3 n(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
  n.normalize();
  if( n[2] < 0 )
    n = -n;
  if( n.dot(ez) < cos(bankLimit/180.0*M_PI) ) {
    double bank = acos( ez.dot(n) );
    std::cout << "creekPointCloudViewer : bank limit over ( bank = " << toDeg(bank) << ", limit = " << bankLimit << " )" << std::endl;
    //return false;
  }
  // plane equation : ax+ by+ cz + d = 0
  // coefficients values : a,b,c,d
  double z = -(coefficients->values[0]*x + coefficients->values[1]*y + coefficients->values[3]) / coefficients->values[2];
  std::cout << "creekPointCloudViewer : plane equation = " << coefficients->values[1];
  for(int i=1; i<4; i++) {
    std::cout << ", " << coefficients->values[i];
  }
  std::cout << std::endl;


  cnoid::Vector3 landP(x,y,z);
  cnoid::Matrix3 landR( Eigen::AngleAxisd(w, cnoid::Vector3::UnitZ()) );
  cnoid::Vector3 landN(landR.transpose() * n);
  cnoid::Vector3 axis( ez.cross(landN) );
  axis.normalize();
  double rp = acos( ez.dot(landN) );
  landR = Eigen::AngleAxisd(w, cnoid::Vector3::UnitZ()) * Eigen::AngleAxisd(rp, axis);

  if( m_autoFitting ) {
    if( fittingFootPosition(plane_indices, landP, landR, ft, 10) ) {
      std::cout << "creekPointCloudViewer : fitting foot position" << std::endl;
    }
    else if( false ) {  // simple mode
      Eigen::Vector4f center; 
      pcl::compute3DCentroid(*area, inliers->indices, center);
      cnoid::Vector3 dist(center[0]-x, center[1]-y, center[2]-z);
      if( dist.norm() > minRange ) {
	dist *= ( minRange/dist.norm() );
      }
      landP += dist;
      std::cout << "creekPointCloudViewer : move landing point,  move = " 
		<< dist.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << std::endl;
    }
  }


  // calc model position and orientation
  cnoid::Vector3 pos(landP);
  pos += (m_ankleHeight * n);
    
  Eigen::AngleAxisd rot( landR );

  // set model
  if( ft == 0 ) {
    m_rfootT->Identity();  // reset
    m_rfootT->Translate(pos[0], pos[1], pos[2]);
    m_rfootT->RotateWXYZ( toDeg(rot.angle()), rot.axis()[0], rot.axis()[1], rot.axis()[2]);

    m_rsole.translation() = landP;
    m_rsole.linear() = landR;

    std::cout << "creekPointCloudViewer : rfoot landing point" << std::endl;
    std::cout << "  pos = " << landP.format( Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]") ) << std::endl;
    std::cout << "  rot =\n" << landR.format( Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", "]\n", "", "", "[", "]") ) << std::endl;
  }
  else if(ft == 1) {
    m_lfootT->Identity();  // reset
    m_lfootT->Translate(pos[0], pos[1], pos[2]);
    m_lfootT->RotateWXYZ( toDeg(rot.angle()), rot.axis()[0], rot.axis()[1], rot.axis()[2]);

    m_lsole.translation() = landP;
    m_lsole.linear() = landR;

    std::cout << "creekPointCloudViewer : lfoot landing point" << std::endl;
    std::cout << "  pos = " << landP.format( Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]") );
    std::cout << "  rot = " << landR.format( Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", "]\n", "", "", "[", "]") );
  }
  else {
    std::cout << "creekPointCloudViewer : foot type error" << std::endl;
    return false;
  }
  m_viewer->updatePointCloud(m_world, "world");
  return true;
}


void creekPointCloudViewer::getLandingPoint(double &x, double &y, double &z, double &r, double &p, double &w, int ft)
{
  if( ft == 0 ) {
    cnoid::Vector3 euler( m_rsole.linear().eulerAngles(2,1,0) );
    x = m_rsole.translation()[0];
    y = m_rsole.translation()[1];
    z = m_rsole.translation()[2];

    r = euler[2];
    p = euler[1];
    w = euler[0];
  }
  else if( ft == 1 ) {
    cnoid::Vector3 euler( m_lsole.linear().eulerAngles(2,1,0) );
    x = m_lsole.translation()[0];
    y = m_lsole.translation()[1];
    z = m_lsole.translation()[2];

    r = euler[2];
    p = euler[1];
    w = euler[0];
  }
  else {
    std::cout << "creekPointCloudViewer : foot type error" << std::endl;
  }
}


bool creekPointCloudViewer::fittingFootPosition(std::vector<int> &indices, cnoid::Vector3 &posd, cnoid::Matrix3 &rotd, int ft, int maxTrialNum, double margin)
{
  pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB> octree(m_octSearchSize);
  octree.setInputCloud(m_world, pcl::IndicesConstPtr(new std::vector<int>(indices)));
  octree.addPointsFromInputCloud();


  // cast
  Eigen::Vector3f pos(posd.cast<float>());
  Eigen::Matrix3f rot(rotd.cast<float>());


  Eigen::Vector3f c2rt, c2lt, c2rh, c2lh;  // center to RorL, TOEorHEEL
  if( ft == 0 ) {
    c2rt <<  m_footSize[0]+margin, -m_footSize[2]-margin, 0;
    c2lt <<  m_footSize[0]+margin,  m_footSize[3]+margin, 0;
    c2rh << -m_footSize[1]-margin, -m_footSize[2]-margin, 0;
    c2lh << -m_footSize[1]-margin,  m_footSize[3]+margin, 0;
  }
  else if( ft == 1 ) {
    c2rt <<  m_footSize[0]+margin, -m_footSize[3]-margin, 0;
    c2lt <<  m_footSize[0]+margin,  m_footSize[2]+margin, 0;
    c2rh << -m_footSize[1]-margin, -m_footSize[3]-margin, 0;
    c2lh << -m_footSize[1]-margin,  m_footSize[2]+margin, 0;
  }
  else {
    std::cout << "creekPointCloudViewer : foot type error" << std::endl;
    return false;
  }


  std::vector<int> k_indices;
  Eigen::Vector3f center, rtoe, ltoe, rheel, lheel;
  Eigen::Vector3f over(0,0,0.1), dir( rot*Eigen::Vector3f(0,0,-1) );
  Eigen::Vector3f direction(0,0,0), adjust(0,0,0);
  float scale(0.01);


  bool ret(false);
  std::bitset<5> bitflag;
  for(int i=0; i<maxTrialNum; i++) {
    
    bitflag.set();
    direction = Eigen::Vector3f::Zero();


    center = pos + rot * ( adjust + over);
    rtoe  = center + rot * c2rt;
    ltoe  = center + rot * c2lt;
    rheel = center + rot * c2rh;
    lheel = center + rot * c2lh;


    if( octree.getIntersectedVoxelIndices(center, dir, k_indices, 1) == 0 ) {
      bitflag.reset(0);
    }
    if( octree.getIntersectedVoxelIndices(rtoe, dir, k_indices, 1) == 0 ) {
      direction[0] -= 1;
      direction[1] += 1;
      bitflag.reset(1);
    }
    if( octree.getIntersectedVoxelIndices(ltoe, dir, k_indices, 1) == 0 ) {
      direction[0] -= 1;
      direction[1] -= 1;
      bitflag.reset(2);
    }
    if( octree.getIntersectedVoxelIndices(rheel, dir, k_indices, 1) == 0 ) {
      direction[0] += 1;
      direction[1] += 1;
      bitflag.reset(3);
    }
    if( octree.getIntersectedVoxelIndices(lheel, dir, k_indices, 1) == 0 ) {
      direction[0] += 1;
      direction[1] -= 1;
      bitflag.reset(4);
    }


    if( bitflag.count() == 5 ) {
      ret = true;
      break;
    }
    else if( !bitflag[0] && bitflag.count() == 4 ) {
      std::cout << "creekPointCloudViewer : on the hole ???" << std::endl;
      ret = true;
      break;
    }
    else if( bitflag.none() ) {
      std::cout << "creekPointCloudViewer : out of detect area, trial num = " << i << std::endl;
      std::cout << "  pos = " << center.format(creek::IOvec()) << std::endl;
      std::cout << "  dir = " << dir.format(creek::IOvec()) << std::endl;
      ret = false;
      break;
    }
    else {
      if( direction.squaredNorm() < 1.0e-6 ) {
	std::cout << "creekPointCloudViewer : exception !!!  trial num = " << i << ",  flag = " << bitflag << std::endl;
	return false;
      }
      else {
	direction.normalize();
	std::cout << "creekPointCloudViewer : fitting trial " << i << ",  direction = " << direction.format(creek::IOvec()) << std::endl;
	adjust += scale*direction;
      }
    }
  }


  if( ret ) {
    posd = ( pos + rot * adjust ).cast<double>();
  }
  else {
    std::cout << "creekPointCloudViewer : fitting false, last flag = " << bitflag << std::endl;
  }
  return ret;
}


bool creekPointCloudViewer::matchingMap()
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr org(new pcl::PointCloud<pcl::PointXYZRGB>);
  org = m_cloud->makeShared();  // deep copy

  pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
  icp.setInputSource(org);
  icp.setInputTarget(m_world);

  //icp.setMaxCorrespondenceDistance(0.05);
  icp.setMaxCorrespondenceDistance(0.5);
  icp.setMaximumIterations(200);
  icp.setTransformationEpsilon(1e-8);
  //icp.setEuclideanFitnessEpsilon (0.005);
  icp.setEuclideanFitnessEpsilon (0.001);

  icp.align(*m_cloud);

  if( icp.hasConverged() ) {
    std::cout << "creekPointCloudViewer : transform" << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl << std::endl;
    
    Eigen::Matrix4f tf = icp.getFinalTransformation();
    cnoid::Matrix3 rot;
    cnoid::Vector3 vec, rpy;
    for(int i=0; i<3; i++) {
      for(int j=0; j<3; j++) {
	rot(i,j) = tf(i,j);
      }
      vec(i) = tf(i,3);
    }

    m_robot->rootLink()->p() = rot * m_robot->rootLink()->p() + vec;
    m_robot->rootLink()->R() = rot * m_robot->rootLink()->R();
    rpy =  m_robot->rootLink()->R().eulerAngles(2,1,0);
    
    m_basePosIcp.data.x = m_robot->rootLink()->p()(0);
    m_basePosIcp.data.y = m_robot->rootLink()->p()(1);
    m_basePosIcp.data.z = m_robot->rootLink()->p()(2);

    m_baseRpyIcp.data.r = rpy(2);
    m_baseRpyIcp.data.p = rpy(1);
    m_baseRpyIcp.data.y = rpy(0);
    
    m_basePosOut.write();
    m_baseRpyOut.write();

    std::cout << m_robot->rootLink()->p().format( Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]") ) << std::endl;
    std::cout << rpy.format( Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]") ) << std::endl;
  }
  return icp.hasConverged();
}


void creekPointCloudViewer::updateMap()
{
  try {
    m_world->insert(m_world->end(), m_cloud->begin(), m_cloud->end());
    m_cloud->clear();


    pcl::PointCloud<pcl::PointXYZRGB>::Ptr org(new pcl::PointCloud<pcl::PointXYZRGB>());
    org = m_world->makeShared();  // deep copy
    pcl::VoxelGrid<pcl::PointXYZRGB> filter;
    filter.setInputCloud(org);
    filter.setLeafSize (m_samplingSize, m_samplingSize, m_samplingSize);
    filter.filter(*m_world);

    m_world->width  = m_world->size();
    m_world->height = 1;

    m_viewer->updatePointCloud(m_cloud, "cloud");
    m_viewer->updatePointCloud(m_world, "world");
  }
  catch( const std::length_error& le ) {
    std::cerr << "updateMap : length error: " << le.what() << std::endl;
  }
  std::cout << "robot pos = " << m_robot->rootLink()->p().format( Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]") ) << std::endl;
}


void creekPointCloudViewer::test()
{
  std::cout << "creekPointCloudViewer : test" << std::endl;

  if(true) {
    std::cout  << "creekPointCloudViewer" << std::endl;
    std::cout << "  time = " << toSec(m_ranger.tm) << std::endl;
    std::cout << "  size = " << m_ranger.ranges.length() << std::endl;
    std::cout << "  config" << std::endl;
    std::cout << "      minAngle    = " << toDeg(m_ranger.config.minAngle) << std::endl;
    std::cout << "      maxAngle    = " << toDeg(m_ranger.config.maxAngle) << std::endl;
    std::cout << "      angularRes  = " << toDeg(m_ranger.config.angularRes) << std::endl;
    std::cout << "      minRange    = " << m_ranger.config.minRange << std::endl;
    std::cout << "      maxRange    = " << m_ranger.config.maxRange << std::endl;
    std::cout << "      rangeRes    = " << m_ranger.config.rangeRes << std::endl;
    std::cout << "      frequency   = " << m_ranger.config.frequency << std::endl;
    std::cout << "  geometry" << std::endl;
    std::cout << "      [ x, y, z ] = [ " << m_ranger.geometry.geometry.pose.position.x << ", " << m_ranger.geometry.geometry.pose.position.y << ", " << m_ranger.geometry.geometry.pose.position.z << " ]" << std::endl;
    std::cout << "      [ r, p, y ] = [ " << m_ranger.geometry.geometry.pose.orientation.r << ", " << m_ranger.geometry.geometry.pose.orientation.p << ", " << m_ranger.geometry.geometry.pose.orientation.y << " ]" << std::endl;
    std::cout << "      [ w, l, h ] = [ " << m_ranger.geometry.geometry.size.w << ", " << m_ranger.geometry.geometry.size.l << ", " << m_ranger.geometry.geometry.size.h << " ]" << std::endl;
  }


  if( true ) {
    //pcl::io::savePCDFile("/home/player/tsml/log/JVRC_R1M.pcd", m_cloud);
  }
}


void creekPointCloudViewer::setModelToCurrent()
{
  { // RFOOT
    cnoid::Vector3 pos = m_rfoot->p();
    Eigen::AngleAxisd rot( m_rfoot->R() );
    m_rfootT->Identity();  // reset
    m_rfootT->Translate(pos[0], pos[1], pos[2]);
    m_rfootT->RotateWXYZ( toDeg(rot.angle()), rot.axis()[0], rot.axis()[1], rot.axis()[2]);

    m_rsole.translation() = m_rfoot->p() + m_rfoot->R() * cnoid::Vector3(0,0,-m_ankleHeight);
    m_rsole.linear() = m_rfoot->R();
  }
  { // LFOOT
    cnoid::Vector3 pos = m_lfoot->p();
    Eigen::AngleAxisd rot( m_lfoot->R() );
    m_lfootT->Identity();  // reset
    m_lfootT->Translate(pos[0], pos[1], pos[2]);
    m_lfootT->RotateWXYZ( toDeg(rot.angle()), rot.axis()[0], rot.axis()[1], rot.axis()[2]);

    m_lsole.translation() = m_lfoot->p() + m_lfoot->R() * cnoid::Vector3(0,0,-m_ankleHeight);
    m_lsole.linear() = m_lfoot->R();
  }
}


void creekPointCloudViewer::setModelToReference()
{
  { // RFOOT
    cnoid::Vector3 pos = m_rsole.translation() +  m_rsole.linear() * cnoid::Vector3(0,0,m_ankleHeight);
    Eigen::AngleAxisd rot( m_rsole.linear() );

    m_rfootT->Identity();  // reset
    m_rfootT->Translate(pos[0], pos[1], pos[2]);
    m_rfootT->RotateWXYZ( toDeg(rot.angle()), rot.axis()[0], rot.axis()[1], rot.axis()[2]);
  }
  { // LFOOT
    cnoid::Vector3 pos = m_lsole.translation() +  m_lsole.linear() * cnoid::Vector3(0,0,m_ankleHeight);
    Eigen::AngleAxisd rot( m_lsole.linear() );

    m_lfootT->Identity();  // reset
    m_lfootT->Translate(pos[0], pos[1], pos[2]);
    m_lfootT->RotateWXYZ( toDeg(rot.angle()), rot.axis()[0], rot.axis()[1], rot.axis()[2]);
  }
}


void creekPointCloudViewer::changeMode()
{
  if( !m_detectMode ) {
    m_detectMode = true;
    std::cout << "creekPointCloudViewer : detect mode" << std::endl;
  }
  else {
    m_detectMode = false;
    std::cout << "creekPointCloudViewer : view mode" << std::endl;
  }
}


bool creekPointCloudViewer::autoFittinSwitch()
{
  m_autoFitting = !m_autoFitting;

  if( m_autoFitting )
    std::cout << "creekPointCloudViewer : auto fitting ON" << std::endl;
  else
    std::cout << "creekPointCloudViewer : auto fitting OFF" << std::endl;

  return m_autoFitting;
}


void creekPointCloudViewer::clear()
{
  m_world->clear();
  m_world->width  = 0;
  m_world->height = 0;
}

extern "C"
{
 
  void creekPointCloudViewerInit(RTC::Manager* manager)
  {
    coil::Properties profile(creekpointcloudviewer_spec);
    manager->registerFactory(profile,
                             RTC::Create<creekPointCloudViewer>,
                             RTC::Delete<creekPointCloudViewer>);
  }
  
};




