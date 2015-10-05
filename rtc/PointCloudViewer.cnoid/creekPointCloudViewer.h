// -*- C++ -*-

#ifndef CREEKPOINTCLOUDVIEWER_H
#define CREEKPOINTCLOUDVIEWER_H

#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypes.hh>
#include <rtm/idl/InterfaceDataTypes.hh>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>

// Service implementation headers
#include "creekPointCloudViewerService_impl.h"

#include <cnoid/Body>
#include <cnoid/Link>
#include <cnoid/RangeSensor>

#include <pcl/point_types.h>
//#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <vtkSmartPointer.h>
#include <vtkTransform.h>

using namespace RTC;

class creekPointCloudViewer  : public RTC::DataFlowComponentBase
{
public:
  creekPointCloudViewer(RTC::Manager* manager);
  ~creekPointCloudViewer();

  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

  void start();
  void stop();
  bool detectLandingPoint(double x, double y, double w, int ft);
  void getLandingPoint(double &x, double &y, double &z, double &r, double &p, double &w, int ft);
  bool matchingMap();
  void updateMap();
  void test();

  void changeMode();
  void setModelToCurrent();
  void setModelToReference();

  bool autoFittinSwitch();
  void clear();


protected:
  RTC::RangeData m_ranger;
  RTC::InPort<RTC::RangeData> m_rangerIn;

  RTC::TimedDoubleSeq               m_qCur;
  RTC::InPort<RTC::TimedDoubleSeq>  m_qCurIn;
  
  RTC::TimedPoint3D               m_basePos;
  RTC::InPort<RTC::TimedPoint3D>  m_basePosIn;
  RTC::TimedOrientation3D               m_baseRpy;
  RTC::InPort<RTC::TimedOrientation3D>  m_baseRpyIn;
  RTC::TimedOrientation3D               m_baseRpyAct;
  RTC::InPort<RTC::TimedOrientation3D>  m_baseRpyActIn;

  RTC::TimedPoint3D                m_basePosIcp;
  RTC::OutPort<RTC::TimedPoint3D>  m_basePosOut;
  RTC::TimedOrientation3D                m_baseRpyIcp;
  RTC::OutPort<RTC::TimedOrientation3D>  m_baseRpyOut;

  RTC::TimedFloatSeq m_axes;
  RTC::InPort<RTC::TimedFloatSeq> m_axesIn;
  RTC::TimedBooleanSeq m_buttons;
  RTC::InPort<RTC::TimedBooleanSeq> m_buttonsIn;
  
  RTC::CorbaPort m_creekPointCloudViewerServicePort;
  creekPointCloudViewerService_impl m_service0;


private:
  bool fittingFootPosition(std::vector<int> &indices, cnoid::Vector3 &pos, cnoid::Matrix3 &rot, int ft, int maxTrialNum=1, double margin=0.01);

  cnoid::BodyPtr m_robot;
  cnoid::RangeSensorPtr m_sensor;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_cloud, m_world, m_flag;
  //boost::shared_ptr<pcl::visualization::CloudViewer> m_viewer;
  pcl::visualization::PCLVisualizer::Ptr m_viewer;

  bool m_active, m_immediately;

  int m_maxSeqNum;
  struct TimedCoordinateSystem{
    double tm;
    cnoid::Vector3 p;
    cnoid::Matrix3 R;
  };
  std::deque<TimedCoordinateSystem> m_tcsSeq;

  double m_ankleHeight;
  std::vector<double> m_footSize;  // toe, heel, outer, inner
  cnoid::Link * m_rfoot, *m_lfoot;
  vtkSmartPointer<vtkTransform> m_rfootT, m_lfootT;
  cnoid::Position m_rsole, m_lsole;

  double m_samplingSize, m_planeThreshold, m_octSearchSize;

  bool m_detectMode, m_autoFitting;
};


extern "C"
{
  DLL_EXPORT void creekPointCloudViewerInit(RTC::Manager* manager);
};

#endif // CREEKPOINTCLOUDVIEWER_H

