// -*- C++ -*-

#ifndef CREEKCAMERAVIEWER_H
#define CREEKCAMERAVIEWER_H

#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>

// Service implementation headers
#include "creekCameraViewerService_impl.h"

#include "CameraPort.h"
#include <cnoid/EigenUtil>

using namespace RTC;

class creekCameraViewer  : public RTC::DataFlowComponentBase
{
public:
  creekCameraViewer(RTC::Manager* manager);
  ~creekCameraViewer();

  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

  void setSearchFlag(std::string &list);
  void show();
  void saveData(const char *path);


protected:
  std::vector< CameraPort * > m_ports;

  std::vector<TimedPose3D>             m_cameraPose;
  std::vector< InPort<TimedPose3D> * > m_cameraPoseIn;

  RTC::CorbaPort m_creekCameraViewerServicePort;
  creekCameraViewerService_impl m_service0;

private:
  cnoid::Vector3 pixelToVector(double x, double y);
  bool intersectLines(const cnoid::Vector3 &p1, const cnoid::Vector3 &e1, const cnoid::Vector3 &p2, const cnoid::Vector3 &e2, cnoid::Vector3 &out);
  bool getCameraPose(double tm, int index, cnoid::Vector3 &p, cnoid::Matrix3 &R);

  void drawFrame( cv::Mat &in_src, std::vector<cv::Point2f> &points, cv::Point2f &center );
  void combineImage();
  cv::Mat m_allImage;

  zbar::ImageScanner m_scanner;

  std::map<std::string, int> m_nameToIndex;
  std::vector<bool> m_searchFlag;

  int m_maxSeqNum;
  struct TimedCoordinateSystem {
    double tm;
    cnoid::Vector3 p;
    cnoid::Matrix3 R;
  };
  std::vector< std::deque<TimedCoordinateSystem> > m_tcsSeq;

  // struct QrCodeConf {
  //   cnoid::Vector3 p, e;
  //   std::string data;
  // };
  // std::vector< std::map<std::string, QrCodeConf> > m_qrConfSet;

  struct QrCodeData {
    cnoid::Vector3 pos;
    std::string data;
    bool calc;
    cnoid::Vector3 p1, e1;
    cnoid::Vector3 p2, e2;
    std::string name1, name2;
  };
  std::map< std::string, QrCodeData > m_qrDataSet;
};


extern "C"
{
  DLL_EXPORT void creekCameraViewerInit(RTC::Manager* manager);
};

#endif // CREEKCAMERAVIEWER_H

