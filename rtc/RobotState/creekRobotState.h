// -*- C++ -*-

#ifndef CREEKROBOTSTATE_H
#define CREEKROBOTSTATE_H

#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>

#include "creekRobotStateService_impl.h"

#include <cnoid/Body>
#include <cnoid/Camera>

using namespace RTC;

class creekRobotState  : public RTC::DataFlowComponentBase
{
 public:
  creekRobotState(RTC::Manager* manager);
  ~creekRobotState();

 virtual RTC::ReturnCode_t onInitialize();
   virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
   virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);
   virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

 protected:
  TimedDoubleSeq m_q;
  InPort<TimedDoubleSeq>  m_qIn;
  OutPort<TimedDoubleSeq> m_qOut;

  TimedDoubleSeq m_dq;
  OutPort<TimedDoubleSeq> m_dqOut;
  TimedDoubleSeq m_ddq;
  OutPort<TimedDoubleSeq> m_ddqOut;

  TimedPoint3D         m_basePos;
  InPort<TimedPoint3D> m_basePosIn;
  TimedOrientation3D         m_baseRpy;
  InPort<TimedOrientation3D> m_baseRpyIn;

  TimedPose3D          m_basePose;
  OutPort<TimedPose3D> m_basePoseOut;

  std::vector<TimedPose3D>              m_cameraPose;
  std::vector< OutPort<TimedPose3D> * > m_cameraPoseOut;

  RTC::CorbaPort m_creekRobotStateServicePort;
  creekRobotStateService_impl m_service0;


 private:
  cnoid::BodyPtr m_robot;
  cnoid::DeviceList<cnoid::Camera> m_cameras;
};


extern "C"
{
  DLL_EXPORT void creekRobotStateInit(RTC::Manager* manager);
};

#endif // CREEKROBOTSTATE_H

