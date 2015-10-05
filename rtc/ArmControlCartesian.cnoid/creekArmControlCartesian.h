// -*- C++ -*-

#ifndef CREEKARMCONTROLCARTESIAN_H
#define CREEKARMCONTROLCARTESIAN_H

#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypes.hh>
#include <rtm/idl/InterfaceDataTypes.hh>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>

#include "creekArmControlCartesianService_impl.h"

#include <cnoid/Body>
#include <cnoid/JointPath>

using namespace RTC;

class creekArmControlCartesian  : public RTC::DataFlowComponentBase
{
public:
  creekArmControlCartesian(RTC::Manager* manager);
  ~creekArmControlCartesian();

  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

  void setArm(int armId);


protected:
  //
  // InPort
  //
  TimedDoubleSeq m_qCur;
  InPort<TimedDoubleSeq> m_qCurIn;
  TimedPoint3D m_basePos;
  InPort<TimedPoint3D> m_basePosIn;
  TimedOrientation3D m_baseRpy;
  InPort<TimedOrientation3D> m_baseRpyIn;

  RTC::TimedFloatSeq m_axes;
  RTC::InPort<RTC::TimedFloatSeq> m_axesIn;
  RTC::TimedBooleanSeq m_buttons;
  RTC::InPort<RTC::TimedBooleanSeq> m_buttonsIn;

  //
  // OutPort
  //
  TimedDoubleSeq m_qRef;
  OutPort<TimedDoubleSeq> m_qRefOut;

  RTC::CorbaPort m_creekArmControlCartesianServicePort;
  creekArmControlCartesianService_impl m_service0;

private:
  void initModel();
  bool calcInverseKinematics(cnoid::JointPathPtr arm, cnoid::Vector3 &refP, cnoid::Matrix3 &refR);

  int m_activeArmId;
  bool m_active;

  cnoid::BodyPtr m_robot;
  cnoid::JointPathPtr m_rarm, m_larm;
  cnoid::Position m_rarmRef, m_larmRef;
};


extern "C"
{
  DLL_EXPORT void creekArmControlCartesianInit(RTC::Manager* manager);
};

#endif // CREEKARMCONTROLCARTESIAN_H

