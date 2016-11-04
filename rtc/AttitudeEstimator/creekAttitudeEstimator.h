// -*- c++ -*-

#ifndef CREEK_ATTITUDE_ESTIMATOR_H
#define CREEK_ATTITUDE_ESTIMATOR_H

// rtm
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>

#include "KalmanFilter.h"

class creekAttitudeEstimator : public RTC::DataFlowComponentBase
{
public:
  creekAttitudeEstimator(RTC::Manager * manager);
  //~creekAttitudeEstimator();

  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);


protected:
  // inport sensor
  RTC::TimedDoubleSeq m_accSensor;
  RTC::InPort<RTC::TimedDoubleSeq> m_accSensorIn;
  RTC::TimedDoubleSeq m_gyroSensor;
  RTC::InPort<RTC::TimedDoubleSeq> m_gyroSensorIn;

  RTC::TimedDoubleSeq m_accRef;
  RTC::InPort<RTC::TimedDoubleSeq> m_accRefIn;


  // outport
  //RTC::TimedDoubleSeq m_rpy;
  //RTC::OutPort<RTC::TimedDoubleSeq> m_rpyOut;
  RTC::TimedOrientation3D m_rpy;
  RTC::OutPort<RTC::TimedOrientation3D> m_rpyOut;


private:
  double m_dt;
  creek::KalmanFilter m_kf[3];
};

extern "C"
{
  void creekAttitudeEstimatorInit(RTC::Manager * manager);
};

#endif
