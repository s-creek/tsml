// -*- C++ -*-

#ifndef CREEKSEQUENCEPLAYER_H
#define CREEKSEQUENCEPLAYER_H

#include <boost/interprocess/sync/interprocess_semaphore.hpp>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>

#include "creekSequencePlayerService_impl.h"

#include <cnoid/Body>
#include <cnoid/JointPath>
#include "VectorConvert.h"

#include <creekInterpolator/Interpolator.h>

using namespace RTC;

class creekSequencePlayer  : public RTC::DataFlowComponentBase
{
public:
  creekSequencePlayer(RTC::Manager* manager);
  ~creekSequencePlayer();

  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);


  inline unsigned int dof() const { return m_dof; };
  void waitInterpolation();
  bool setJointAngles(const double *angles, double tm);
  bool setJointAngle(const char* jname, double jv, double tm);
  bool setBasePos(const double *pos, double tm);
  bool setBaseRpy(const double *rpy, double tm);
  bool setZmp(const double *zmp, double tm);
  bool isEmpty();

  bool setBasePosRel(const double *pos, double tm);

  void jointCalib(int scale);


protected:
  //
  // InPort
  //
  TimedDoubleSeq m_qInit;
  InPort<TimedDoubleSeq> m_qInitIn;
  TimedPoint3D m_basePosInit;
  InPort<TimedPoint3D> m_basePosInitIn;
  TimedOrientation3D m_baseRpyInit;
  InPort<TimedOrientation3D> m_baseRpyInitIn;
  TimedPoint3D m_zmpRefInit;
  InPort<TimedPoint3D> m_zmpRefInitIn;


  //
  // OutPort
  //
  TimedDoubleSeq m_qRef;
  OutPort<TimedDoubleSeq> m_qRefOut;
  TimedPoint3D m_basePos;
  OutPort<TimedPoint3D> m_basePosOut;
  TimedOrientation3D m_baseRpy;
  OutPort<TimedOrientation3D> m_baseRpyOut;
  TimedPoint3D m_zmpRef;
  OutPort<TimedPoint3D> m_zmpRefOut;


  //
  // ServicePort
  //
  RTC::CorbaPort m_creekSequencePlayerServicePort;
  creekSequencePlayerService_impl m_service0;


private:
  double m_dt;
  unsigned int m_dof;
  cnoid::BodyPtr m_robot;

  enum {ANGLES, POS, RPY, ZMP, NUM_SEQ};
  creek::Interpolator *m_seq[NUM_SEQ];

  bool m_waitFlag;
  boost::interprocess::interprocess_semaphore m_waitSem;

  cnoid::JointPathPtr m_rleg, m_lleg;

  void calcCoM();
};


extern "C"
{
  DLL_EXPORT void creekSequencePlayerInit(RTC::Manager* manager);
};

#endif // CREEKSEQUENCEPLAYER_H

