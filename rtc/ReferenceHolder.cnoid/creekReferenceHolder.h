// -*- C++ -*-

#ifndef CREEKREFERENCEHOLDER_H
#define CREEKREFERENCEHOLDER_H

#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>

#include <cnoid/Body>
#include "VectorConvert.h"

using namespace RTC;

class creekReferenceHolder  : public RTC::DataFlowComponentBase
{
public:
  creekReferenceHolder(RTC::Manager* manager);
  ~creekReferenceHolder();

  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);


protected:
  
  TimedDoubleSeq          m_qCur;
  InPort<TimedDoubleSeq>  m_qCurIn;

  // 
  // reference data
  //
  TimedDoubleSeq          m_q;
  InPort<TimedDoubleSeq>  m_qIn;
  OutPort<TimedDoubleSeq> m_qOut;

  TimedPoint3D          m_basePos;
  InPort<TimedPoint3D>  m_basePosIn;
  OutPort<TimedPoint3D> m_basePosOut;

  TimedOrientation3D          m_baseRpy;
  InPort<TimedOrientation3D>  m_baseRpyIn;
  OutPort<TimedOrientation3D> m_baseRpyOut;

  TimedPoint3D          m_zmpRef;
  InPort<TimedPoint3D>  m_zmpRefIn;
  OutPort<TimedPoint3D> m_zmpRefOut;


private:
  cnoid::BodyPtr m_robot;
  bool m_goAct;
};


extern "C"
{
  DLL_EXPORT void creekReferenceHolderInit(RTC::Manager* manager);
};

#endif // CREEKREFERENCEHOLDER_H

