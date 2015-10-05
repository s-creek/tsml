// -*- c++ -*-

#ifndef CREEK_PDSERVO_H
#define CREEK_PDSERVO_H

#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>


class creekPdServo : public RTC::DataFlowComponentBase
{
public:
  creekPdServo(RTC::Manager * manager);
  //~creekPdServo();

  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);


protected:
  RTC::TimedDoubleSeq m_qRef;
  RTC::InPort<RTC::TimedDoubleSeq> m_qRefIn;

  RTC::TimedDoubleSeq m_qCur;
  RTC::InPort<RTC::TimedDoubleSeq> m_qCurIn;

  RTC::TimedDoubleSeq m_tauRef;
  RTC::OutPort<RTC::TimedDoubleSeq> m_tauRefOut;


private:
  double m_dt;
  std::string m_instanceName;

  int m_dof;
  std::vector<double> m_pGain;
  std::vector<double> m_dGain;

  bool m_isInit;
  std::vector<double> m_qPre;
  std::vector<double> m_qRefPre;

  unsigned int m_calcGainMode;
  std::vector<double> m_maxErr;
  std::vector<int> m_count;
};

extern "C"
{
  void creekPdServoInit(RTC::Manager * manager);
};

#endif
