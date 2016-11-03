#include "creekPdServo.h"
#include "VectorConvert.h"

#include <rtm/CorbaNaming.h>
#include <cmath>

// Module specification
static const char* module_spec[] =
  {
    "implementation_id", "creekPdServo",
    "type_name",         "creekPdServo",
    "description",       "Sequence InPort component",
    "version",           "1.0",
    "vendor",            "s-creek",
    "category",          "example",
    "activity_type",     "SPORADIC",
    "kind",              "SequenceInComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    "conf.default.calcGainMode",  "0",
    // Configuration variables
    ""
  };


creekPdServo::creekPdServo(RTC::Manager * manager)
  : RTC::DataFlowComponentBase(manager),
    m_qRefIn("qRef", m_qRef),
    m_qActIn("q", m_qAct),
    m_tauRefOut("tau", m_tauRef)
{
}


RTC::ReturnCode_t creekPdServo::onInitialize()
{
  std::cout << "creekPdServo : onInitialize" << std::endl;


  //
  // Registration: InPort/OutPort/Service
  //
  // Set InPort buffers
  addInPort("qRef", m_qRefIn);
  addInPort("q", m_qActIn);
  
  // Set OutPort buffer
  addOutPort("tau", m_tauRefOut);


  //
  // get properties
  //
  RTC::Properties& prop = getProperties();
  coil::stringTo(m_dt, prop["pdservo.dt"].c_str());
  m_instanceName = prop["instance_name"];
  std::cout << m_instanceName << " : dt = " << m_dt << std::endl;


  //
  // set PD gain
  //
  coil::stringTo(m_pGain, prop["pdservo.pgain"].c_str());
  coil::stringTo(m_dGain, prop["pdservo.dgain"].c_str());

  m_dof = m_pGain.size();
  std::cout << m_instanceName << " : dof = " << m_dof << std::endl;


  if( m_pGain.size() != m_dGain.size() ) {
    std::cerr << m_instanceName << " : failed to load gain" << std::endl;
    return RTC::RTC_ERROR;
  }
  else {
    std::cout << m_instanceName << " : p gain =";  for( int i = 0; i < m_dof; i++)  std::cout << "  " << m_pGain[i];  std::cout << std::endl;
    std::cout << m_instanceName << " : d gain =";  for( int i = 0; i < m_dof; i++)  std::cout << "  " << m_dGain[i];  std::cout << std::endl;
  }

  m_qActPre.resize(m_dof);
  m_qRefPre.resize(m_dof);

  m_qAct.data.length(m_dof);
  m_qRef.data.length(m_dof);
  m_tauRef.data.length(m_dof);

  return RTC::RTC_OK;
}


RTC::ReturnCode_t creekPdServo::onActivated(RTC::UniqueId ec_id)
{
  std::cout << m_instanceName << " : onActivated" << std::endl;

  //
  // set initial data
  //
  if( m_qActIn.isNew() ) {
    m_qActIn.read();

    for(int i = 0; i < m_dof; i++) {
      m_qActPre[i] = m_qRefPre[i] = m_qRef.data[i] = m_qAct.data[i];
    }
  }
  else {
    std::cout << m_instanceName << " : connection error" << std::endl;
    return RTC::RTC_ERROR;
  }
  return RTC::RTC_OK;
}


RTC::ReturnCode_t creekPdServo::onDeactivated(RTC::UniqueId ec_id)
{
  std::cout << m_instanceName << " : onDeactivated" << std::endl;
  return RTC::RTC_OK;
}


RTC::ReturnCode_t creekPdServo::onExecute(RTC::UniqueId ec_id)
{
  if( m_qActIn.isNew() )  m_qActIn.read();
  if( m_qRefIn.isNew() )  m_qRefIn.read();


  for( int i = 0; i < m_dof; i++) {
    double qAct = m_qAct.data[i];
    double qRef = m_qRef.data[i];
    double vAct = (qAct - m_qActPre[i]) / m_dt;
    double vRef = (qRef - m_qRefPre[i]) / m_dt;

    m_tauRef.data[i] = m_pGain[i] * (qRef - qAct) + m_dGain[i] * (vRef - vAct);

    m_qActPre[i] = m_qAct.data[i];
    m_qRefPre[i] = m_qRef.data[i];
  }


  m_tauRefOut.write();
  return RTC::RTC_OK;
}


//-----------------------------------------------------------------------


extern "C"
{
  void creekPdServoInit(RTC::Manager * manager)
  {
    coil::Properties profile(module_spec);
    manager->registerFactory(profile,
                             RTC::Create<creekPdServo>,
                             RTC::Delete<creekPdServo>);
  }
};
