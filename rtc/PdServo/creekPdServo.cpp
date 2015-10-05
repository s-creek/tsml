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
    m_qCurIn("q", m_qCur),
    m_tauRefOut("tau", m_tauRef),
    m_calcGainMode(0)
{
}


RTC::ReturnCode_t creekPdServo::onInitialize()
{
  std::cout << "creekPdServo : onInitialize" << std::endl;


  bindParameter("calcGainMode", m_calcGainMode, "0");
  //
  // Registration: InPort/OutPort/Service
  //
  // Set InPort buffers
  addInPort("qRef", m_qRefIn);
  addInPort("q", m_qCurIn);
  
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
  // get the reference of nameservice
  //
  RTC::Manager& rtcManager = RTC::Manager::instance();
  std::string nameServer = rtcManager.getConfig()["corba.nameservers"];
  int commaPos = nameServer.find(",");
  if (commaPos > 0)
    nameServer = nameServer.substr(0, commaPos);
  RTC::CorbaNaming naming(rtcManager.getORB(), nameServer.c_str());
  CosNaming::NamingContext_var m_rootNameContext = CosNaming::NamingContext::_duplicate(naming.getRootContext());


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
    
    m_maxErr.resize(m_dof);
    m_count.resize(m_dof);
    for(int i=0; i<m_maxErr.size(); i++) {
      m_maxErr[i] = 0.0;
      m_count[i] = 0;
    }
  }

  m_qPre.resize(m_dof);
  m_qRefPre.resize(m_dof);

  m_qCur.data.length(m_dof);
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
  if( m_qCurIn.isNew() ) {
    m_qCurIn.read();

    for(int i = 0; i < m_dof; i++) {
      m_qPre[i]      = m_qCur.data[i];
      m_qRefPre[i]   = m_qCur.data[i];
      m_qRef.data[i] = m_qCur.data[i];
    }
    for(int i=0; i<m_maxErr.size(); i++) {
      m_maxErr[i] = 0.0;
      m_count[i] = 0;
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
  //
  // for calc gain mode
  //
  static double maxGain[] = {20000,20000,20000,20000,20000,20000,
			     20000,20000,20000,20000,20000,20000,
			     10000,10000,10000,
			     2000,2000,2000,
			     10000,10000,10000,10000,10000,10000,10000, 300,300,300,300,300,300,
			     10000,10000,10000,10000,10000,10000,10000, 300,300,300,300,300,300};
  
  // air
  //static double thErr[] = {0.59626, 0.509112, 0.362656, 0.527227, 0.313177, 0.327363, 0.59626, 0.551913, 0.315458, 0.527227, 0.310908, 0.327363, 0.5349, 0.509666, 0.502582, 0.171652, 0.328412, 0.426725, 0.531603, 0.504394, 0.207968, 0.499999, 0.170974, 0.490664, 0.132161, 0.153571, 0.270139, 0.265642, 0.175156, 0.265642, 0.175156, 0.53146, 0.501976, 0.218838, 0.499999, 0.155884, 0.490908, 0.132214, 0.153542, 0.270132, 0.26566, 0.175161, 0.265659, 0.175161};
  
  // ground
  static double thErr[] = {0.528185,0.808676,0.525903,0.594445,0.503274,0.671872,0.481917,0.864954,0.505766,0.581467,0.479635,0.802765,0.471956,0.236085,0.403944,0.117391,0.227474,0.213028,0.128676,0.213394,0.479941,0.27255,0.0523233,0.438349,0.057503,0.0597337,0.0165583,0.0565178,0.0111312,0.0518545,0.0101693,0.119548,0.218813,0.665238,0.261283,0.0513789,0.438549,0.0518852,0.0615148,0.0166798,0.0589486,0.011668,0.0536421,0.010565};



  if( m_qCurIn.isNew() )  m_qCurIn.read();
  if( m_qRefIn.isNew() )  m_qRefIn.read();

  bool updateGain(false);
  for( int i = 0; i < m_dof; i++) {
    double qCur = m_qCur.data[i];
    double qRef = m_qRef.data[i];
    double vCur = (qCur - m_qPre[i]) / m_dt;
    double vRef = (qRef - m_qRefPre[i]) / m_dt;

    m_tauRef.data[i] = m_pGain[i] * (qRef - qCur) + m_dGain[i] * (vRef - vCur);

    m_qPre[i]    = m_qCur.data[i];
    m_qRefPre[i] = m_qRef.data[i];
    

    double e = (m_qRefPre[i] - qCur)/M_PI*180.0;
    if( m_calcGainMode > 0 ) {
      if( std::fabs(e) > thErr[i] + 0.05 && maxGain[i] > m_pGain[i] ) {
	//std::cout << i << " : " << e << std::endl;
	m_pGain[i] += 1.0;
	m_dGain[i] = (int)(m_pGain[i] / 100.0);
	updateGain = true;
	m_count[i] += 1;
      }
      if( std::fabs(e) > m_maxErr[i] )
	m_maxErr[i] = std::fabs(e);
    }
  }

  if( (updateGain && false) || m_calcGainMode > 1 ) {
    std::cout << std::endl;
    std::cout << m_pGain[0];
    for(int i=1; i<m_pGain.size(); i++)
      std::cout << "," << m_pGain[i];
    std::cout << std::endl << std::endl;

    std::cout << m_dGain[0];
    for(int i=1; i<m_dGain.size(); i++)
      std::cout << "," << m_dGain[i];
    std::cout << std::endl << std::endl;

    std::cout << m_maxErr[0];
    for(int i=1; i<m_maxErr.size(); i++)
      std::cout << "," << m_maxErr[i];
    std::cout << std::endl << std::endl;
    
    std::cout << m_count[0];
    for(int i=1; i<m_count.size(); i++)
      std::cout << ", " << m_count[i];
    std::cout << std::endl << std::endl;
    
    std::cout << "--------------------------------------------------------" << std::endl << std::endl;;
    m_calcGainMode -= 1;
  }
  if( m_calcGainMode > 1 ) {
    for(int i=0; i<m_maxErr.size(); i++) {
      m_maxErr[i] = 0.0;
      m_count[i] = 0;
    }
    m_calcGainMode = 1;
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
