// -*- C++ -*-

#include "creekReferenceHolder.h"

// Module specification
static const char* creekreferenceholder_spec[] =
  {
    "implementation_id", "creekReferenceHolder",
    "type_name",         "creekReferenceHolder",
    "description",       "creekReferenceHolder",
    "version",           "1.0",
    "vendor",            "s-creek",
    "category",          "example",
    "activity_type",     "SPORADIC",
    "kind",              "creekReferenceHolder",
    "max_instance",      "0",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    ""
  };

template <class T> double toSec(T t)
{
  return t.sec + t.nsec / 1000000000.0;
}

creekReferenceHolder::creekReferenceHolder(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    m_qCurIn("qCur", m_qCur),
    m_qIn ("qIn",  m_q),
    m_qOut("qOut", m_q),
    m_basePosIn ("basePosIn",  m_basePos),
    m_basePosOut("basePosOut", m_basePos),
    m_baseRpyIn ("baseRpyIn",  m_baseRpy),
    m_baseRpyOut("baseRpyOut", m_baseRpy),
    m_zmpRefIn ("zmpRefIn",  m_zmpRef),
    m_zmpRefOut("zmpRefOut", m_zmpRef)
{
}

creekReferenceHolder::~creekReferenceHolder()
{
}


RTC::ReturnCode_t creekReferenceHolder::onInitialize()
{
  std::cout << "creekReferenceHolder : onInitialize" << std::endl;

  addInPort("qCur", m_qCurIn);

  addInPort ("qIn",  m_qIn);
  addOutPort("qOut", m_qOut);

  addInPort ("basePosIn",  m_basePosIn);
  addOutPort("basePosOut", m_basePosOut);

  addInPort ("baseRpyIn",  m_baseRpyIn);
  addOutPort("baseRpyOut", m_baseRpyOut);

  addInPort ("zmpRefIn",  m_zmpRefIn);
  addOutPort("zmpRefOut", m_zmpRefOut);

  
  //
  // init data port
  //
  RTC::Properties& prop = getProperties();

  std::vector<double> tmp;
  coil::stringTo(tmp, prop["initBasePos"].c_str());
  m_basePos.data.x = tmp[0];
  m_basePos.data.y = tmp[1];
  m_basePos.data.z = tmp[2];

  tmp.clear();
  coil::stringTo(tmp, prop["initBaseRpy"].c_str());
  m_baseRpy.data.r = tmp[0];
  m_baseRpy.data.p = tmp[1];
  m_baseRpy.data.y = tmp[2];

  tmp.clear();
  coil::stringTo(tmp, prop["initZmp"].c_str());
  m_zmpRef.data.x = tmp[0];
  m_zmpRef.data.y = tmp[1];
  m_zmpRef.data.z = tmp[2];


  std::cout << "creekReferenceHolder : init data\n"
	    << "    base pos = " << m_basePos.data.x << ", " << m_basePos.data.y << ", " << m_basePos.data.z << "\n"
	    << "    base rpy = " << m_baseRpy.data.r << ", " << m_baseRpy.data.p << ", " << m_baseRpy.data.y << "\n"
	    << "    zmp pos  = " << m_zmpRef.data.x  << ", " << m_zmpRef.data.y  << ", " << m_zmpRef.data.z  << "\n";


  return RTC::RTC_OK;
}


RTC::ReturnCode_t creekReferenceHolder::onActivated(RTC::UniqueId ec_id)
{
  std::cout << "creekReferenceHolder : onActivated" << std::endl;

  if( m_qCurIn.isNew() ) {
    m_qCurIn.read();
    unsigned int dof = qCur.data.length();
    m_q.data.length(dof);
    memcpy(m_q.data.get_buffer(), m_qCur.data.get_buffer(), sizeof(double)*dof );
  }
  else {
    std::cout << "creekReferenceHolder : connection error" << std::endl;
    return RTC::RTC_ERROR;
  }
  return RTC::RTC_OK;
}


RTC::ReturnCode_t creekReferenceHolder::onDeactivated(RTC::UniqueId ec_id)
{
  std::cout << "creekReferenceHolder : onDeactivated" << std::endl;
  return RTC::RTC_OK;
}


RTC::ReturnCode_t creekReferenceHolder::onExecute(RTC::UniqueId ec_id)
{
  if( m_qCurIn.isNew() ) m_qCurIn.read();

  if( m_qIn.isNew() )       m_qIn.read();
  if( m_basePosIn.isNew() ) m_basePosIn.read();
  if( m_baseRpyIn.isNew() ) m_baseRpyIn.read();
  if( m_zmpRefIn.isNew() )  m_zmpRefIn.read();

  //std::cout << "creekReferenceHolder : time = " << toSec(m_qCur.tm) << std::endl;
  
  m_q.tm = m_qCur.tm;
  m_qOut.write();
  m_basePosOut.write();
  m_baseRpyOut.write();
  m_zmpRefOut.write();

  return RTC::RTC_OK;
}



extern "C"
{
 
  void creekReferenceHolderInit(RTC::Manager* manager)
  {
    coil::Properties profile(creekreferenceholder_spec);
    manager->registerFactory(profile,
                             RTC::Create<creekReferenceHolder>,
                             RTC::Delete<creekReferenceHolder>);
  }
  
};



