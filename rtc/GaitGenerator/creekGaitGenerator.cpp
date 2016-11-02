// -*- C++ -*-
/*!
 * @file  creekGaitGenerator.cpp * @brief creekGaitGenerator * $Date$ 
 *
 * $Id$ 
 */
#include "creekGaitGenerator.h"

// Module specification
// <rtc-template block="module_spec">
static const char* creekgaitgenerator_spec[] =
  {
    "implementation_id", "creekGaitGenerator",
    "type_name",         "creekGaitGenerator",
    "description",       "creekGaitGenerator",
    "version",           "1.0",
    "vendor",            "s-creek",
    "category",          "example",
    "activity_type",     "SPORADIC",
    "kind",              "creekGaitGenerator",
    "max_instance",      "0",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    ""
  };
// </rtc-template>

creekGaitGenerator::creekGaitGenerator(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_creekGaitGeneratorServicePort("creekGaitGeneratorService")

    // </rtc-template>
{
}

creekGaitGenerator::~creekGaitGenerator()
{
}


RTC::ReturnCode_t creekGaitGenerator::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers

  // Set OutPort buffer

  // Set service provider to Ports
  m_creekGaitGeneratorServicePort.registerProvider("service0", "creekGaitGeneratorService", m_service0);

  // Set service consumers to Ports

  // Set CORBA Service Ports
  addPort(m_creekGaitGeneratorServicePort);

  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable

  // </rtc-template>
  return RTC::RTC_OK;
}


/*
RTC::ReturnCode_t creekGaitGenerator::onFinalize()
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t creekGaitGenerator::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t creekGaitGenerator::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t creekGaitGenerator::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t creekGaitGenerator::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t creekGaitGenerator::onExecute(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t creekGaitGenerator::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t creekGaitGenerator::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t creekGaitGenerator::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t creekGaitGenerator::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t creekGaitGenerator::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


extern "C"
{
 
  void creekGaitGeneratorInit(RTC::Manager* manager)
  {
    coil::Properties profile(creekgaitgenerator_spec);
    manager->registerFactory(profile,
                             RTC::Create<creekGaitGenerator>,
                             RTC::Delete<creekGaitGenerator>);
  }
  
};



