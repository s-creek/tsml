/*!
  @brief A component for accessing a joystick control device
*/

#include "GamepadRTC.h"

// Module specification
static const char* gamepad_spec[] =
  {
    "implementation_id", "GamepadRTC",
    "type_name",         "GamepadRTC",
    "description",       "Access a joystick control device.",
    "version",           "1.0.0",
    "vendor",            "AIST",
    "category",          "Human input",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.device", "/dev/input/js0",
    "conf.default.debugLevel", "0",
    ""
  };


GamepadRTC::GamepadRTC(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    m_axesOut("axes", m_axes),
    m_buttonsOut("buttons", m_buttons),
    m_debugLevel(0)
{
  m_joystick = 0;
}


GamepadRTC::~GamepadRTC()
{
  if(m_joystick){
    delete m_joystick;
  }
}


RTC::ReturnCode_t GamepadRTC::onInitialize()
{
  // Bind variables and configuration variable
  bindParameter("device", m_device, "/dev/input/js0");
  bindParameter("debugLevel", m_debugLevel, "0");

  // Set OutPort buffer
  addOutPort("axes", m_axesOut);
  addOutPort("buttons", m_buttonsOut);
  

  return RTC::RTC_OK;
}


RTC::ReturnCode_t GamepadRTC::onActivated(RTC::UniqueId ec_id)
{
  //for debug
  //m_debugLevel=1;

  if(m_debugLevel > 0){
    std::cout << "GamepadRTC::onActivated(" << ec_id << ")" << std::endl;
  }

  if(!m_joystick){
    m_joystick = new cnoid::Joystick(m_device.c_str());
  }
  if(!m_joystick->isReady()){
    std::cerr << "Gamepad device(" << m_device << ") is not opened" << std::endl;
    return RTC::RTC_ERROR;  
  } else {
    int n = m_joystick->numAxes();
    m_axes.data.length(n);
    for(int i=0; i < n; ++i){
      m_axes.data[i] = m_joystick->getPosition(i);
    }
    int m = m_joystick->numButtons();
    m_buttons.data.length(m);

    //std::cout<<"num "<<n<<" "<<m<<std::endl;

    for(int i=0; i < m; ++i){
      m_buttons.data[i] = m_joystick->getButtonState(i);
    }
    return RTC::RTC_OK;
  }
}


RTC::ReturnCode_t GamepadRTC::onDeactivated(RTC::UniqueId ec_id)
{
  if(m_debugLevel > 0){
    std::cout << "GamepadRTC::onDeactivated(" << ec_id << ")" << std::endl;
  }

  if(m_joystick){
    delete m_joystick;
    m_joystick = 0;
  }
    
  return RTC::RTC_OK;
}



RTC::ReturnCode_t GamepadRTC::onExecute(RTC::UniqueId ec_id)
{
  m_joystick->readCurrentState();
    
  if(m_debugLevel > 0){
    std::cout << "axes:";
  }
  const int n = m_joystick->numAxes();
  for(int i=0; i < n; ++i){
    m_axes.data[i] = m_joystick->getPosition(i);
    if(m_debugLevel > 0){
      std::cout << " " << m_axes.data[i];
    }
  }
  if(m_debugLevel > 0){
    std::cout << "\n" << "buttons:";
  }
  const int m = m_joystick->numButtons();
  //std::cout<<"num "<<n<<" "<<m<<std::endl;

  for(int i=0; i < m; ++i){
    m_buttons.data[i] = m_joystick->getButtonState(i);
    if(m_debugLevel > 0){
      std::cout << " " << m_buttons.data[i];
    }
  }
  if(m_debugLevel > 0){
    std::cout << std::endl;
  }

  m_axesOut.write();
  m_buttonsOut.write();

  return RTC::RTC_OK;
}


extern "C"
{
  void GamepadRTCInit(RTC::Manager* manager)
  {
    coil::Properties profile(gamepad_spec);
    manager->registerFactory(profile,
			     RTC::Create<GamepadRTC>,
			     RTC::Delete<GamepadRTC>);
  }
};
