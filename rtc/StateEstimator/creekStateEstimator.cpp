// 
// reference url : http://tom.pycke.be/mav/71/kalman-filtering-of-imu-data
//
#include "creekStateEstimator.h"
//#include "../util/CheckCounter.h"

// tvmet (from OpenHRP)
//#include <tvmet/Matrix.h>
//#include <tvmet/Vector.h>
//typedef tvmet::Matrix<double, 3, 3> Matrix33;
//typedef tvmet::Vector<double, 3> Vector3;

using Eigen::Matrix3d;
using Eigen::Vector3d;
typedef Eigen::Matrix3d Matrix33;
typedef Eigen::Vector3d Vector3;

void calcRotFromRpy(Matrix33& out_R, double r, double p, double y)
{
    const double cr = cos(r), sr = sin(r), cp = cos(p), sp = sin(p), cy = cos(y), sy = sin(y);
    out_R(0,0)= cp*cy;
    out_R(0,1)= sr*sp*cy - cr*sy;
    out_R(0,2)= cr*sp*cy + sr*sy;
    out_R(1,0)= cp*sy;
    out_R(1,1)= sr*sp*sy + cr*cy;
    out_R(1,2)= cr*sp*sy - sr*cy;
    out_R(2,0)= -sp;
    out_R(2,1)= sr*cp;
    out_R(2,2)= cr*cp;
}


// Module specification
static const char* module_spec[] =
  {
    "implementation_id", "creekStateEstimator",
    "type_name",         "creekStateEstimator",
    "description",       "Sequence InPort component",
    "version",           "1.0",
    "vendor",            "s-creek",
    "category",          "example",
    "activity_type",     "SPORADIC",
    "kind",              "SequenceInComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    ""
  };


creekStateEstimator::creekStateEstimator(RTC::Manager * manager)
  : RTC::DataFlowComponentBase(manager),
    m_accSensorIn("acc", m_accSensor),
    m_gyroSensorIn("rate", m_gyroSensor),
    m_accRefIn("accRef", m_accRef),
    m_rpyOut("rpy", m_rpy)
{
}


RTC::ReturnCode_t creekStateEstimator::onInitialize()
{
  std::cout << "creekStateEstimator : onInitialize" << std::endl;

  //
  // set port
  //
  addInPort("acc",     m_accSensorIn);
  addInPort("rate",    m_gyroSensorIn);
  addInPort("accRef",  m_accRefIn);

  addOutPort("rpy",  m_rpyOut);

  m_accSensor.data.length(3);
  m_gyroSensor.data.length(3);
  m_accRef.data.length(3);
  //m_rpy.data.length(3);

  for(int i=0; i<3; i++) {
    m_accSensor.data[i]  = 0.0;
    m_gyroSensor.data[i] = 0.0;
    m_accRef.data[i]     = 0.0;
    //m_rpy.data[i]        = 0.0;
  }
  RTC::Properties& prop = getProperties();
  coil::vstring tmp = coil::split( prop["initBaseRpy"], "," );
  std::vector<double> initrpy(tmp.size());
  std::cout << "creekStateEstimator :";
  for( int i=0; i<tmp.size(); i++) {
    coil::stringTo(initrpy[i], tmp[i].c_str());
    std::cout << " [" << tmp[i] << ", " << initrpy[i] << "] ";
  }
  std::cout << std::endl;
  m_rpy.data.r=initrpy[0];  m_rpy.data.p=initrpy[1];  m_rpy.data.y=initrpy[2];
  //m_rpy.data.r=0.0;  m_rpy.data.p=0.0;  m_rpy.data.y=0.0;


  //
  // set default param
  //
  //RTC::Properties& prop = getProperties();
  if ( ! coil::stringTo(m_dt, prop["dt"].c_str()) ) {
    std::cerr << "creekStateEstimator failed to prop[dt] " << prop["dt"] << "" << std::endl;
    return RTC::RTC_ERROR;
  }
  //SET_CHECK_COUNTER;


  for(int i=0; i<3; i++) {
    m_kf[i].setP(0,0,0,0);
    m_kf[i].setQ(0.001*m_dt, 0, 0, 0.003*m_dt);
    //m_kf[i].setR(0.3);
    m_kf[i].setR(0.03);
    m_kf[i].setA(1, -m_dt, 0, 1);
    m_kf[i].setB(m_dt, 0);
    m_kf[i].setX(initrpy[i]);
  }
  

  return RTC::RTC_OK;
}


RTC::ReturnCode_t creekStateEstimator::onActivated(RTC::UniqueId ec_id)
{
  std::cout << "creekStateEstimator : onActivated" << std::endl;
  //cc::m_stepCounter=0;
  return RTC::RTC_OK;
}


RTC::ReturnCode_t creekStateEstimator::onDeactivated(RTC::UniqueId ec_id)
{
  std::cout << "creekStateEstimator : onDeactivated" << std::endl;
  return RTC::RTC_OK;
}


RTC::ReturnCode_t creekStateEstimator::onExecute(RTC::UniqueId ec_id)
{
  if( m_accSensorIn.isNew() )   m_accSensorIn.read();
  if( m_gyroSensorIn.isNew() )  m_gyroSensorIn.read();
  
  if( m_accRefIn.isNew() ) {
    m_accRefIn.read();
  }
  else {
    //for(int i=0; i<3; i++) m_accRef.data[i] = 0.0;
  }


  //
  // sync
  //
  //CHECK_COUNTER(cc::m_stepCounter);


  //
  // gravity = sensor - ref
  //
  Vector3 gacc;  // = ax, ay, az
  for(int i=0; i<3; i++) {
    gacc(i) = m_accSensor.data[i] - m_accRef.data[i];
  }
  //double g = tvmet::norm2(gacc);
  double g = gacc.norm();


  //
  // gyro (on world)
  //
  Matrix33 rot; //( rotFromRpy(m_rpy.data[0], m_rpy.data[1], m_rpy.data[2]) );
  //calcRotFromRpy( rot, m_rpy.data[0], m_rpy.data[1], m_rpy.data[2] );
  calcRotFromRpy( rot, m_rpy.data.r, m_rpy.data.p, m_rpy.data.y );
  Vector3 gyro( rot * Vector3(m_gyroSensor.data[0], m_gyroSensor.data[1], m_gyroSensor.data[2]) );


  //
  // ax = -sin(p) * g
  // ay = cos(p) * sin(r) * g
  // az = cos(p) * cos(r) * g
  //
  Vector3 rpy;
  rpy(1) = atan2(-gacc(0), sqrt(gacc(1)*gacc(1) + gacc(2)*gacc(2)));  // p : -90～90 [deg]  ->  cos(p) >= 0
  rpy(0) = atan2(gacc(1), gacc(2));
  //rpy(2) = m_rpy.data[2] + m_dt*gyro(2);
  rpy(2) = m_rpy.data.y + m_dt*gyro(2);
  

  //
  // update
  //
  //for(int i=0; i<3; i++)  m_rpy.data[i] = m_kf[i].filtering(rpy(i), m_gyroSensor.data[i]);
  m_rpy.data.r = m_kf[0].filtering(rpy(0), m_gyroSensor.data[0]);
  m_rpy.data.p = m_kf[1].filtering(rpy(1), m_gyroSensor.data[1]);
  m_rpy.data.y = m_kf[2].filtering(rpy(2), m_gyroSensor.data[2]);

  
  /*
  // debug
  rpy << m_rpy.data.r, m_rpy.data.p, m_rpy.data.y;
  Vector3 rpyDeg;
  rpyDeg = rpy / M_PI * 180.0;
  std::cout << "creekStateEstimator : " << rpyDeg(0) << ", " << rpyDeg(1) << ", " << rpyDeg(2) << std::endl;
  */  

  m_rpyOut.write();
  return RTC::RTC_OK;
}


//-----------------------------------------------------------------------


extern "C"
{
  void creekStateEstimatorInit(RTC::Manager* manager)
  {
    coil::Properties profile(module_spec);
    manager->registerFactory(profile,
                             RTC::Create<creekStateEstimator>,
                             RTC::Delete<creekStateEstimator>);
  }  
};
