// -*- C++ -*-

#include "creekRobotState.h"

#include <cnoid/BodyLoader>
#include <cnoid/EigenUtil>

static const char* creekrobotstate_spec[] =
  {
    "implementation_id", "creekRobotState",
    "type_name",         "creekRobotState",
    "description",       "creekRobotState",
    "version",           "1.0",
    "vendor",            "s-creek",
    "category",          "example",
    "activity_type",     "SPORADIC",
    "kind",              "creekRobotState",
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


creekRobotState::creekRobotState(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    m_creekRobotStateServicePort("creekRobotStateService"),
    m_qIn("qIn", m_q),
    m_qOut("qOut", m_q),
    m_dqOut("dqOut", m_dq),
    m_ddqOut("ddqOut", m_ddq),
    m_basePosIn("basePos", m_basePos),
    m_baseRpyIn("baseRpy", m_baseRpy),
    m_basePoseOut("basePoseOut", m_basePose),
    m_qRefIn("qRef", m_qRef),
    m_baseRpyRefIn("baseRpyRef", m_baseRpyRef),
    m_zmpRefIn("zmpRef", m_zmpRef),
    m_rfsensorIn("rfsensor", m_force[0]),
    m_lfsensorIn("lfsensor", m_force[1]),
    m_rhsensorIn("rhsensor", m_force[2]),
    m_lhsensorIn("lhsensor", m_force[3])
{
  m_service0.setComponent(this);
}

creekRobotState::~creekRobotState()
{
}


RTC::ReturnCode_t creekRobotState::onInitialize()
{
  std::cout << "creekRobotState : onInitialize" << std::endl;

  addInPort("qIn", m_qIn);
  addOutPort("qOut", m_qOut);
  addOutPort("dqOut", m_dqOut);
  addOutPort("ddqOut", m_ddqOut);

  addInPort("basePos", m_basePosIn);
  addInPort("baseRpy", m_baseRpyIn);
  addOutPort("basePoseOut", m_basePoseOut);

  addInPort("qRef", m_qRefIn);
  addInPort("baseRpyRef", m_baseRpyRefIn);
  addInPort("zmpRef", m_zmpRefIn);
  addInPort("rfsensor", m_rfsensorIn);
  addInPort("lfsensor", m_lfsensorIn);
  addInPort("rhsensor", m_rhsensorIn);
  addInPort("lhsensor", m_lhsensorIn);

  m_creekRobotStateServicePort.registerProvider("service0", "creekRobotStateService", m_service0);
  addPort(m_creekRobotStateServicePort);


  // setup model
  RTC::Properties& prop = getProperties();
  cnoid::BodyLoader bl;
  m_robot=bl.load( prop["model"] );
  m_robotRef=bl.load( prop["model"] );


  // get sensors
  m_cameras = m_robot->devices();
  int numCamera = m_cameras.size();
  m_forceSensors = m_robot->devices();


  // set inport
  m_cameraPose.resize(numCamera);
  m_cameraPoseOut.resize(numCamera);
  for(int i=0; i<numCamera; i++) {
    std::string name = m_cameras[i]->name() + "Pose";
    m_cameraPoseOut[i] = new OutPort<TimedPose3D>(name.c_str(), m_cameraPose[i]);
    addOutPort(name.c_str(), *m_cameraPoseOut[i]);
  }


  m_q.data.length(m_robot->numJoints());
  m_dq.data.length(m_robot->numJoints());
  m_ddq.data.length(m_robot->numJoints());
  for(int i=0; i<m_robot->numJoints(); i++) {
    m_q.data[i]   = 0.0;
    m_dq.data[i]  = 0.0;
    m_ddq.data[i] = 0.0;
  }
  for(int i=0; i<4; i++) {
    m_force[i].data.length(6);
    for(int j=0; j<6; j++) {
      m_force[i].data[j] = 0.0;
    }
  }

  return RTC::RTC_OK;
}

RTC::ReturnCode_t creekRobotState::onActivated(RTC::UniqueId ec_id)
{
  std::cout << "creekRobotState : onActivated" << std::endl;

  m_basePose.data.position.x =  0.0;
  m_basePose.data.position.y =  0.0;
  m_basePose.data.position.z =  0.0;

  m_basePose.data.orientation.r = 0.0;
  m_basePose.data.orientation.p = 0.0;
  m_basePose.data.orientation.y = 0.0;

  m_basePoseOut.write();

  m_showState = false;

  return RTC::RTC_OK;
}

RTC::ReturnCode_t creekRobotState::onDeactivated(RTC::UniqueId ec_id)
{
  if( m_logfile.is_open() ) {
    m_logfile.close();
  }
  std::cout << "creekRobotState : onDeactivated" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t creekRobotState::onExecute(RTC::UniqueId ec_id)
{
  if( m_basePosIn.isNew() )  m_basePosIn.read();
  if( m_baseRpyIn.isNew() ) {
    m_baseRpyIn.read();

    m_basePose.data.position.x =  0.0;
    m_basePose.data.position.y =  0.0;
    m_basePose.data.position.z =  0.0;

    m_basePose.data.orientation.r = m_baseRpy.data.r;
    m_basePose.data.orientation.p = m_baseRpy.data.p;
    m_basePose.data.orientation.y = m_baseRpy.data.y;
  }


  if( m_qIn.isNew() ) {
    m_qIn.read();

    for(int i=0; i<m_robot->numJoints(); i++) {
      m_robot->joint(i)->q() = m_q.data[i];
    }
    m_robot->rootLink()->p() << m_basePos.data.x, m_basePos.data.y, m_basePos.data.z;
    m_robot->rootLink()->R() = cnoid::rotFromRpy(m_baseRpy.data.r, m_baseRpy.data.p, m_baseRpy.data.y);
    m_robot->calcForwardKinematics();

    for( int i=0; i<m_cameras.size(); i++) {
      cnoid::Vector3 p, rpy;
      cnoid::Matrix3 R;
      p = m_cameras[i]->link()->p() + m_cameras[i]->link()->R() * m_cameras[i]->p_local();
      R = m_cameras[i]->link()->R() * m_cameras[i]->R_local();
      rpy = R.eulerAngles(2,1,0);

      m_cameraPose[i].data.position.x = p(0);
      m_cameraPose[i].data.position.y = p(1);
      m_cameraPose[i].data.position.z = p(2);

      m_cameraPose[i].data.orientation.r = rpy(2);
      m_cameraPose[i].data.orientation.p = rpy(1);
      m_cameraPose[i].data.orientation.y = rpy(0);

      m_cameraPose[i].tm = m_q.tm;

      m_cameraPoseOut[i]->write();
    }
  }


  if( m_baseRpyRefIn.isNew() )  m_baseRpyRefIn.read();
  if( m_zmpRefIn.isNew() )      m_zmpRefIn.read();
  if( m_rfsensorIn.isNew() )    m_rfsensorIn.read();
  if( m_lfsensorIn.isNew() )    m_lfsensorIn.read();
  if( m_rhsensorIn.isNew() )    m_rhsensorIn.read();
  if( m_lhsensorIn.isNew() )    m_lhsensorIn.read();

  cnoid::Vector3 absZmpRef, absZmpAct;
  cnoid::Vector3 comRef, comAct;
  if( m_qRefIn.isNew() ) {
    m_qRefIn.read();
    m_qRef.tm = m_q.tm;
    
    for(int i=0; i<m_robot->numJoints(); i++) {
      m_robotRef->joint(i)->q() = m_qRef.data[i];
    }
    m_robotRef->rootLink()->p() << m_basePos.data.x, m_basePos.data.y, m_basePos.data.z;
    m_robotRef->rootLink()->R() = cnoid::rotFromRpy(m_baseRpyRef.data.r, m_baseRpyRef.data.p, m_baseRpyRef.data.y);
    m_robotRef->calcForwardKinematics();
    
    cnoid::Vector3 relZmpRef(m_zmpRef.data.x, m_zmpRef.data.y, m_zmpRef.data.z);
    absZmpRef = m_robotRef->rootLink()->p() + m_robotRef->rootLink()->R() * relZmpRef;
    
    calcZMP(absZmpAct, absZmpRef(2));
    comRef = m_robotRef->calcCenterOfMass();
    comAct = m_robot->calcCenterOfMass();
    
    if( m_logfile.is_open() ) {
      m_logfile << toSec(m_q.tm);
      for(int i=0; i<3; i++)  m_logfile << " " << absZmpRef(i);
      for(int i=0; i<3; i++)  m_logfile << " " << absZmpAct(i);
      for(int i=0; i<3; i++)  m_logfile << " " << comRef(i);
      for(int i=0; i<3; i++)  m_logfile << " " << comAct(i);
      m_logfile << " " << m_basePos.data.x << " " << m_basePos.data.y << " " << m_basePos.data.z;
      m_logfile << " " << m_baseRpy.data.r << " " << m_baseRpy.data.p << " " << m_baseRpy.data.y;
      m_logfile << " " << m_baseRpyRef.data.r << " " << m_baseRpyRef.data.p << " " << m_baseRpyRef.data.y;
      m_logfile << std::endl;
    }
  }
  



  if( m_showState ) {
    std::cout << "----------------------------------------------------------------------------" << std::endl;
    std::cout << "time = " << toSec(m_q.tm) << std::endl;
    std::cout << "base pos = [ " << m_basePos.data.x << ", " << m_basePos.data.y << ", " << m_basePos.data.z  << " ]" << std::endl;
    std::cout << "base rpy = [ " << m_baseRpy.data.r << ", " << m_baseRpy.data.p << ", " << m_baseRpy.data.y  << " ]" << std::endl;
    std::cout << std::endl << std::endl;
    std::cout << "base rpy ref = [ " << m_baseRpyRef.data.r << ", " << m_baseRpyRef.data.p << ", " << m_baseRpyRef.data.y  << " ]" << std::endl;
    for(int i=0; i<4; i++) {
      std::cout << m_forceSensors[i]->name() << " = [ " << m_force[i].data[0];
      for(int j=1; j<6; j++) {
	std::cout << ", " << m_force[i].data[j];
      }
      std::cout << " ]" << std::endl;
    }
    std::cout << "abs zmp ref = [ " << absZmpRef(0) << ", " << absZmpRef(1) << ", " << absZmpRef(2) << " ]" << std::endl;
    std::cout << "abs zmp act = [ " << absZmpAct(0) << ", " << absZmpAct(1) << ", " << absZmpAct(2) << " ]" << std::endl;
    std::cout << "com ref = [ " << comRef(0) << ", " << comRef(1) << ", " << comRef(2) << " ]" << std::endl;
    std::cout << "com ref = [ " << comAct(0) << ", " << comAct(1) << ", " << comAct(2) << " ]" << std::endl;
    std::cout << "----------------------------------------------------------------------------" << std::endl;
  }




  m_qOut.write();
  m_dqOut.write();
  m_ddqOut.write();
  m_basePoseOut.write();

  return RTC::RTC_OK;
}


void creekRobotState::showState()
{
  m_showState = !m_showState;
}


void creekRobotState::calcZMP(cnoid::Vector3& ret_zmp, const double zmp_z)
{
  double tmpzmpx = 0;
  double tmpzmpy = 0;
  double tmpfz = 0;
  for (size_t i = 0; i < 2; i++) {
    cnoid::ForceSensor* sensor = m_forceSensors[i];
    cnoid::Vector3 fsp = sensor->link()->p() + sensor->link()->R() * sensor->p_local();
    cnoid::Matrix3 tmpR(sensor->link()->R() * sensor->R_local());
    cnoid::Vector3 nf = tmpR * cnoid::Vector3(m_force[i].data[0], m_force[i].data[1], m_force[i].data[2]);
    cnoid::Vector3 nm = tmpR * cnoid::Vector3(m_force[i].data[3], m_force[i].data[4], m_force[i].data[5]);
    
    tmpzmpx += nf(2) * fsp(0) - (fsp(2) - zmp_z) * nf(0) - nm(1);
    tmpzmpy += nf(2) * fsp(1) - (fsp(2) - zmp_z) * nf(1) + nm(0);
    tmpfz += nf(2);
  }  
  ret_zmp = cnoid::Vector3(tmpzmpx / tmpfz, tmpzmpy / tmpfz, zmp_z);
}


void creekRobotState::logStart(std::string date)
{
  if( !m_logfile.is_open() ) {
    std::string filepath("/home/player/tsml/log/");
    filepath += (date+"_RobotState.log");
    m_logfile.open(filepath);
  }
}


extern "C"
{
 
  void creekRobotStateInit(RTC::Manager* manager)
  {
    coil::Properties profile(creekrobotstate_spec);
    manager->registerFactory(profile,
                             RTC::Create<creekRobotState>,
                             RTC::Delete<creekRobotState>);
  }
  
};



