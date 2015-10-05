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


creekRobotState::creekRobotState(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    m_creekRobotStateServicePort("creekRobotStateService"),
    m_qIn("qIn", m_q),
    m_qOut("qOut", m_q),
    m_dqOut("dqOut", m_dq),
    m_ddqOut("ddqOut", m_ddq),
    m_basePosIn("basePos", m_basePos),
    m_baseRpyIn("baseRpy", m_baseRpy),
    m_basePoseOut("basePoseOut", m_basePose)
{
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

  m_creekRobotStateServicePort.registerProvider("service0", "creekRobotStateService", m_service0);
  addPort(m_creekRobotStateServicePort);


  // setup model
  RTC::Properties& prop = getProperties();
  cnoid::BodyLoader bl;
  m_robot=bl.load( prop["model"] );


  // get cameras
  m_cameras = m_robot->devices();
  int numCamera = m_cameras.size();
  

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

  return RTC::RTC_OK;
}

RTC::ReturnCode_t creekRobotState::onDeactivated(RTC::UniqueId ec_id)
{
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

  m_qOut.write();
  m_dqOut.write();
  m_ddqOut.write();
  m_basePoseOut.write();

  return RTC::RTC_OK;
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



