// -*- C++ -*-
/*!
 * @file  creekGaitGenerator.cpp * @brief creekGaitGenerator * $Date$ 
 *
 * $Id$ 
 */
#include "creekGaitGenerator.h"
#include <creeklib/walk/Geometry.h>
#include <boost/make_shared.hpp>

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
  : RTC::DataFlowComponentBase(manager),
    m_qInitIn("qInit", m_qInit),
    m_basePosInitIn("basePosInit", m_basePosInit),
    m_baseRpyInitIn("baseRpyInit", m_baseRpyInit),
    m_zmpRefInitIn("zmpRefInit", m_zmpRefInit),
    m_qRefOut("qRef", m_qRef),
    m_basePosOut("basePos", m_basePos),
    m_baseRpyOut("baseRpy", m_baseRpy),
    m_zmpRefOut("zmpRef", m_zmpRef),
    m_contactStatesOut("contactStates", m_contactStates),
    m_creekGaitGeneratorServicePort("creekGaitGeneratorService")
{
  m_service0.setComp(this);
}


creekGaitGenerator::~creekGaitGenerator()
{
}


RTC::ReturnCode_t creekGaitGenerator::onInitialize()
{
  std::cout << " creekGaitGenerator : onInitialize" << std::endl;

  // Set InPort buffers
  addInPort("qInit", m_qInitIn);
  addInPort("basePosInit", m_basePosInitIn);
  addInPort("baseRpyInit", m_baseRpyInitIn);
  addInPort("zmpRefInit", m_zmpRefInitIn);

  // Set OutPort buffer
  addOutPort("qRef", m_qRefOut);
  addOutPort("basePos", m_basePosOut);
  addOutPort("baseRpy", m_baseRpyOut);
  addOutPort("zmpRef", m_zmpRefOut);
  addOutPort("contactStates", m_contactStatesOut);

  // Set service provider to Ports
  // Set CORBA Service Ports
  m_creekGaitGeneratorServicePort.registerProvider("service0", "creekGaitGeneratorService", m_service0);
  addPort(m_creekGaitGeneratorServicePort);

  
  //
  // get properties
  //
  RTC::Properties& prop = getProperties();


  // setup robot
  creek::BodyPtr robot( new creek::Body() );
  if( !creek::loadBody(robot, prop["model"]) ) {
    std::cerr << "error : model loader" << std::endl;
  }
  robot->rootLink()->p() << 0.0, 0.0, 0.854;
  
  m_biped = boost::make_shared<creek::BipedRobot>();
  m_biped->setRobot(robot);
  //m_biped->setJointPath("R_ANKLE_P", "PELVIS", "L_ANKLE_P", "PELVIS");
  m_biped->setJointPath(prop["RLEG_END"], prop["BASE_LINK"], prop["LLEG_END"], prop["BASE_LINK"]);
  
  creek::Vector3 offset;  offset << 0, 0, -0.108;
  m_biped->setSoleOffset(offset, offset);

  
  // setup planner
  double dt;
  coil::stringTo(dt, prop["dt"].c_str());
  
  m_planner = new creek::WalkPlanner(dt);
  m_planner->setTime(0.7, 0.1);
  m_planner->setFootSize(0.160, 0.100, 0.055, 0.055);
  m_planner->init(m_biped);
  //m_planner->calcOffset(0.19, 0.8, m_planner->capturePoint()->comHeight());
  m_planner->setOffset(0.015);


  // init data port
  m_qRef.data.length(robot->numJoints());
  m_contactStates.data.length(2);
  m_contactStates.data[0]=m_contactStates.data[1]=1;
  

  return RTC::RTC_OK;
}


RTC::ReturnCode_t creekGaitGenerator::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t creekGaitGenerator::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t creekGaitGenerator::onExecute(RTC::UniqueId ec_id)
{
  if( m_qInitIn.isNew() )       m_qInitIn.read();
  if( m_basePosInitIn.isNew() ) m_basePosInitIn.read();
  if( m_baseRpyInitIn.isNew() ) m_baseRpyInitIn.read();
  if( m_zmpRefInitIn.isNew() )  m_zmpRefInitIn.read();


  // set data
  if( m_planner->empty() ) {
    creek::FootType swing_foot_type;
    creek::StepData step;
    m_planner->getGoal(step);
    
  
    // add step
    if( m_planner->isStepping() ) {
      step.sup = creek::reverseFoot(step.sup);
      if( step.sup == creek::DFOOT )
	step.sup = creek::LFOOT;
  
      step.stepType = creek::CYCLOID_STEP;
      m_planner->addStep(step);
    }
    else if( (swing_foot_type = m_planner->getSwingFootType(step.rfoot, step.lfoot)) != creek::AIR ) {
      switch(swing_foot_type)
	{
	case creek::RFOOT:
	case creek::LFOOT:
	  {
	    step.sup = creek::reverseFoot(swing_foot_type);
	    m_planner->addStep(step);
	  }
	  break;
	case creek::DFOOT:
	default:
	  m_planner->stop();
	}
    }
    else {
      return RTC::RTC_OK;
    }
  }


  // get data
  {
    creek::StepData step;
    m_planner->get(step);
 
    m_waistRef = creek::midYaw(step.rfoot.linear(), step.lfoot.linear());
    if( !m_biped->calc(step.sup, step.com, m_waistRef,
		       step.rfoot.translation(), step.rfoot.linear(),
		       step.lfoot.translation(), step.lfoot.linear()) ) {
      std::cout << "creekGaitPatternGeneratorCp : com inverse error" << std::endl;
    }
    else {
      for(int i=0; i<m_biped->body()->numJoints(); i++) {
	m_qRef.data[i] = m_biped->body()->joint(i)->q();
      }
      m_basePos.data.x = m_biped->body()->rootLink()->p()[0];
      m_basePos.data.y = m_biped->body()->rootLink()->p()[1];
      m_basePos.data.z = m_biped->body()->rootLink()->p()[2];

      creek::Vector3 rpy = creek::rpyFromRot(m_biped->body()->rootLink()->R());
      m_baseRpy.data.r = rpy[0];
      m_baseRpy.data.p = rpy[1];
      m_baseRpy.data.y = rpy[2];

      creek::Vector3 zmpRel = m_biped->body()->rootLink()->R().transpose() * (step.zmp - m_biped->body()->rootLink()->p());
      m_zmpRef.data.x = zmpRel[0];
      m_zmpRef.data.y = zmpRel[1];
      m_zmpRef.data.z = zmpRel[2];

      switch( step.sup )
	{
	case creek::RFOOT:
	  m_contactStates.data[0] = 1;
	  m_contactStates.data[1] = 0;
	  break;
	case creek::LFOOT:
	  m_contactStates.data[0] = 0;
	  m_contactStates.data[1] = 1;
	  break;
	case creek::DFOOT:
	  m_contactStates.data[0] = 1;
	  m_contactStates.data[1] = 1;
	  break;
	default:
	  m_contactStates.data[0] = 0;
	  m_contactStates.data[1] = 0;
	}


      m_qRefOut.write();
      m_basePosOut.write();
      m_baseRpyOut.write();
      m_zmpRefOut.write();
      m_contactStatesOut.write();
    }
  }

  return RTC::RTC_OK;
}


//-----------------------------------------------------------------------------


void creekGaitGenerator::init()
{
  if( !m_planner->empty() )
    return;


  for(int i=0; i<m_biped->body()->numJoints(); i++) {
    m_biped->body()->joint(i)->q() = m_qInit.data[i];
  }
  m_biped->body()->rootLink()->p() << m_basePosInit.data.x, m_basePosInit.data.y, m_basePosInit.data.z;
  m_biped->body()->rootLink()->R() = creek::rotFromRpy(m_baseRpyInit.data.r, m_baseRpyInit.data.p, m_baseRpyInit.data.y);
  m_biped->body()->calcForwardKinematics();


  m_planner->init(m_biped);
  m_planner->calcOffset(0.19, 0.8, m_planner->capturePoint()->comHeight());
  //m_planner->setOffset(0.015);

  m_waistRef = m_biped->body()->rootLink()->R();
  m_rfootRef = m_biped->rfoot()->position();
  m_lfootRef = m_biped->lfoot()->position();
}


void creekGaitGenerator::startStepping()
{
  init();
  m_planner->setStepping(true);
}


void creekGaitGenerator::stopStepping()
{
  m_planner->stop();
}


void creekGaitGenerator::test()
{
  init();

  creek::StepData step;
  m_planner->getGoal(step);
  step.stepType = creek::CYCLOID_STEP;

  // swing right foot
  m_rfootRef.translation()(0) += 0.1;
  step.rfoot = m_rfootRef;
  step.sup = creek::LFOOT;
  m_planner->addStep(step);

  // swing left foot
  m_lfootRef.translation()(0) += 0.2;
  step.lfoot = m_lfootRef;
  step.sup = creek::RFOOT;
  m_planner->addStep(step);

  // swing right foot
  m_rfootRef.translation()(0) += 0.2;
  step.rfoot = m_rfootRef;
  step.sup = creek::LFOOT;
  m_planner->addStep(step);

  // swing left foot
  m_lfootRef.translation()(0) += 0.2;
  step.lfoot = m_lfootRef;
  step.sup = creek::RFOOT;
  m_planner->addStep(step);

  // swing right foot
  m_rfootRef.translation()(0) += 0.1;
  step.rfoot = m_rfootRef;
  step.sup = creek::LFOOT;
  m_planner->addStep(step);

  // stop
  //m_planner->stop();
}


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



