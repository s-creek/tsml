// -*- C++ -*-

#include "creekSequencePlayer.h"
#include <creeklib/model/modelLoader.hpp>

#ifdef USE_CNOID_MODEL
#include <cnoid/EigenUtil>
#else 
#include <hrpUtil/Tvmet3d.h>
#endif

static const char* creeksequenceplayer_spec[] =
  {
    "implementation_id", "creekSequencePlayer",
    "type_name",         "creekSequencePlayer",
    "description",       "SequencePlayer",
    "version",           "1.0",
    "vendor",            "s-creek",
    "category",          "example",
    "activity_type",     "SPORADIC",
    "kind",              "SequencePlayer",
    "max_instance",      "0",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    ""
  };


Eigen::IOFormat iovec(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]");


creekSequencePlayer::creekSequencePlayer(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    m_qInitIn("qInit", m_qInit),
    m_basePosInitIn("basePosInit", m_basePosInit),
    m_baseRpyInitIn("baseRpyInit", m_baseRpyInit),
    m_zmpRefInitIn("zmpRefInit", m_zmpRefInit),
    m_qRefOut("qRef", m_qRef),
    m_basePosOut("basePos", m_basePos),
    m_baseRpyOut("baseRpy", m_baseRpy),
    m_zmpRefOut("zmpRef", m_zmpRef),
    m_creekSequencePlayerServicePort("creekSequencePlayerService"),
    m_waitFlag(false),
    m_waitSem(0)
{
  m_service0.setComponent(this);
}

creekSequencePlayer::~creekSequencePlayer()
{
}


RTC::ReturnCode_t creekSequencePlayer::onInitialize()
{
  std::cout << "creekSequencePlayer : onInitialize" << std::endl;

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

  // Set service provider to Ports
  // Set CORBA Service Ports
  m_creekSequencePlayerServicePort.registerProvider("service0", "creekSequencePlayerService", m_service0);
  addPort(m_creekSequencePlayerServicePort);


  //
  // get properties
  //
  RTC::Properties& prop = getProperties();
  coil::stringTo(m_dt, prop["dt"].c_str());
  //coil::stringTo(m_dt, prop["pdservo.dt"].c_str());


  //
  // setup robot model
  //
  //m_robot = new creek::Body();
  m_robot = creek::BodyPtr(new creek::Body());
  creek::loadBody( m_robot, prop["model"] );
  m_dof = m_robot->numJoints();

  m_rleg = m_robot->getJointPath(m_robot->rootLink(), m_robot->link(prop["RLEG_END"]) );
  m_lleg = m_robot->getJointPath(m_robot->link(prop["LLEG_END"]), m_robot->rootLink() );


  //
  // init interpolator
  //
  m_seq[ANGLES] = new creek::Interpolator(m_dof, m_dt);
  for(int i=POS; i<NUM_SEQ; i++)
    m_seq[i] = new creek::Interpolator(3, m_dt);


  //
  // init data port
  //
  m_qRef.data.length(m_dof);
  m_qInit.data.length(m_dof);
  for(unsigned int i=0; i<m_dof; i++) m_qInit.data[i] = 0.0;


  //SET_CHECK_COUNTER;
  return RTC::RTC_OK;
}


RTC::ReturnCode_t creekSequencePlayer::onActivated(RTC::UniqueId ec_id)
{
  std::cout << "creekSequencePlayer : onActivated" << std::endl;

  if( m_qInitIn.isNew() )       m_qInitIn.read();
  if( m_basePosInitIn.isNew() ) m_basePosInitIn.read();
  if( m_baseRpyInitIn.isNew() ) m_baseRpyInitIn.read();
  if( m_zmpRefInitIn.isNew() )  m_zmpRefInitIn.read();
  

  if( isEmpty() && m_waitFlag ) {
    m_waitFlag = false;
    m_waitSem.post();
  }

 
  return RTC::RTC_OK;
}


RTC::ReturnCode_t creekSequencePlayer::onDeactivated(RTC::UniqueId ec_id)
{
  std::cout << "creekSequencePlayer : onDeactivated" << std::endl;
  return RTC::RTC_OK;
}


RTC::ReturnCode_t creekSequencePlayer::onExecute(RTC::UniqueId ec_id)
{
  if( m_qInitIn.isNew() )       m_qInitIn.read();
  if( m_basePosInitIn.isNew() ) m_basePosInitIn.read();
  if( m_baseRpyInitIn.isNew() ) m_baseRpyInitIn.read();
  if( m_zmpRefInitIn.isNew() )  m_zmpRefInitIn.read();


  if( isEmpty() && m_waitFlag ) {
    m_waitFlag = false;
    m_waitSem.post();
  }


  if( !m_seq[ANGLES]->empty() ) {
    m_seq[ANGLES]->get(m_qRef.data.get_buffer());
    m_qRefOut.write();
  }


  double tmp[3];
  if( !m_seq[POS]->empty() )  {
    m_seq[POS]->get(&tmp[0]);
    m_basePos.data.x = tmp[0];
    m_basePos.data.y = tmp[1];
    m_basePos.data.z = tmp[2];
    m_basePosOut.write();
  }

  if( !m_seq[RPY]->empty() )  {
    m_seq[RPY]->get(&tmp[0]);
    m_baseRpy.data.r = tmp[0];
    m_baseRpy.data.p = tmp[1];
    m_baseRpy.data.y = tmp[2];
    m_baseRpyOut.write();
  }

  if( !m_seq[ZMP]->empty() )  {
    m_seq[ZMP]->get(&tmp[0]);
    m_zmpRef.data.x = tmp[0];
    m_zmpRef.data.y = tmp[1];
    m_zmpRef.data.z = tmp[2];
    m_zmpRefOut.write();
  }



  return RTC::RTC_OK;
}


//----------------------------------------------------------------------------------


void creekSequencePlayer::waitInterpolation()
{
  m_waitFlag = true;
  m_waitSem.wait();
}


bool creekSequencePlayer::setJointAngles(const double *angles, double tm)
{
  if( m_qInit.data.length() != m_dof ) return false;

  // if( m_seq[ANGLES]->empty() ) {
  //   return m_seq[ANGLES]->calcInterpolation(m_qInit.data.get_buffer(), angles, tm);
  // }
  // else
  //   return false;


  if( m_seq[POS]->empty() && m_seq[ANGLES]->empty() && m_seq[ZMP]->empty() ) {
    // get initial data
    creek::Vector3 zmpInitRel;  zmpInitRel << m_zmpRefInit.data.x, m_zmpRefInit.data.y, m_zmpRefInit.data.z;
    creek::Vector3 posInit;  posInit << m_basePosInit.data.x, m_basePosInit.data.y, m_basePosInit.data.z;
    creek::Matrix3 rotInit( model::rotFromRpy(m_baseRpyInit.data.r, m_baseRpyInit.data.p, m_baseRpyInit.data.y) );
    creek::Vector3 zmpInitAbs(posInit + rotInit * zmpInitRel);


    // set current model
    m_robot->rootLink()->p() = posInit;
    m_robot->rootLink()->R() = rotInit;
    for(unsigned int i=0; i<m_dof; i++)
      m_robot->joint(i)->q() = m_qInit.data[i];
    m_robot->calcForwardKinematics();


    // calc reference model
    for(unsigned int i=0; i<m_dof; i++)
      m_robot->joint(i)->q() = angles[i];
    m_lleg->calcForwardKinematics();
    m_robot->calcForwardKinematics();
   
    creek::Vector3 zmpRefRel( m_robot->rootLink()->R().transpose() * (zmpInitAbs-m_robot->rootLink()->p()) );
    
    m_seq[ANGLES]->calc(m_qInit.data.get_buffer(), angles, tm, creek::CUBIC);
    m_seq[POS]->calc(posInit.data(), m_robot->rootLink()->p().data(), tm, creek::CUBIC);
    m_seq[ZMP]->calc(zmpInitRel.data(), zmpRefRel.data(), tm, creek::CUBIC);

    std::cout << "seq : ref base pos = " << m_robot->rootLink()->p().format(iovec) << std::endl;
    return true;
  }
  else
    return false;
}


bool creekSequencePlayer::setJointAngle(const char* jname, double jv, double tm)
{
  if( m_qInit.data.length() != m_dof ) return false;
  
  if( m_seq[ANGLES]->empty() ) {
    creek::Link *link = m_robot->link(jname);
    if( link ) {
      double angles[m_dof];
      std::memcpy(&angles[0], m_qInit.data.get_buffer(), m_dof*sizeof(double));
      angles[link->jointId()] = jv;

      m_seq[ANGLES]->calc(m_qInit.data.get_buffer(), angles, tm, creek::CUBIC);
      return true;
    }
  }

  return false;
}


bool creekSequencePlayer::setBasePos(const double *pos, double tm)
{
  // if( !m_seq[POS]->empty() ) 
  //   return false;
  // else {
  //   cnoid::Vector3 posInit(m_basePosInit.data.x, m_basePosInit.data.y, m_basePosInit.data.z);
  //   //std::cout << "creekSequencePlayer : init pos = " << posInit[0] << ", " << posInit[1] << ", " << posInit[2] << std::endl;
  //   return m_seq[POS]->calcInterpolation(posInit.data(), pos, tm);
  // }


  if( m_seq[POS]->empty() && m_seq[ANGLES]->empty() && m_seq[ZMP]->empty() ) {
    // get initial data
    creek::Vector3 zmpInitRel;  zmpInitRel << m_zmpRefInit.data.x, m_zmpRefInit.data.y, m_zmpRefInit.data.z;
    creek::Vector3 posInit;  posInit << m_basePosInit.data.x, m_basePosInit.data.y, m_basePosInit.data.z;
    creek::Matrix3 rotInit( model::rotFromRpy(m_baseRpyInit.data.r, m_baseRpyInit.data.p, m_baseRpyInit.data.y) );
    creek::Vector3 zmpInitAbs(posInit + rotInit * zmpInitRel);


    // set current model
    m_robot->rootLink()->p() = posInit;
    m_robot->rootLink()->R() = rotInit;
    for(unsigned int i=0; i<m_dof; i++)
      m_robot->joint(i)->q() = m_qInit.data[i];
    m_robot->calcForwardKinematics();


    // calc reference model
    creek::Vector3 posw;  posw << pos[0], pos[1], pos[2];
    creek::Position posr = m_rleg->endLink()->position();
    bool update(false);
    if( m_lleg->calcInverseKinematics( posw, rotInit) ) {
       m_robot->calcForwardKinematics();
       if( m_rleg->calcInverseKinematics(posr.translation(), posr.linear()) )
	 update = true;
    }
    

    if( update ) {
      double qref[m_dof];
      for(unsigned int i=0; i<m_dof; i++)
	qref[i] = m_robot->joint(i)->q();

      creek::Vector3 zmpRefRel( m_robot->rootLink()->R().transpose() * (zmpInitAbs-m_robot->rootLink()->p()) );
      
      m_seq[ANGLES]->calc(m_qInit.data.get_buffer(), &qref[0], tm, creek::CUBIC);
      m_seq[POS]->calc(posInit.data(), pos, tm, creek::CUBIC);
      m_seq[ZMP]->calc(zmpInitRel.data(), zmpRefRel.data(), tm, creek::CUBIC);
    }
    else {
      m_robot->rootLink()->p() = posInit;
      m_robot->rootLink()->R() = rotInit;
      for(unsigned int i=0; i<m_dof; i++)
	m_robot->joint(i)->q() = m_qInit.data[i];
      m_robot->calcForwardKinematics();
    }
    return update;
  }
  else
    return false;
}


bool creekSequencePlayer::setBaseRpy(const double *rpy, double tm)
{
  if( !m_seq[RPY]->empty() ) 
    return false;
  else {
    creek::Vector3 rpyInit;  rpyInit << m_baseRpyInit.data.r, m_baseRpyInit.data.p, m_baseRpyInit.data.y;
    //std::cout << "creekSequencePlayer : init rpy = " << rpyInit[0] << ", " << rpyInit[1] << ", " << rpyInit[2] << std::endl;
    m_seq[RPY]->calc(rpyInit.data(), rpy, tm, creek::CUBIC);
    return true;
  }
}


bool creekSequencePlayer::setZmp(const double *zmp, double tm)
{
  if( !m_seq[ZMP]->empty() ) 
    return false;
  else {
    creek::Vector3 zmpInit;  zmpInit << m_zmpRefInit.data.x, m_zmpRefInit.data.y, m_zmpRefInit.data.z;
    //std::cout << "creekSequencePlayer : init zmp = " << zmpInit[0] << ", " << zmpInit[1] << ", " << zmpInit[2] << std::endl;
    m_seq[ZMP]->calc(zmpInit.data(), zmp, tm, creek::CUBIC);
    return true;
  }
}


bool creekSequencePlayer::isEmpty()
{
  for(int i=0; i<NUM_SEQ; i++)
    if( !m_seq[i]->empty() ) return false;
  return true;
}


void creekSequencePlayer::calcCoM()
{
  for(unsigned int i=0; i<m_dof; i++)
    m_robot->joint(i)->q() = m_qInit.data[i];
  m_robot->calcForwardKinematics();
  
  creek::Vector3 ground((m_robot->joint(5)->p() + m_robot->joint(11)->p()) / 2.0);
  creek::Vector3 com(m_robot->calcCenterOfMass());

  creek::Vector3 com_pos(com-ground);
  
  std::cout << "creekSequencePlayer\n"
	    << "com pos = " << com_pos(0) << ", " << com_pos(1) << ", " << com_pos(2)
	    << "\n"
	    << "ground  = " << ground(0)  << ", " << ground(1)  << ", " << ground(2)
	    << std::endl << std::endl;
}


void creekSequencePlayer::jointCalib(int scale)
{
  if( m_qInit.data.length() != m_dof ) return;


  std::string name;
  double uangle, langle, time;

  //std::cout << "creekSequencePlayer : jointCalib" << std::endl;

  for(int i=0; i<m_robot->numJoints(); i++) {
    name  = m_robot->joint(i)->name();
    /*
    std::cout << name << std::endl;
    std::cout << "    " << m_qInit.data[i] 
	      << ",  " << m_robot->joint(i)->q_upper()
	      << ",  " << m_robot->joint(i)->dq_upper()
	      << ",  " << m_robot->joint(i)->q_lower()
	      << ",  " << m_robot->joint(i)->dq_lower()
	      << std::endl;
    */

    if( std::isnan( m_qInit.data[i] ) ) {
      std::cout << "nan" << std::endl;
      m_qInit.data[i] = 0.0;
    }


    // to upper
    uangle = m_robot->joint(i)->q_upper() * 0.9;
    time   = std::fabs( ( uangle - m_qInit.data[i] ) / m_robot->joint(i)->dq_upper() * scale ) + 0.5;

    //std::cout << "    to upper,    angle = " << uangle << ",    time = " << time << std::endl;

    setJointAngle( name.c_str(), uangle, time );
    waitInterpolation();


    // to lower
    langle = m_robot->joint(i)->q_lower() * 0.9;
    time   = std::fabs( ( langle - uangle ) / m_robot->joint(i)->dq_lower() * scale ) + 0.5;    

    //std::cout << "    to lower,    angle = " << langle << ",    time = " << time << std::endl;

    setJointAngle( name.c_str(), langle, time );
    waitInterpolation();


    // to zero
    time   = std::fabs( ( 0 - langle ) / m_robot->joint(i)->dq_lower() * scale ) + 0.5;

    //std::cout << "    to zero,     angle = " << 0 << ",    time = " << time << std::endl;

    setJointAngle(name.c_str(), 0, std::fabs(time));
    waitInterpolation();
  }
}


bool creekSequencePlayer::setBasePosRel(const double *pos, double tm)
{
  creek::Vector3 posInit;  posInit << m_basePosInit.data.x, m_basePosInit.data.y, m_basePosInit.data.z;
  creek::Matrix3 rotInit( model::rotFromRpy(m_baseRpyInit.data.r, m_baseRpyInit.data.p, m_baseRpyInit.data.y) );
  creek::Vector3 tmp;  tmp << pos[0], pos[1], pos[2];
  creek::Vector3 refPos(posInit + rotInit * tmp );
  return setBasePos(refPos.data(), tm);
}


//----------------------------------------------------------------------------------


extern "C"
{
 
  void creekSequencePlayerInit(RTC::Manager* manager)
  {
    coil::Properties profile(creeksequenceplayer_spec);
    manager->registerFactory(profile,
                             RTC::Create<creekSequencePlayer>,
                             RTC::Delete<creekSequencePlayer>);
  }
  
};



