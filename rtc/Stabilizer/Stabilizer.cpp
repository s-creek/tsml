// -*- C++ -*-
/*!
 * @file  Stabilizer.cpp
 * @brief stabilizer filter
 * $Date$
 *
 * $Id$
 */

#include <rtm/CorbaNaming.h>
//#include <hrpModel/Link.h>
//#include <hrpModel/Sensor.h>
//#include <hrpModel/ModelLoaderUtil.h>
#include "Stabilizer.h"
//#include "util/VectorConvert.h"
#include <math.h>
//std::ofstream ofs("/home/wu/src/HRP3.1x/Stabilizer/st.log");

// Module specification
// <rtc-template block="module_spec">
static const char* stabilizer_spec[] =
  {
    "implementation_id", "Stabilizer",
    "type_name",         "Stabilizer",
    "description",       "stabilizer",
    "version",           "HRPSYS_PACKAGE_VERSION",
    "vendor",            "AIST",
    "category",          "example",
    "activity_type",     "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.debugLevel", "0",
    ""
  };
// </rtc-template>

#define MAX_TRANSITION_COUNT (2/dt)
static double vlimit(double value, double llimit_value, double ulimit_value);
//add by wu
static Vector3 rot2rpy(Matrix3 R);
//static double switching_inpact_absorber(double force, double lower_th, double upper_th);

Stabilizer::Stabilizer(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    // <rtc-template block="initializer">
    m_qCurrentIn("qCurrent", m_qCurrent),
    m_qRefIn("qRef", m_qRef),
    m_rpyIn("rpy", m_rpy),
    m_forceRIn("forceR", m_force[0]),
    m_forceLIn("forceL", m_force[1]),
    m_zmpRefIn("zmpRef", m_zmpRef),
    m_StabilizerServicePort("StabilizerService"),
    m_basePosIn("basePosIn", m_basePos),
    m_baseRpyIn("baseRpyIn", m_baseRpy),
    m_contactStatesIn("contactStates", m_contactStates),
    m_controlSwingSupportTimeIn("controlSwingSupportTime", m_controlSwingSupportTime),
    m_localEEposIn("localEEpos", m_localEEpos),
    m_qRefOut("q", m_qRefToRobot),
    m_tauOut("tau", m_tau),
    m_zmpOut("zmp", m_zmp),
    // for debug output
    m_originRefZmpOut("originRefZmp", m_originRefZmp),
    m_originRefCogOut("originRefCog", m_originRefCog),
    m_originRefCogVelOut("originRefCogVel", m_originRefCogVel),
    m_originNewZmpOut("originNewZmp", m_originNewZmp),
    m_originActZmpOut("originActZmp", m_originActZmp),
    m_originActCogOut("originActCog", m_originActCog),
    m_originActCogVelOut("originActCogVel", m_originActCogVel),
    m_refWrenchROut("refWrenchR", m_refWrenchR),
    m_refWrenchLOut("refWrenchL", m_refWrenchL),
    m_footCompROut("footCompR", m_footCompR),
    m_footCompLOut("footCompL", m_footCompL),
    m_actBaseRpyOut("actBaseRpy", m_actBaseRpy),
    m_currentBasePosOut("currentBasePos", m_currentBasePos),
    m_currentBaseRpyOut("currentBaseRpy", m_currentBaseRpy),
    m_debugDataOut("debugData", m_debugData),
    control_mode(MODE_IDLE),
    st_algorithm(OpenHRP::StabilizerService::EEFM),
    // </rtc-template>
    m_debugLevel(0)
{
  m_service0.stabilizer(this);
}

Stabilizer::~Stabilizer()
{
}

RTC::ReturnCode_t Stabilizer::onInitialize()
{
  std::cerr << m_profile.instance_name << ": onInitialize()" << std::endl;
  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("debugLevel", m_debugLevel, "0");
  
  // </rtc-template>

  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("qCurrent", m_qCurrentIn);
  addInPort("qRef", m_qRefIn);
  addInPort("forceR", m_forceRIn);
  addInPort("forceL", m_forceLIn);
  addInPort("rpy", m_rpyIn);
  addInPort("zmpRef", m_zmpRefIn);
  addInPort("basePosIn", m_basePosIn);
  addInPort("baseRpyIn", m_baseRpyIn);
  addInPort("contactStates", m_contactStatesIn);
  addInPort("controlSwingSupportTime", m_controlSwingSupportTimeIn);
  addInPort("localEEpos",m_localEEposIn);

  // Set OutPort buffer
  addOutPort("q", m_qRefOut);
  addOutPort("tau", m_tauOut);
  addOutPort("zmp", m_zmpOut);
  // for debug output
  addOutPort("originRefZmp", m_originRefZmpOut);
  addOutPort("originRefCog", m_originRefCogOut);
  addOutPort("originRefCogVel", m_originRefCogVelOut);
  addOutPort("originNewZmp", m_originNewZmpOut);
  addOutPort("originActZmp", m_originActZmpOut);
  addOutPort("originActCog", m_originActCogOut);
  addOutPort("originActCogVel", m_originActCogVelOut);
  addOutPort("refWrenchR", m_refWrenchROut);
  addOutPort("refWrenchL", m_refWrenchLOut);
  addOutPort("footCompR", m_footCompROut);
  addOutPort("footCompL", m_footCompLOut);
  addOutPort("actBaseRpy", m_actBaseRpyOut);
  addOutPort("currentBasePos", m_currentBasePosOut);
  addOutPort("currentBaseRpy", m_currentBaseRpyOut);
  addOutPort("debugData", m_debugDataOut);
  
  // Set service provider to Ports
  m_StabilizerServicePort.registerProvider("service0", "StabilizerService", m_service0);
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  addPort(m_StabilizerServicePort);
  
  // </rtc-template>
  RTC::Properties& prop = getProperties();
  coil::stringTo(dt, prop["Stabilizer.dt"].c_str());

  end_link[RLEG]=prop["RLEG_END"];
  end_link[LLEG]=prop["LLEG_END"];
  end_link[RARM]=prop["RARM_END"];
  end_link[LARM]=prop["LARM_END"];
  end_link[WAIST]=prop["BASE_LINK"];

  // parameters for corba
  /*
  RTC::Manager& rtcManager = RTC::Manager::instance();
  std::string nameServer = rtcManager.getConfig()["corba.nameservers"];
  int comPos = nameServer.find(",");
  if (comPos < 0){
    comPos = nameServer.length();
  }
  nameServer = nameServer.substr(0, comPos);
  RTC::CorbaNaming naming(rtcManager.getORB(), nameServer.c_str());

  // parameters for internal robot model
  m_robot = hrp::BodyPtr(new hrp::Body());
  if (!loadBodyFromModelLoader(m_robot, prop["model"].c_str(), 
                               CosNaming::NamingContext::_duplicate(naming.getRootContext())
                               )){
    std::cerr << "[" << m_profile.instance_name << "]failed to load model[" << prop["model"] << "]" << std::endl;
    return RTC::RTC_ERROR;
  }
  */
  cnoid::BodyLoader bl;
  m_robot=bl.load( prop["model"].c_str());
  std::cout<<"ST dof robot "<<m_robot->numJoints()<<std::endl;
  coil::stringTo(m_waist_height, prop["waist_height"].c_str());
  //std::cout<<"sony waist_height "<<m_waist_height<<std::endl;
  //m_robot->rootLink()->p()<<0.0, 0.0, m_waist_height;

  //std::cout<<"R "<<m_robot->rootLink()->name()<<std::endl;
  m_robot->calcForwardKinematics();
  m_robot->calcCenterOfMass();
  //std::cout<<"sony robot mass "<<m_robot->mass()<<endl;
  forceSensors = m_robot->devices();
  RateGyroSensors= m_robot->devices();
  // setting from conf file
  // rleg,TARGET_LINK,BASE_LINK,x,y,z,rx,ry,rz,rth #<=pos + rot (axis+angle)
  coil::vstring end_effectors_str = coil::split(prop["end_effectors"], ",");
  if (end_effectors_str.size() > 0) {
    size_t prop_num = 10;
    size_t num = end_effectors_str.size()/prop_num;
    for (size_t i = 0; i < num; i++) {
      std::string ee_name, ee_target, ee_base;
      coil::stringTo(ee_name, end_effectors_str[i*prop_num].c_str());
      coil::stringTo(ee_target, end_effectors_str[i*prop_num+1].c_str());
      coil::stringTo(ee_base, end_effectors_str[i*prop_num+2].c_str());
      ee_trans eet;
      for (size_t j = 0; j < 3; j++) {
        coil::stringTo(eet.localp(j), end_effectors_str[i*prop_num+3+j].c_str());
      }
      double tmpv[4];
      for (int j = 0; j < 4; j++ ) {
        coil::stringTo(tmpv[j], end_effectors_str[i*prop_num+6+j].c_str());
      }
      eet.localR = Eigen::AngleAxis<double>(tmpv[3], Vector3(tmpv[0], tmpv[1], tmpv[2])).toRotationMatrix(); // rotation in VRML is represented by axis + angle
      // manip2[i] = hrp::JointPathExPtr(new hrp::JointPathEx(m_robot, m_robot->link(ee_base),
      //                                                      m_robot->link(ee_target)));
      //ee_map.insert(std::pair<std::string, ee_trans>(ee_name , eet));
      ee_map.insert(std::pair<std::string, ee_trans>(ee_target , eet));
      contact_states_index_map.insert(std::pair<std::string, size_t>(ee_name, i));
    }
    m_contactStates.data.length(num);
  }
 

  // parameters for TPCC
  act_zmp = Vector3::Zero();
  for (int i = 0; i < ST_NUM_LEGS; i++) {
    k_tpcc_p[i] = 0.2;
    k_tpcc_x[i] = 4.0;
    k_brot_p[i] = 0.1;
    k_brot_tc[i] = 1.5;
  }
  // parameters for EEFM///////////////////////////
  /*  
  double k_ratio = 0.9;
  for (int i = 0; i < 2; i++) {
    //eefm_k1[i] = -1.41429*k_ratio;
    //eefm_k2[i] = -0.404082*k_ratio;
    //eefm_k3[i] = -0.18*k_ratio;

    //calc by wu
    //when Tc=0.04
    eefm_k1[i] =-1.2310124;
    eefm_k2[i] =-0.3577664;
    eefm_k3[i] =-0.2223669;
    // jvrc

    //org
    //eefm_body_attitude_control_gain[i] = 1.0;
    //eefm_body_attitude_control_time_const[i] = 1e5;
  }
  
  //org gain
  //eefm_rot_time_const = 1;
  //eefm_pos_time_const_support = 1;
  eefm_pos_time_const_swing = 0.04;
  eefm_pos_transition_time = 0.02;//unuse
  eefm_pos_margin_time = 0.02;//unuse
  eefm_zmp_delay_time_const[0] = eefm_zmp_delay_time_const[1] = 0.04;//org 0.04

  //wutest gain
  eefm_pos_damping_gain = 2*1e5;
  eefm_rot_damping_gain[0] = 1000; 
  eefm_rot_damping_gain[1] = 750; 
  eefm_pos_time_const_support = 0.1; 
  //eefm_pos_time_const_swing = 0.1;
  eefm_rot_time_const = 0.1;
  for(int i=0;i<2;i++){
    eefm_body_attitude_control_gain[i] = 0.1;
    //eefm_body_attitude_control_gain[i] = 1.3;
    eefm_body_attitude_control_time_const[i] = 0.003;
  }
  
  eefm_cogvel_cutoff_freq = 35.3678; //[Hz]
  //for hrp2
  eefm_leg_inside_margin = 0.055; // [m]
  eefm_leg_front_margin = 0.13;
  eefm_leg_rear_margin = 0.1;
  ////////////////////////////////////////////////////////////////
  */
  eefm_pos_control_switch=eefm_rot_control_switch=eefm_att_control_switch=1;
  eefm_pos_control_switch=eefm_att_control_switch=1;

  //ogawa
  setEefmParameters();

  // parameters for RUNST
  {
    double ke = 0, tc = 0;
    for (int i = 0; i < ST_NUM_LEGS; i++) {
      //m_tau_x[i].setup(ke, tc, dt);
      //m_tau_y[i].setup(ke, tc, dt);
      //m_f_z.setup(ke, tc, dt);
    }
    pangx_ref = pangy_ref = pangx = pangy = 0;
    rdx = rdy = rx = ry = 0;
    pdr = Vector3::Zero();
    prev_act_force_z[0] = prev_act_force_z[1] = 0.0;
    //unused by choreonoid lib
    sensor_names.push_back("rfsensor");
    sensor_names.push_back("lfsensor");
  }

  is_legged_robot = false;
  for (size_t i = 0; i < 2; i++) {
    //if ( m_robot->sensor<hrp::ForceSensor>(sensor_names[i]) != NULL) {
    // manip2[i] = hrp::JointPathExPtr(new hrp::JointPathEx(m_robot, m_robot->rootLink(), m_robot->sensor<hrp::ForceSensor>(sensor_names[i])->link));
    // is_legged_robot = true;
    //}
    manip2[i] = hrp::JointPathExPtr(new hrp::JointPathEx(m_robot, m_robot->rootLink(), forceSensors[i]->link() ));
    is_legged_robot = true;

  }

  m_qCurrent.data.length(m_robot->numJoints());
  m_qRef.data.length(m_robot->numJoints());
  m_qRefToRobot.data.length(m_robot->numJoints());
  m_tau.data.length(m_robot->numJoints());
  m_force[0].data.length(6);
  m_force[1].data.length(6);
  transition_joint_q.resize(m_robot->numJoints());
  qorg.resize(m_robot->numJoints());
  qrefv.resize(m_robot->numJoints());
  transition_count = 0;
  loop = 0;
  if (is_legged_robot) {
    //zmp_origin_off = ee_map[m_robot->sensor<hrp::ForceSensor>(sensor_names[0])->link->name()].localp(2);
    //no needed. calculated in getTarget
    //zmp_origin_off = ee_map["RLEG_JOINT5"].localp(2); 
    zmp_origin_off = ee_map[ end_effectors_str[1] ].localp(2);
  }
  //total_mass = m_robot->totalMass();
  total_mass = m_robot->mass();
  ref_zmp_aux = Vector3::Zero();
  //for right and left foot
  for (size_t i = 0; i < m_contactStates.data.length(); i++) {
    contact_states.push_back(true);
    prev_contact_states.push_back(true);
  }

  for ( int i = 0; i < m_robot->numJoints(); i++ )
    m_qRef.data[i] = m_qRefToRobot.data[i] = 0.0;


  // for debug output
  m_originRefZmp.data.x = m_originRefZmp.data.y = m_originRefZmp.data.z = 0.0;
  m_originRefCog.data.x = m_originRefCog.data.y = m_originRefCog.data.z = 0.0;
  m_originRefCogVel.data.x = m_originRefCogVel.data.y = m_originRefCogVel.data.z = 0.0;
  m_originNewZmp.data.x = m_originNewZmp.data.y = m_originNewZmp.data.z = 0.0;
  m_originActZmp.data.x = m_originActZmp.data.y = m_originActZmp.data.z = 0.0;
  m_originActCog.data.x = m_originActCog.data.y = m_originActCog.data.z = 0.0;
  m_originActCogVel.data.x = m_originActCogVel.data.y = m_originActCogVel.data.z = 0.0;
  m_refWrenchR.data.length(6); m_refWrenchL.data.length(6);
  m_refWrenchR.data[0] = m_refWrenchR.data[1] = m_refWrenchR.data[2] = m_refWrenchR.data[3] = m_refWrenchR.data[4] = m_refWrenchR.data[5] = 0.0;
  m_refWrenchL.data[0] = m_refWrenchL.data[1] = m_refWrenchL.data[2] = m_refWrenchL.data[3] = m_refWrenchL.data[4] = m_refWrenchL.data[5] = 0.0;
  m_footCompR.data.length(6); m_footCompL.data.length(6);
  m_footCompR.data[0] = m_footCompR.data[1] = m_footCompR.data[2] = m_footCompR.data[3] = m_footCompR.data[4] = m_footCompR.data[5] = 0.0;
  m_footCompL.data[0] = m_footCompL.data[1] = m_footCompL.data[2] = m_footCompL.data[3] = m_footCompL.data[4] = m_footCompL.data[5] = 0.0;
  m_debugData.data.length(1); m_debugData.data[0] = 0.0;


  //wu
  step_counter=0;
  coil::stringTo(m_nStep, prop["wutest.nStep"].c_str());
  //force_dz_offset=40.824;
   coil::stringTo(force_dz_offset, prop["force_dz_offset"].c_str());
   //force_dz_offset=40.7735;

  return RTC::RTC_OK;
}


/*
RTC::ReturnCode_t Stabilizer::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Stabilizer::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Stabilizer::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t Stabilizer::onActivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name<< ": onActivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t Stabilizer::onDeactivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name<< ": onDeactivated(" << ec_id << ")" << std::endl;
  if ( (control_mode == MODE_ST || control_mode == MODE_AIR) ) {
    sync_2_idle ();
    control_mode = MODE_IDLE;
    transition_count = 1; // sync in one controller loop
  }
  return RTC::RTC_OK;
}

#define DEBUGP ((m_debugLevel==1 && loop%200==0) || m_debugLevel > 1 )
#define DEBUGP2 (loop%10==0)
RTC::ReturnCode_t Stabilizer::onExecute(RTC::UniqueId ec_id)
{
  /*
  step_counter+=1;
  step_counter=step_counter%m_nStep;
  if(step_counter!=0)
    return RTC::RTC_OK;
  */

  loop++;
  //std::cout<< transition_smooth_gain<<std::endl;
  // std::cout << m_profile.instance_name<< ": onExecute(" << ec_id << ")" << std::endl;

  if (m_qRefIn.isNew()) {
    m_qRefIn.read();
  }
  if (m_qCurrentIn.isNew()) {
    m_qCurrentIn.read();
  }
  if (m_rpyIn.isNew()) {
    m_rpyIn.read();
  }
  if (m_forceRIn.isNew()) {
    m_forceRIn.read();
  }
  if (m_forceLIn.isNew()) {
    m_forceLIn.read();
  }
  if (m_zmpRefIn.isNew()) {
    m_zmpRefIn.read();
  }
  if (m_basePosIn.isNew()){
    m_basePosIn.read();
  }
  if (m_baseRpyIn.isNew()){
    m_baseRpyIn.read();
  }
  if (m_contactStatesIn.isNew()){
    m_contactStatesIn.read();
    for (size_t i = 0; i < m_contactStates.data.length(); i++) {
      contact_states[i] = m_contactStates.data[i];
      ///wutest///
      ForceSensor* sensor=forceSensors[i];
      Vector3 sensor_force = (sensor->link()->R() * sensor->R_local()) * Vector3(m_force[i].data[0], m_force[i].data[1], m_force[i].data[2]);
      if(m_contactStates.data[i]==0&&sensor_force[2]>100)
	contact_states[i]=1;
      ///////////
    }
  }
  if (m_controlSwingSupportTimeIn.isNew()){
    m_controlSwingSupportTimeIn.read();
  }

  //ofs<<m_force[0].data[2]<<" "<<m_force[1].data[2]<<endl;
  

  /*
  if(m_localEEposIn.isNew()){
    m_localEEposIn.read();
    for(int i=0;i<3;i++){
      ee_map["RLEG_JOINT5"].localp(i)= m_localEEpos.data[i];
      ee_map["LLEG_JOINT5"].localp(i)= m_localEEpos.data[i+3];
    } 
  }
  */
  if (is_legged_robot) {
    getCurrentParameters();
    getTargetParameters();
    getActualParameters();
    switch (control_mode) {
    case MODE_IDLE:
      //ofs<<m_rpy.data.r<<" "<<m_rpy.data.p<<" "<<m_rpy.data.y<<std::endl;
      break;
    case MODE_AIR:
      if ( transition_count == 0 && on_ground ) sync_2_st();
      break;
    case MODE_ST:
      if (st_algorithm == OpenHRP::StabilizerService::EEFM) {
        calcEEForceMomentControl();
      } else {
        //calcTPCC();
      }
      if ( transition_count == 0 && !on_ground ) control_mode = MODE_SYNC_TO_AIR;
      break;
    case MODE_SYNC_TO_IDLE:
      sync_2_idle();
      control_mode = MODE_IDLE;
      break;
    case MODE_SYNC_TO_AIR:
      sync_2_idle();
      control_mode = MODE_AIR;
      break;
    }
  }
  if ( m_robot->numJoints() == m_qRef.data.length() ) {
    if (is_legged_robot) {

      for ( int i = 0; i < m_robot->numJoints(); i++ ){
        m_qRefToRobot.data[i] = m_robot->joint(i)->q();
       }

      m_zmp.data.x = rel_act_zmp(0);
      m_zmp.data.y = rel_act_zmp(1);
      m_zmp.data.z = rel_act_zmp(2);
      m_zmpOut.write();
      //m_tauOut.write();
      // for debug output
      m_originRefZmp.data.x = ref_zmp(0); m_originRefZmp.data.y = ref_zmp(1); m_originRefZmp.data.z = ref_zmp(2);
      m_originRefCog.data.x = ref_cog(0); m_originRefCog.data.y = ref_cog(1); m_originRefCog.data.z = ref_cog(2);
      m_originRefCogVel.data.x = ref_cogvel(0); m_originRefCogVel.data.y = ref_cogvel(1); m_originRefCogVel.data.z = ref_cogvel(2);
      m_originNewZmp.data.x = new_refzmp(0); m_originNewZmp.data.y = new_refzmp(1); m_originNewZmp.data.z = new_refzmp(2);
      m_originActZmp.data.x = act_zmp(0); m_originActZmp.data.y = act_zmp(1); m_originActZmp.data.z = act_zmp(2);
      m_originActCog.data.x = act_cog(0); m_originActCog.data.y = act_cog(1); m_originActCog.data.z = act_cog(2);
      m_originActCogVel.data.x = act_cogvel(0); m_originActCogVel.data.y = act_cogvel(1); m_originActCogVel.data.z = act_cogvel(2);
      m_refWrenchR.data[0] = ref_foot_force[0](0); m_refWrenchR.data[1] = ref_foot_force[0](1); m_refWrenchR.data[2] = ref_foot_force[0](2);
      m_refWrenchR.data[3] = ref_foot_moment[0](0); m_refWrenchR.data[4] = ref_foot_moment[0](1); m_refWrenchR.data[5] = ref_foot_moment[0](2);
      m_refWrenchL.data[0] = ref_foot_force[1](0); m_refWrenchL.data[1] = ref_foot_force[1](1); m_refWrenchL.data[2] = ref_foot_force[1](2);
      m_refWrenchL.data[3] = ref_foot_moment[1](0); m_refWrenchL.data[4] = ref_foot_moment[1](1); m_refWrenchL.data[5] = ref_foot_moment[1](2);
      m_footCompR.data[2] = f_zctrl[0]; m_footCompL.data[2] = f_zctrl[1];
      m_footCompR.data[3] = d_foot_rpy[0](0); m_footCompR.data[4] = d_foot_rpy[0](1);
      m_footCompL.data[3] = d_foot_rpy[1](0); m_footCompL.data[4] = d_foot_rpy[1](1);
      m_originRefZmpOut.write();
      m_originRefCogOut.write();
      m_originRefCogVelOut.write();
      m_originNewZmpOut.write();
      m_originActZmpOut.write();
      m_originActCogOut.write();
      m_originActCogVelOut.write();
      m_refWrenchROut.write(); m_refWrenchLOut.write();
      m_footCompROut.write(); m_footCompLOut.write();
      m_actBaseRpy.data.r = act_base_rpy(0);
      m_actBaseRpy.data.p = act_base_rpy(1);
      m_actBaseRpy.data.y = act_base_rpy(2);
      m_currentBaseRpy.data.r = current_base_rpy(0);
      m_currentBaseRpy.data.p = current_base_rpy(1);
      m_currentBaseRpy.data.y = current_base_rpy(2);
      m_currentBasePos.data.x = current_base_pos(0);
      m_currentBasePos.data.y = current_base_pos(1);
      m_currentBasePos.data.z = current_base_pos(2);
      m_actBaseRpyOut.write();
      m_currentBaseRpyOut.write();
      m_currentBasePosOut.write();
      m_debugDataOut.write();
    }
    m_qRefOut.write();
  }

  return RTC::RTC_OK;
}

void Stabilizer::getCurrentParameters ()
{
  current_root_p = m_robot->rootLink()->p();
  current_root_R = m_robot->rootLink()->R();
  for ( int i = 0; i < m_robot->numJoints(); i++ ){
    qorg[i] = m_robot->joint(i)->q();
  }
}

void Stabilizer::calcFootOriginCoords (Vector3& foot_origin_pos, Matrix3& foot_origin_rot)
{
  rats::coordinates leg_c[2], tmpc;
  //Vector3 ez = Vector3::UnitZ();
  //Vector3 ex = Vector3::UnitX();
  
  for (size_t i = 0; i < 2; i++) {
    //Link* target =m_robot->sensor<hrp::ForceSensor>(sensor_names[i])->link;
    Link* target = forceSensors[i]->link();
    //org
    /*
    leg_c[i].pos = target->p();
    Vector3 xv1(target->R() * ex);
    xv1(2)=0.0;
    xv1.normalize();
    Vector3 yv1(ez.cross(xv1));
    leg_c[i].rot(0,0) = xv1(0); leg_c[i].rot(1,0) = xv1(1); leg_c[i].rot(2,0) = xv1(2);
    leg_c[i].rot(0,1) = yv1(0); leg_c[i].rot(1,1) = yv1(1); leg_c[i].rot(2,1) = yv1(2);
    leg_c[i].rot(0,2) = ez(0); leg_c[i].rot(1,2) = ez(1); leg_c[i].rot(2,2) = ez(2);
    */
    //wutest //maybe extractYow is needed
    Vector3 euler =target->R().eulerAngles(2,1,0);
    leg_c[i].pos = target->p() + target->R() * ee_map[target->name()].localp;
    leg_c[i].rot=rotFromRpy(0.0, 0.0, euler(0));

    }
  if (contact_states[contact_states_index_map["rleg"]] &&
      contact_states[contact_states_index_map["lleg"]]) {
    rats::mid_coords(tmpc, 0.5, leg_c[0], leg_c[1]);
    foot_origin_pos = tmpc.pos;
    foot_origin_rot = tmpc.rot;
  } else if (contact_states[contact_states_index_map["rleg"]]) {
    foot_origin_pos = leg_c[0].pos;
    foot_origin_rot = leg_c[0].rot;
  } else {
    foot_origin_pos = leg_c[1].pos;
    foot_origin_rot = leg_c[1].rot;
  }
}

void Stabilizer::getActualParameters ()
{
  // Actual world frame =>
  Vector3 foot_origin_pos;
  Matrix3 foot_origin_rot;
  if (st_algorithm == OpenHRP::StabilizerService::EEFM) {
    // update by current joint angles
    for ( int i = 0; i < m_robot->numJoints(); i++ ){
      m_robot->joint(i)->q() = m_qCurrent.data[i];
    }
    // tempolary
    m_robot->rootLink()->p() = Vector3::Zero();
    m_robot->calcForwardKinematics();
    //hrp::Sensor* sen = m_robot->sensor<hrp::RateGyroSensor>("gyrometer");
    RateGyroSensor* sen=RateGyroSensors[0];
    //Matrix3 senR = sen->link->R() * sen->localR;
    Matrix3 senR = sen->link()->R() * sen->R_local();
    Matrix3 act_Rs(rotFromRpy(m_rpy.data.r, m_rpy.data.p, m_rpy.data.y));
    //Matrix3 act_Rs(hrp::rotFromRpy(m_rpy.data.r*0.5, m_rpy.data.p*0.5, m_rpy.data.y*0.5));
    //org
    //m_robot->rootLink()->R() = act_Rs * (senR.transpose() * m_robot->rootLink()->R());
    //wutest
    Matrix3 R_root2chest=m_robot->rootLink()->R().transpose() *  sen->link()->R();
    m_robot->rootLink()->R() = act_Rs * sen->R_local().transpose() * R_root2chest;
    
    m_robot->calcForwardKinematics();
    act_base_rpy = rot2rpy(m_robot->rootLink()->R());//rpyFromRot(m_robot->rootLink()->R());
    calcFootOriginCoords (foot_origin_pos, foot_origin_rot);
  } else {
    for ( int i = 0; i < m_robot->numJoints(); i++ ) {
      m_robot->joint(i)->q() = qorg[i];
    }
    m_robot->rootLink()->p() = current_root_p;
    m_robot->rootLink()->R() = current_root_R;
    m_robot->calcForwardKinematics();
  }
  // cog
  act_cog = m_robot->calcCenterOfMass();
  // zmp
  on_ground = false;
  if (st_algorithm == OpenHRP::StabilizerService::EEFM) {
    on_ground = calcZMP(act_zmp, zmp_origin_off+foot_origin_pos(2));
  } else {
    on_ground = calcZMP(act_zmp, ref_zmp(2));
  }
  // <= Actual world frame

  // convert absolute (in st) -> root-link relative 
  //unused
  rel_act_zmp = m_robot->rootLink()->R().transpose() * (act_zmp - m_robot->rootLink()->p());
  
  if (st_algorithm == OpenHRP::StabilizerService::EEFM) {
    // Actual foot_origin frame =>
    act_zmp = foot_origin_rot.transpose() * (act_zmp - foot_origin_pos);
    act_cog = foot_origin_rot.transpose() * (act_cog - foot_origin_pos);
    //act_cogvel = foot_origin_rot.transpose() * act_cogvel;
    if (contact_states != prev_contact_states) {
      act_cogvel = (foot_origin_rot.transpose() * prev_act_foot_origin_rot) * act_cogvel;
    } else {
      act_cogvel = (act_cog - prev_act_cog)/dt;
    }
    prev_act_foot_origin_rot = foot_origin_rot;
    double const_param = 2 * M_PI * eefm_cogvel_cutoff_freq * dt;
    act_cogvel = 1.0/(1+const_param) * prev_act_cogvel + const_param/(1+const_param) * act_cogvel;
    prev_act_cog = act_cog;
    prev_act_cogvel = act_cogvel;
    //act_root_rot = m_robot->rootLink()->R();
    // <= Actual foot_origin frame

    // Actual world frame =>
    // new ZMP calculation
    // Kajita's feedback law
    Vector3 dcog=foot_origin_rot * (ref_cog - act_cog);
    Vector3 dcogvel=foot_origin_rot * (ref_cogvel - act_cogvel);
    Vector3 dzmp=foot_origin_rot * (ref_zmp - act_zmp);
    
    //this zmp is in ref_foot_origin 
    new_refzmp = foot_origin_rot * new_refzmp + foot_origin_pos;
    //world frame
    for (size_t i = 0; i < 2; i++) {
      new_refzmp(i) += eefm_k1[i] * transition_smooth_gain * dcog(i) + eefm_k2[i] * transition_smooth_gain * dcogvel(i) + eefm_k3[i] * transition_smooth_gain * dzmp(i) + ref_zmp_aux(i);
    }
    if (DEBUGP) {
      std::cerr << "[" << m_profile.instance_name << "] state values" << std::endl;
      std::cerr << "[" << m_profile.instance_name << "]   "
                << "ref_cog    = " << Vector3(ref_cog*1e3).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]"))
                << ", act_cog    = " << Vector3(act_cog*1e3).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[mm]" << std::endl;
      std::cerr << "[" << m_profile.instance_name << "]   "
                << "ref_cogvel = " << Vector3(ref_cogvel*1e3).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]"))
                << ", act_cogvel = " << Vector3(act_cogvel*1e3).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[mm/s]" << std::endl;
      std::cerr << "[" << m_profile.instance_name << "]   "
                << "ref_zmp    = " << Vector3(ref_zmp*1e3).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]"))
                << ", act_zmp    = " << Vector3(act_zmp*1e3).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[mm]" << std::endl;
      std::cerr << "[" << m_profile.instance_name << "]   "
                << "ref_zmp    = " << Vector3((new_refzmp - ref_zmp)*1e3).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[mm]" << std::endl;
    }

    // distribute new ZMP into foot force & moment
    {
      Vector3 tau_0 = Vector3::Zero();
      std::vector<Vector3> ee_pos;
      std::vector<Matrix3> ee_rot;
      for (size_t i = 0; i < 2; i++) {
        //hrp::Link* target = m_robot->sensor<hrp::ForceSensor>(sensor_names[i])->link;
	Link* target=forceSensors[i]->link();
        ee_pos.push_back(target->p() + target->R() * ee_map[target->name()].localp);
        ee_rot.push_back(target->R() * ee_map[target->name()].localR);
      }
      //double fz_alpha =  calcAlpha(Vector3(foot_origin_rot * ref_zmp + foot_origin_pos), ee_pos, ee_rot);
      double alpha = calcAlpha(new_refzmp, ee_pos, ee_rot);
      ref_foot_force[0] = Vector3(0,0, alpha * 9.8 * total_mass);
      ref_foot_force[1] = Vector3(0,0, (1-alpha) * 9.8 * total_mass);

      //wutest
      //ref_foot_force[0](2)+=force_dz_offset/2;
      //ref_foot_force[1](2)-=force_dz_offset/2;
      


      for (size_t i = 0; i < 2; i++) {
        tau_0 -= (ee_pos[i] - new_refzmp).cross(ref_foot_force[i]);//
      }
      if ( alpha == 0.0 ) { // lleg support
        ref_foot_moment[0] = Vector3::Zero();
        ref_foot_moment[1] =  (ee_pos[1] - new_refzmp).cross(ref_foot_force[1]);
      } else if ( alpha == 1.0 ) { // rleg support
	ref_foot_moment[0] =  (ee_pos[0] - new_refzmp).cross(ref_foot_force[0]);
        ref_foot_moment[1] = Vector3::Zero();
      } else { // double support
        // Foot-distribution-coords frame =>
        Vector3 foot_dist_coords_y = (ee_pos[1] - ee_pos[0]); // e_y'
        foot_dist_coords_y(2) = 0.0;
        foot_dist_coords_y.normalize();
        Vector3 foot_dist_coords_x = Vector3(foot_dist_coords_y.cross(Vector3::UnitZ())); // e_x'
        Matrix3 foot_dist_coords_rot;
        foot_dist_coords_rot(0,0) = foot_dist_coords_x(0);
        foot_dist_coords_rot(1,0) = foot_dist_coords_x(1);
        foot_dist_coords_rot(2,0) = foot_dist_coords_x(2);
        foot_dist_coords_rot(0,1) = foot_dist_coords_y(0);
        foot_dist_coords_rot(1,1) = foot_dist_coords_y(1);
        foot_dist_coords_rot(2,1) = foot_dist_coords_y(2);
        foot_dist_coords_rot(0,2) = 0;
        foot_dist_coords_rot(1,2) = 0;
        foot_dist_coords_rot(2,2) = 1;
        Vector3 tau_0_f = foot_dist_coords_rot.transpose() * tau_0; // tau_0'
        // x
        // right
        if (tau_0_f(0) < 0) ref_foot_moment[0](0) = tau_0_f(0);
        else ref_foot_moment[0](0) = 0;
        // left
        if (tau_0_f(0) < 0) ref_foot_moment[1](0) = 0;
        else ref_foot_moment[1](0) = tau_0_f(0);
        // ref_foot_moment[0](0) = tau_0_f(0) * alpha;
        // ref_foot_moment[1](0) = tau_0_f(0) * (1-alpha);
        // y
        ref_foot_moment[0](1) =     alpha* tau_0_f(1);
        ref_foot_moment[1](1) = (1-alpha)* tau_0_f(1);
        ref_foot_moment[0](2) = ref_foot_moment[1](2) = 0.0;
        // <= Foot-distribution-coords frame
        // Convert foot-distribution-coords frame => actual world frame
        ref_foot_moment[0] = foot_dist_coords_rot * ref_foot_moment[0];
        ref_foot_moment[1] = foot_dist_coords_rot * ref_foot_moment[1];
      }
      if (DEBUGP) {
        std::cerr << "[" << m_profile.instance_name << "] force moment distribution" << std::endl;
        std::cerr << "[" << m_profile.instance_name << "]   alpha = " << alpha << "" << std::endl;
        std::cerr << "[" << m_profile.instance_name << "]   "
                  << "total_tau    = " << Vector3(tau_0).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[Nm]" << std::endl;
        std::cerr << "[" << m_profile.instance_name << "]   "
                  << "ref_force_R  = " << Vector3(ref_foot_force[0]).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[N]" << std::endl;
        std::cerr << "[" << m_profile.instance_name << "]   "
                  << "ref_force_L  = " << Vector3(ref_foot_force[1]).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[N]" << std::endl;
        std::cerr << "[" << m_profile.instance_name << "]   "
                  << "ref_moment_R = " << Vector3(ref_foot_moment[0]).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[Nm]" << std::endl;
        std::cerr << "[" << m_profile.instance_name << "]   "
                  << "ref_moment_L = " << Vector3(ref_foot_moment[1]).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[Nm]" << std::endl;
      }
      // for debug output
      new_refzmp = foot_origin_rot.transpose() * (new_refzmp - foot_origin_pos);
    }
    //rpy control
    {
#define deg2rad(x) ((x) * M_PI / 180.0)
      Vector3 act_root_rpy = rot2rpy(m_robot->rootLink()->R());
                            //rpyFromRot(m_robot->rootLink()->R());
      Vector3 ref_root_rpy = rot2rpy(target_root_R);//rpyFromRot(target_root_R);
      for (size_t i = 0; i < 2; i++) {
        //d_rpy[i] = transition_smooth_gain * (eefm_body_attitude_control_gain[i] * (ref_root_rpy(i) - act_root_rpy(i)) - 1/eefm_body_attitude_control_time_const[i] * d_rpy[i]) * dt + d_rpy[i];
	d_rpy[i]=calcDampingControl(ref_root_rpy(i), act_root_rpy(i),  d_rpy[i], eefm_body_attitude_control_gain[i], eefm_body_attitude_control_time_const[i]);
	d_rpy[i] = vlimit(d_rpy[i], deg2rad(-10.0), deg2rad(10.0));
	//wutest
	if(!eefm_att_control_switch)
	  d_rpy[i]=0;
      }
    }

    // foot modif
    //sensor->p_local() == ee_map[sensor->link()->name()].localp in simulation
    {
      double fz_diff = 0;
      double fz[2];//i dont use this
      // moment control
#define deg2rad(x) ((x) * M_PI / 180.0)
      for (size_t i = 0; i < 2; i++) {
        //hrp::Sensor* sensor = m_robot->sensor<hrp::ForceSensor>(sensor_names[i]);
        ForceSensor* sensor=forceSensors[i];
	// Actual world frame =>
        Vector3 sensor_force = (sensor->link()->R() * sensor->R_local()) * Vector3(m_force[i].data[0], m_force[i].data[1], m_force[i].data[2]);
        Vector3 sensor_moment = (sensor->link()->R() * sensor->R_local()) * Vector3(m_force[i].data[3], m_force[i].data[4], m_force[i].data[5]);
        Vector3 ee_moment = (sensor->link()->R() * (sensor->p_local() - ee_map[sensor->link()->name()].localp)).cross(sensor_force) + sensor_moment;
        // <= Actual world frame
        fz_diff += (i==0? -sensor_force(2) : sensor_force(2));
        fz[i] = sensor_force(2);
        // calcDampingControl
        d_foot_rpy[i](0) = calcDampingControl(ref_foot_moment[i](0), ee_moment(0), d_foot_rpy[i](0), eefm_rot_damping_gain[i], eefm_rot_time_const);//roll
        d_foot_rpy[i](1) = calcDampingControl(ref_foot_moment[i](1), ee_moment(1), d_foot_rpy[i](1), eefm_rot_damping_gain[i], eefm_rot_time_const);//pitch
        d_foot_rpy[i](0) = vlimit(d_foot_rpy[i](0), deg2rad(-10.0), deg2rad(10.0));
        d_foot_rpy[i](1) = vlimit(d_foot_rpy[i](1), deg2rad(-10.0), deg2rad(10.0));
        // Actual ee frame =>
        ee_d_foot_rpy[i] = (sensor->link()->R() * ee_map[sensor->link()->name()].localR).transpose() * d_foot_rpy[i];// desire torque and delta rpy is in inverse direction

	//wutest
	if(!eefm_rot_control_switch)
	  ee_d_foot_rpy[i] =Vector3::Zero();
      }

      // Convert actual world frame => actual foot_origin frame for debug data port.(unuse)
      ref_foot_moment[0] = foot_origin_rot.transpose() * ref_foot_moment[0];
      ref_foot_moment[1] = foot_origin_rot.transpose() * ref_foot_moment[1];

      // fz control
      // foot force difference control version
      double ref_fz_diff = (ref_foot_force[1](2)-ref_foot_force[0](2));

      //wutest because the force  z in two legs in Inipos is different
      //fz_diff+=force_dz_offset;

      if ( (contact_states[contact_states_index_map["rleg"]] && contact_states[contact_states_index_map["lleg"]]) // Reference : double support phase
           || (isContact(0) && isContact(1)) ) { // Actual : double support phase
        zctrl = calcDampingControl (ref_fz_diff, fz_diff, zctrl,
                                    eefm_pos_damping_gain, eefm_pos_time_const_support);
	//wutest
	if(!eefm_pos_control_switch)
	  zctrl=0;
      } 
      else {
	/*
	  //commentout by wu
        double remain_swing_time;
        if ( !contact_states[contact_states_index_map["rleg"]] ) { // rleg swing
          remain_swing_time = m_controlSwingSupportTime.data[contact_states_index_map["rleg"]];
        } else { // lleg swing
          remain_swing_time = m_controlSwingSupportTime.data[contact_states_index_map["lleg"]];
        }
        // std::cerr << "st " << remain_swing_time << " rleg " << contact_states[contact_states_index_map["rleg"]] << " lleg " << contact_states[contact_states_index_map["lleg"]] << std::endl;
        if (eefm_pos_transition_time+eefm_pos_margin_time<remain_swing_time) {
          zctrl = calcDampingControl (0, 0, zctrl,
                                      eefm_pos_damping_gain, eefm_pos_time_const_swing);
        } else {
          double tmp_ratio = std::min(1.0, 1.0 - (remain_swing_time-eefm_pos_margin_time)/eefm_pos_transition_time); // 0=>1
          zctrl = calcDampingControl (tmp_ratio * ref_fz_diff, tmp_ratio * fz_diff, zctrl,
                                      eefm_pos_damping_gain, ((1-tmp_ratio)*eefm_pos_time_const_swing+tmp_ratio*eefm_pos_time_const_support));
        }
	*/
	//add by wu
	  zctrl = calcDampingControl (0, 0, zctrl,
                                      eefm_pos_damping_gain, eefm_pos_time_const_swing);
      }
      // zctrl = vlimit(zctrl, -0.02, 0.02);
      zctrl = vlimit(zctrl, -0.05, 0.05);
      //wutest revise
      f_zctrl[0] = 0.5 * zctrl;
      f_zctrl[1] = -0.5 * zctrl;
      // foot force independent damping control
      // for (size_t i = 0; i < 2; i++) {
      //   f_zctrl[i] = calcDampingControl (ref_foot_force[i](2),
      //                                    fz[i], f_zctrl[i], eefm_pos_damping_gain, eefm_pos_time_const);
      //   f_zctrl[i] = vlimit(f_zctrl[i], -0.05, 0.05);
      // }
    }
  } // st_algorithm == OpenHRP::StabilizerService::EEFM

  //needed for idle mode
  for ( int i = 0; i < m_robot->numJoints(); i++ ){
    m_robot->joint(i)->q() = qrefv[i];
  }
  m_robot->rootLink()->p() = target_root_p;
  m_robot->rootLink()->R() = target_root_R;
  

 
  // recalculate in calceeforceMomentcontrol
  /*
  if ( !(control_mode == MODE_IDLE || control_mode == MODE_AIR) ) {
    for (size_t i = 0; i < 2; i++) {
      for ( int j = 0; j < manip2[i]->numJoints(); j++ ){
        int idx = manip2[i]->joint(j)->jointId();
        m_robot->joint(idx)->q() = qorg[idx];
      }
    }
    m_robot->rootLink()->p()(0) = current_root_p(0);
    m_robot->rootLink()->p()(1) = current_root_p(1);
    m_robot->rootLink()->R() = current_root_R;
    m_robot->calcForwardKinematics();
  }
  */
  copy (contact_states.begin(), contact_states.end(), prev_contact_states.begin());
}

double Stabilizer::calcAlpha (const Vector3& tmprefzmp,
                              const std::vector<Vector3>& ee_pos,
                              const std::vector<Matrix3>& ee_rot)
{
  double alpha;
  Vector3 l_local_zmp = ee_rot[1].transpose() * (tmprefzmp-ee_pos[1]);
  Vector3 r_local_zmp = ee_rot[0].transpose() * (tmprefzmp-ee_pos[0]);
  if ( is_inside_foot(l_local_zmp, true) && !is_front_of_foot(l_local_zmp) && !is_rear_of_foot(l_local_zmp)) { // new_refzmp is inside lfoot
    alpha = 0.0;
  } else if ( is_inside_foot(r_local_zmp, false) && !is_front_of_foot(r_local_zmp) && !is_rear_of_foot(r_local_zmp)) { // new_refzmp is inside rfoot
    alpha = 1.0;
  } else {
    Vector3 ledge_foot;
    Vector3 redge_foot;
    // lleg
    if (is_inside_foot(l_local_zmp, true) && is_front_of_foot(l_local_zmp)) {
      ledge_foot = Vector3(eefm_leg_front_margin, l_local_zmp(1), 0.0);
    } else if (!is_inside_foot(l_local_zmp, true) && is_front_of_foot(l_local_zmp)) {
      ledge_foot = Vector3(eefm_leg_front_margin, -1 * eefm_leg_inside_margin, 0.0);
    } else if (!is_inside_foot(l_local_zmp, true) && !is_front_of_foot(l_local_zmp) && !is_rear_of_foot(l_local_zmp)) {
      ledge_foot = Vector3(l_local_zmp(0), -1 * eefm_leg_inside_margin, 0.0);
    } else if (!is_inside_foot(l_local_zmp, true) && is_rear_of_foot(l_local_zmp)) {
      ledge_foot = Vector3(-1 * eefm_leg_rear_margin, -1 * eefm_leg_inside_margin, 0.0);
    } else {
      ledge_foot = Vector3(-1 * eefm_leg_rear_margin, l_local_zmp(1), 0.0);
    }
    ledge_foot = ee_rot[1] * ledge_foot + ee_pos[1];
    // rleg
    if (is_inside_foot(r_local_zmp, false) && is_front_of_foot(r_local_zmp)) {
      redge_foot = Vector3(eefm_leg_front_margin, r_local_zmp(1), 0.0);
    } else if (!is_inside_foot(r_local_zmp, false) && is_front_of_foot(r_local_zmp)) {
      redge_foot = Vector3(eefm_leg_front_margin, eefm_leg_inside_margin, 0.0);
    } else if (!is_inside_foot(r_local_zmp, false) && !is_front_of_foot(r_local_zmp) && !is_rear_of_foot(r_local_zmp)) {
      redge_foot = Vector3(r_local_zmp(0), eefm_leg_inside_margin, 0.0);
    } else if (!is_inside_foot(r_local_zmp, false) && is_rear_of_foot(r_local_zmp)) {
      redge_foot = Vector3(-1 * eefm_leg_rear_margin, eefm_leg_inside_margin, 0.0);
    } else {
      redge_foot = Vector3(-1 * eefm_leg_rear_margin, r_local_zmp(1), 0.0);
    }
    redge_foot = ee_rot[0] * redge_foot + ee_pos[0];
    // calc alpha
    Vector3 difp = redge_foot - ledge_foot;
    alpha = difp.dot(tmprefzmp-ledge_foot)/difp.squaredNorm();
  }
  return alpha;
};

void Stabilizer::getTargetParameters ()
{
  // Reference world frame =>
  // update internal robot model
  if ( transition_count == 0 ) {
    transition_smooth_gain = 1.0;
  } else {
    transition_smooth_gain = 1/(1+exp(-9.19*(((MAX_TRANSITION_COUNT - std::fabs(transition_count)) / MAX_TRANSITION_COUNT) - 0.5)));
  }
  if (transition_count > 0) {
    for ( int i = 0; i < m_robot->numJoints(); i++ ){
      m_robot->joint(i)->q() = ( m_qRef.data[i] - transition_joint_q[i] ) * transition_smooth_gain + transition_joint_q[i];
    }
  } else {
    for ( int i = 0; i < m_robot->numJoints(); i++ ){
      m_robot->joint(i)->q() = m_qRef.data[i];
    }
  }
  if ( transition_count < 0 ) {
    transition_count++;
  } else if ( transition_count > 0 ) {
    transition_count--;
  }
  for ( int i = 0; i < m_robot->numJoints(); i++ ){
    qrefv[i] = m_robot->joint(i)->q();
  }
  m_robot->rootLink()->p() = Vector3(m_basePos.data.x, m_basePos.data.y, m_basePos.data.z);
  target_root_p = m_robot->rootLink()->p();
  m_robot->rootLink()->R()= rotFromRpy(m_baseRpy.data.r, m_baseRpy.data.p, m_baseRpy.data.y);
  target_root_R = m_robot->rootLink()->R() ;
  m_robot->calcForwardKinematics();
  ref_zmp = m_robot->rootLink()->R() * Vector3(m_zmpRef.data.x, m_zmpRef.data.y, m_zmpRef.data.z) + m_robot->rootLink()->p(); // base frame -> world frame
  
  if (st_algorithm == OpenHRP::StabilizerService::EEFM) {
    // apply inverse system
    Vector3 tmp_ref_zmp=ref_zmp + eefm_zmp_delay_time_const[0] * (ref_zmp - prev_ref_zmp)/ dt;
   
    //wutest with out inverse system
    tmp_ref_zmp = ref_zmp;

    prev_ref_zmp = ref_zmp;
    ref_zmp = tmp_ref_zmp;
  }
  ref_cog = m_robot->calcCenterOfMass();
  for (size_t i = 0; i < 2; i++) {
    //hrp::Sensor* sen = m_robot->sensor<hrp::ForceSensor>(sensor_names[i]);
    ForceSensor* sen=forceSensors[i];
    if ( sen != NULL) {
      target_foot_p[i] = sen->link()->p() + sen->link()->R() * ee_map[sen->link()->name()].localp;
      target_foot_R[i] = sen->link()->R() * ee_map[sen->link()->name()].localR;
    }
  }
  // <= Reference world frame

  if (st_algorithm == OpenHRP::StabilizerService::EEFM) {
    // Reference foot_origin frame =>
    Vector3 foot_origin_pos;
    Matrix3 foot_origin_rot;
    calcFootOriginCoords (foot_origin_pos, foot_origin_rot);
    // initialize for new_refzmp
    //new_refzmp = ref_zmp;
    //use in calcEEForceMomentControl() latter
    rel_cog = m_robot->rootLink()->R().transpose() * (ref_cog-m_robot->rootLink()->p());
    // convert world (current-tmp) => local (foot_origin)
    zmp_origin_off = ref_zmp(2) - foot_origin_pos(2);
    ref_zmp = foot_origin_rot.transpose() * (ref_zmp - foot_origin_pos);
    new_refzmp = ref_zmp;
    //new_refzmp = foot_origin_rot.transpose() * (new_refzmp - foot_origin_pos);
    ref_cog = foot_origin_rot.transpose() * (ref_cog - foot_origin_pos);
   
   
    if (contact_states != prev_contact_states) {
      ref_cogvel = (foot_origin_rot.transpose() * prev_ref_foot_origin_rot) * ref_cogvel;
    } else {
      ref_cogvel = (ref_cog - prev_ref_cog)/dt;
    }
    prev_ref_foot_origin_rot = foot_origin_rot;
    // <= Reference foot_origin frame
  } else {
    ref_cogvel = (ref_cog - prev_ref_cog)/dt;
  } // st_algorithm == OpenHRP::StabilizerService::EEFM
  prev_ref_cog = ref_cog;
}

bool Stabilizer::calcZMP(Vector3& ret_zmp, const double zmp_z)
{
  double tmpzmpx = 0;
  double tmpzmpy = 0;
  double tmpfz = 0, tmpfz2 = 0.0;
  double pre_act_force_zORG[2];
  for (size_t i = 0; i < 2; i++) {
    //hrp::ForceSensor* sensor = m_robot->sensor<hrp::ForceSensor>(sensor_names[i]);
    ForceSensor* sensor=forceSensors[i];
    Vector3 fsp = sensor->link()->p() + sensor->link()->R() * sensor->p_local();
    Matrix3 tmpR;
    rats::rotm3times(tmpR, sensor->link()->R(), sensor->R_local());
    Vector3 nf = tmpR * Vector3(m_force[i].data[0], m_force[i].data[1], m_force[i].data[2]);
    Vector3 nm = tmpR * Vector3(m_force[i].data[3], m_force[i].data[4], m_force[i].data[5]);
    
    tmpzmpx += nf(2) * fsp(0) - (fsp(2) - zmp_z) * nf(0) - nm(1);
    tmpzmpy += nf(2) * fsp(1) - (fsp(2) - zmp_z) * nf(1) + nm(0);
    tmpfz += nf(2);
    //wu
    pre_act_force_zORG[i] = prev_act_force_z[i];
    prev_act_force_z[i] = 0.85 * prev_act_force_z[i] + 0.15 * nf(2); // filter, cut off 5[Hz]
  }
  tmpfz2 = prev_act_force_z[0] + prev_act_force_z[1];

  //wu
  if (tmpfz2 < 50) {
    prev_act_force_z[0] =  pre_act_force_zORG[0];
    prev_act_force_z[1] =  pre_act_force_zORG[1];
    tmpfz2 = prev_act_force_z[0] + prev_act_force_z[1];
  } 
  ret_zmp = Vector3(tmpzmpx / tmpfz, tmpzmpy / tmpfz, zmp_z);
  return true; // on ground
  

  /*
  if (tmpfz2 < 50) {
    ret_zmp = act_zmp;
    return false; // in the air
  } else {
    ret_zmp = Vector3(tmpzmpx / tmpfz, tmpzmpy / tmpfz, zmp_z);
    return true; // on ground
  }
  */
};

/*
void Stabilizer::calcTPCC() {
  if ( m_robot->numJoints() == m_qRef.data.length() ) {

    // stabilizer loop
    if ( ( m_force[ST_LEFT].data.length() > 0 && m_force[ST_RIGHT].data.length() > 0 ) ) {
      // Choi's feedback law
      Vector3 cog = m_robot->calcCenterOfMass();
      Vector3 newcog = Vector3::Zero();
      Vector3 dcog(ref_cog - act_cog);
      Vector3 dzmp(ref_zmp - act_zmp);
      for (size_t i = 0; i < 2; i++) {
        double uu = ref_cogvel(i) - k_tpcc_p[i] * transition_smooth_gain * dzmp(i)
                                  + k_tpcc_x[i] * transition_smooth_gain * dcog(i);
        newcog(i) = uu * dt + cog(i);
      }

      //rpy control
      {
        //hrp::Sensor* sen = m_robot->sensor<hrp::RateGyroSensor>("gyrometer");
	RateGyroSensor* sen=RateGyroSensors[0];
        if (sen != NULL) {
          Matrix3 act_Rs(hrp::rotFromRpy(m_rpy.data.r, m_rpy.data.p, m_rpy.data.y));
          Matrix3 tmpm, act_Rb;
          //rats::rotm3times(tmpm, Matrix3(sen->link->R() * sen->localR).transpose(), m_robot->rootLink()->R());
	  rats::rotm3times(tmpm, Matrix3(sen->link->R() * sen->R_local()).transpose(), m_robot->rootLink()->R());
          rats::rotm3times(act_Rb, act_Rs, tmpm);
          Vector3 act_rpy = hrp::rpyFromRot(act_Rb);
          Vector3 ref_rpy = hrp::rpyFromRot(target_root_R);
          for (size_t i = 0; i < 2; i++) {
            d_rpy[i] = transition_smooth_gain * (k_brot_p[i] * (ref_rpy(i) - act_rpy(i)) - 1/k_brot_tc[i] * d_rpy[i]) * dt + d_rpy[i];
          }
          rats::rotm3times(current_root_R, target_root_R, hrp::rotFromRpy(d_rpy[0], d_rpy[1], 0));
          m_robot->rootLink()->R() = current_root_R;
        }
      }

      // ee target => link-origin target
      for (size_t i = 0; i < 2; i++) {
        hrp::Link* target = m_robot->sensor<hrp::ForceSensor>(sensor_names[i])->link;
        Matrix3 tmpR;
        rats::rotm3times(tmpR, target_foot_R[i], ee_map[target->name()].localR.transpose());
        target_foot_R[i] = tmpR;
        target_foot_p[i] -=  target_foot_R[i] * ee_map[target->name()].localp;
      }
      // solveIK
      //for (size_t jj = 0; jj < 5; jj++) {
      for (size_t jj = 0; jj < 3; jj++) {
        Vector3 tmpcm = m_robot->calcCenterOfMass();
        for (size_t i = 0; i < 2; i++) {
          m_robot->rootLink()->p()(i) = m_robot->rootLink()->p()(i) + 0.9 * (newcog(i) - tmpcm(i));
        }
        m_robot->calcForwardKinematics();
        for (size_t i = 0; i < 2; i++) {
          hrp::Link* target = m_robot->sensor<hrp::ForceSensor>(sensor_names[i])->link;
          Vector3 vel_p, vel_r;
          vel_p = target_foot_p[i] - target->p();
          rats::difference_rotation(vel_r, target->R(), target_foot_R[i]);
          manip2[i]->calcInverseKinematics2Loop(vel_p, vel_r, 1.0, 0.001, 0.01, &qrefv);
        }
      }
    }
  }
}
*/

void Stabilizer::calcEEForceMomentControl() {
  if ( m_robot->numJoints() == m_qRef.data.length() ) {

    // stabilizer loop
    if ( ( m_force[0].data.length() > 0 && m_force[1].data.length() > 0 ) ) {

      // return to referencea
      m_robot->rootLink()->R() = target_root_R;
      m_robot->rootLink()->p() = target_root_p;
      for ( int i = 0; i < m_robot->numJoints(); i++ ) {
        m_robot->joint(i)->q() = qrefv[i];
      }
     
      //rpy control
      rats::rotm3times(current_root_R, target_root_R, rotFromRpy(d_rpy[0], d_rpy[1], 0));
     
      //org
      //m_robot->rootLink()->R() = current_root_R;
      //m_robot->rootLink()->p() = target_root_p + target_root_R * rel_cog - current_root_R * rel_cog;
      //m_robot->calcForwardKinematics();
      //current_base_rpy = rot2rpy(m_robot->rootLink()->R());//for log
      //current_base_pos = m_robot->rootLink()->p();//for log
      
      //wutest rotate waist, keep foot and CoM in the same place
      m_robot->calcForwardKinematics();
      if(!CalcIVK_biped(m_robot, current_root_R, m_contactStates, end_link))
	std::cout<<"InverseKinametic bipped error"<<std::endl;


      // foot modif
      Vector3 total_target_foot_p[2];
      Matrix3 total_target_foot_R[2];
      {
        // moment control
#define deg2rad(x) ((x) * M_PI / 180.0)
        for (size_t i = 0; i < 2; i++) { //rotate and desire torque is in inverse direction
            rats::rotm3times(total_target_foot_R[i], target_foot_R[i], rotFromRpy(-ee_d_foot_rpy[i](0), -ee_d_foot_rpy[i](1), 0));
        }
        for (size_t i = 0; i < 2; i++) {
          //hrp::Link* target = m_robot->sensor<hrp::ForceSensor>(sensor_names[i])->link;
	  Link* target = forceSensors[i]->link();
          total_target_foot_p[i](0) = target_foot_p[i](0);
          total_target_foot_p[i](1) = target_foot_p[i](1);
          // foot force difference control version
          // total_target_foot_p[i](2) = target_foot_p[i](2) + (i==0?0.5:-0.5)*zctrl;
          // foot force independent damping control
          //total_target_foot_p[i](2) = target_foot_p[i](2) - f_zctrl[i];
	  //wutest
	  total_target_foot_p[i](2) = target_foot_p[i](2) + f_zctrl[i];
        }
      }

      // ee=>link-origin
      for (size_t i = 0; i < 2; i++) {
        //hrp::Link* target = m_robot->sensor<hrp::ForceSensor>(sensor_names[i])->link;
	Link* target = forceSensors[i]->link();
        Matrix3 tmpR;
        rats::rotm3times(tmpR, total_target_foot_R[i], ee_map[target->name()].localR.transpose());
        total_target_foot_R[i] = tmpR;
        total_target_foot_p[i] -= total_target_foot_R[i] * ee_map[target->name()].localp;
      }
      // solveIK
      for (size_t jj = 0; jj < 3; jj++) {
        m_robot->calcForwardKinematics();
        for (size_t i = 0; i < 2; i++) {
          //hrp::Link* target = m_robot->sensor<hrp::ForceSensor>(sensor_names[i])->link;
	  Link* target = forceSensors[i]->link();
          Vector3 vel_p, vel_r;
          vel_p = total_target_foot_p[i] - target->p();
          rats::difference_rotation(vel_r, target->R(), total_target_foot_R[i]);
          vel_p *= transition_smooth_gain;
          vel_r *= transition_smooth_gain;
          manip2[i]->calcInverseKinematics2Loop(vel_p, vel_r, 1.0, 0.001, 0.01, &qrefv);
        }
      }
    }
  }
}

double Stabilizer::calcDampingControl (const double tau_d, const double tau, const double prev_d,
                                       const double DD, const double TT)
{
  return (1/DD * (tau_d - tau) - 1/TT * prev_d) * dt + prev_d;
};

/*
RTC::ReturnCode_t Stabilizer::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Stabilizer::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Stabilizer::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Stabilizer::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Stabilizer::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

void Stabilizer::sync_2_st ()
{
  std::cerr << "[" << m_profile.instance_name << "] " << "Sync IDLE => ST"  << std::endl;
  pangx_ref = pangy_ref = pangx = pangy = 0;
  rdx = rdy = rx = ry = 0;
  d_rpy[0] = d_rpy[1] = 0;
  pdr = Vector3::Zero();
  zctrl = f_zctrl[0] = f_zctrl[1] = 0.0;
  d_foot_rpy[0] = d_foot_rpy[1] = Vector3::Zero();
  ee_d_foot_rpy[0] = ee_d_foot_rpy[1] = Vector3::Zero();
  if (on_ground) {
    transition_count = -MAX_TRANSITION_COUNT;
    control_mode = MODE_ST;
  } else {
    transition_count = 0;
    control_mode = MODE_AIR;
  }
}

void Stabilizer::sync_2_idle ()
{
  std::cerr << "[" << m_profile.instance_name << "] " << "Sync ST => IDLE"  << std::endl;
  transition_count = MAX_TRANSITION_COUNT;
  for (int i = 0; i < m_robot->numJoints(); i++ ) {
    transition_joint_q[i] = m_robot->joint(i)->q();
  }
}

void Stabilizer::startStabilizer(void)
{
  if ( transition_count == 0 && control_mode == MODE_IDLE ) {
    std::cerr << "[" << m_profile.instance_name << "] " << "Start ST"  << std::endl;
    sync_2_st();
    waitSTTransition();
    std::cerr << "[" << m_profile.instance_name << "] " << "Start ST DONE"  << std::endl;
  }
}

void Stabilizer::stopStabilizer(void)
{
  if ( transition_count == 0 && (control_mode == MODE_ST || control_mode == MODE_AIR) ) {
    std::cerr << "[" << m_profile.instance_name << "] " << "Stop ST"  << std::endl;
    control_mode = MODE_SYNC_TO_IDLE;
    waitSTTransition();
    std::cerr << "[" << m_profile.instance_name << "] " << "Stop ST DONE"  << std::endl;
  }
}

void Stabilizer::getParameter(OpenHRP::StabilizerService::stParam& i_stp)
{
  for (size_t i = 0; i < 2; i++) {
    i_stp.k_run_b[i] = k_run_b[i];
    i_stp.d_run_b[i] = d_run_b[i];
    ////m_tau_x[i].setup(i_stp.tdfke[0], i_stp.tdftc[0], dt);
    ////m_tau_y[i].setup(i_stp.tdfke[0], i_stp.tdftc[0], dt);
    ////m_f_z.setup(i_stp.tdfke[1], i_stp.tdftc[1], dt);
    i_stp.k_tpcc_p[i] = k_tpcc_p[i];
    i_stp.k_tpcc_x[i] = k_tpcc_x[i];
    i_stp.k_brot_p[i] = k_brot_p[i];
    i_stp.k_brot_tc[i] = k_brot_tc[i];
  }
  i_stp.k_run_x = m_torque_k[0];
  i_stp.k_run_y = m_torque_k[1];
  i_stp.d_run_x = m_torque_d[0];
  i_stp.d_run_y = m_torque_d[1];
  for (size_t i = 0; i < 2; i++) {
    i_stp.eefm_k1[i] = eefm_k1[i];
    i_stp.eefm_k2[i] = eefm_k2[i];
    i_stp.eefm_k3[i] = eefm_k3[i];
    i_stp.eefm_zmp_delay_time_const[i] = eefm_zmp_delay_time_const[i];
    i_stp.eefm_ref_zmp_aux[i] = ref_zmp_aux(i);
    i_stp.eefm_body_attitude_control_time_const[i] = eefm_body_attitude_control_time_const[i];
    i_stp.eefm_body_attitude_control_gain[i] = eefm_body_attitude_control_gain[i];
    i_stp.eefm_rot_damping_gain[i] = eefm_rot_damping_gain[i];
  }

  i_stp.eefm_pos_damping_gain = eefm_pos_damping_gain;
  i_stp.eefm_rot_time_const = eefm_rot_time_const;
  i_stp.eefm_pos_time_const_support = eefm_pos_time_const_support;
  i_stp.eefm_pos_time_const_swing = eefm_pos_time_const_swing;
  i_stp.eefm_pos_transition_time = eefm_pos_transition_time;
  i_stp.eefm_pos_margin_time = eefm_pos_margin_time;
  i_stp.eefm_leg_inside_margin = eefm_leg_inside_margin;
  i_stp.eefm_leg_front_margin = eefm_leg_front_margin;
  i_stp.eefm_leg_rear_margin = eefm_leg_rear_margin;
  i_stp.eefm_cogvel_cutoff_freq = eefm_cogvel_cutoff_freq;
  i_stp.st_algorithm = st_algorithm;
  //wutest
  i_stp.eefm_pos_control_switch = eefm_pos_control_switch;
  i_stp.eefm_rot_control_switch = eefm_rot_control_switch;
  i_stp.eefm_att_control_switch = eefm_att_control_switch;

  switch(control_mode) {
  case MODE_IDLE: i_stp.controller_mode = OpenHRP::StabilizerService::MODE_IDLE; break;
  case MODE_AIR: i_stp.controller_mode = OpenHRP::StabilizerService::MODE_AIR; break;
  case MODE_ST: i_stp.controller_mode = OpenHRP::StabilizerService::MODE_ST; break;
  case MODE_SYNC_TO_IDLE: i_stp.controller_mode = OpenHRP::StabilizerService::MODE_SYNC_TO_IDLE; break;
  case MODE_SYNC_TO_AIR: i_stp.controller_mode = OpenHRP::StabilizerService::MODE_SYNC_TO_AIR; break;
  default: break;
  }
};

void Stabilizer::setParameter(const OpenHRP::StabilizerService::stParam& i_stp)
{
  std::cerr << "[" << m_profile.instance_name << "] setParameter" << std::endl;
  for (size_t i = 0; i < 2; i++) {
    k_tpcc_p[i] = i_stp.k_tpcc_p[i];
    k_tpcc_x[i] = i_stp.k_tpcc_x[i];
    k_brot_p[i] = i_stp.k_brot_p[i];
    k_brot_tc[i] = i_stp.k_brot_tc[i];
  }
  //std::cerr << "[" << m_profile.instance_name << "]  TPCC" << std::endl;
  //std::cerr << "[" << m_profile.instance_name << "]   k_tpcc_p  = [" << k_tpcc_p[0] << ", " <<  k_tpcc_p[1] << "]" << std::endl;
  //std::cerr << "[" << m_profile.instance_name << "]   k_tpcc_x  = [" << k_tpcc_x[0] << ", " << k_tpcc_x[1] << "]" << std::endl;
  //std::cerr << "[" << m_profile.instance_name << "]   k_brot_p  = [" << k_brot_p[0] << ", " << k_brot_p[1] << "]" << std::endl;
  //std::cerr << "[" << m_profile.instance_name << "]   k_brot_tc = [" << k_brot_tc[0] << ", " << k_brot_tc[1] << "]" << std::endl;
  for (size_t i = 0; i < 2; i++) {
    k_run_b[i] = i_stp.k_run_b[i];
    d_run_b[i] = i_stp.d_run_b[i];
    //m_tau_x[i].setup(i_stp.tdfke[0], i_stp.tdftc[0], dt);
    //m_tau_y[i].setup(i_stp.tdfke[0], i_stp.tdftc[0], dt);
    //m_f_z.setup(i_stp.tdfke[1], i_stp.tdftc[1], dt);
  }
  m_torque_k[0] = i_stp.k_run_x;
  m_torque_k[1] = i_stp.k_run_y;
  m_torque_d[0] = i_stp.d_run_x;
  m_torque_d[1] = i_stp.d_run_y;
  //std::cerr << "[" << m_profile.instance_name << "]  RUNST" << std::endl;
  //std::cerr << "[" << m_profile.instance_name << "]   m_torque_k  = [" << m_torque_k[0] << ", " <<  m_torque_k[1] << "]" << std::endl;
  //std::cerr << "[" << m_profile.instance_name << "]   m_torque_d  = [" << m_torque_d[0] << ", " <<  m_torque_d[1] << "]" << std::endl;
  //std::cerr << "[" << m_profile.instance_name << "]   k_run_b  = [" << k_run_b[0] << ", " <<  k_run_b[1] << "]" << std::endl;
  //std::cerr << "[" << m_profile.instance_name << "]   d_run_b  = [" << d_run_b[0] << ", " <<  d_run_b[1] << "]" << std::endl;
  //std::cerr << "[" << m_profile.instance_name << "]  EEFM" << std::endl;
  for (size_t i = 0; i < 2; i++) {
    eefm_k1[i] = i_stp.eefm_k1[i];
    eefm_k2[i] = i_stp.eefm_k2[i];
    eefm_k3[i] = i_stp.eefm_k3[i];
    eefm_zmp_delay_time_const[i] = i_stp.eefm_zmp_delay_time_const[i];
    ref_zmp_aux(i) = i_stp.eefm_ref_zmp_aux[i];
    eefm_body_attitude_control_gain[i] = i_stp.eefm_body_attitude_control_gain[i];
    eefm_body_attitude_control_time_const[i] = i_stp.eefm_body_attitude_control_time_const[i];
    eefm_rot_damping_gain[i] = i_stp.eefm_rot_damping_gain[i];
  }
 
  eefm_pos_damping_gain = i_stp.eefm_pos_damping_gain;
  eefm_rot_time_const = i_stp.eefm_rot_time_const;
  eefm_pos_time_const_support = i_stp.eefm_pos_time_const_support;
  eefm_pos_time_const_swing = i_stp.eefm_pos_time_const_swing;
  eefm_pos_transition_time = i_stp.eefm_pos_transition_time;
  eefm_pos_margin_time = i_stp.eefm_pos_margin_time;
  eefm_leg_inside_margin = i_stp.eefm_leg_inside_margin;
  eefm_leg_front_margin = i_stp.eefm_leg_front_margin;
  eefm_leg_rear_margin = i_stp.eefm_leg_rear_margin;
  eefm_cogvel_cutoff_freq = i_stp.eefm_cogvel_cutoff_freq;
  
  //wutest
  eefm_pos_control_switch=  i_stp.eefm_pos_control_switch;
  eefm_rot_control_switch=  i_stp.eefm_rot_control_switch;
  eefm_att_control_switch=  i_stp.eefm_att_control_switch;

std::cerr << "[" << m_profile.instance_name << "]   eefm_k1  = [" << eefm_k1[0] << ", " << eefm_k1[1] << "]" << std::endl;
  std::cerr << "[" << m_profile.instance_name << "]   eefm_k2  = [" << eefm_k2[0] << ", " << eefm_k2[1] << "]" << std::endl;
  std::cerr << "[" << m_profile.instance_name << "]   eefm_k3  = [" << eefm_k3[0] << ", " << eefm_k3[1] << "]" << std::endl;
  std::cerr << "[" << m_profile.instance_name << "]   eefm_zmp_delay_time_const  = [" << eefm_zmp_delay_time_const[0] << ", " << eefm_zmp_delay_time_const[1] << "][s]" << std::endl;
  std::cerr << "[" << m_profile.instance_name << "]   eefm_ref_zmp_aux  = [" << ref_zmp_aux(0) << ", " << ref_zmp_aux(1) << "][m]" << std::endl;
  std::cerr << "[" << m_profile.instance_name << "]   eefm_body_attitude_control_gain  = [" << eefm_body_attitude_control_gain[0] << ", " << eefm_body_attitude_control_gain[1] <<"]"<<" switch "<<eefm_att_control_switch << std::endl;
  std::cerr << "[" << m_profile.instance_name << "]   eefm_body_attitude_control_time_const  = [" << eefm_body_attitude_control_time_const[0] << ", " << eefm_body_attitude_control_time_const[1] << "][s]" << std::endl;
  std::cerr << "[" << m_profile.instance_name << "]   eefm_rot_damping_gain = " << eefm_rot_damping_gain[0]<<" "<<  eefm_rot_damping_gain[1]<< ", eefm_rot_time_const = " << eefm_rot_time_const << "[s]" <<" switch "<<eefm_rot_control_switch<< std::endl;
  std::cerr << "[" << m_profile.instance_name << "]   eefm_pos_damping_gain = " << eefm_pos_damping_gain << ", eefm_pos_time_const_support = " << eefm_pos_time_const_support << "[s], "
            << "eefm_pos_time_const_swing = " << eefm_pos_time_const_swing << "[s]" << std::endl;
  std::cerr << "[" << m_profile.instance_name << "]   eefm_pos_transition_time = " << eefm_pos_transition_time << "[s], eefm_pos_margin_time = " << eefm_pos_margin_time << "[s]" <<" switch "<<eefm_pos_control_switch<< std::endl;
  std::cerr << "[" << m_profile.instance_name << "]   eefm_leg_inside_margin = " << eefm_leg_inside_margin << "[m], eefm_leg_front_margin = " << eefm_leg_front_margin << "[m], eefm_leg_rear_margin = " << eefm_leg_rear_margin << "[m]" << std::endl;
  std::cerr << "[" << m_profile.instance_name << "]   eefm_cogvel_cutoff_freq = " << eefm_cogvel_cutoff_freq << "[Hz]" << std::endl;
  std::cerr << "[" << m_profile.instance_name << "]  COMMON" << std::endl;
  
  if (control_mode == MODE_IDLE) {
    st_algorithm = i_stp.st_algorithm;
    std::cerr << "[" << m_profile.instance_name << "]   st_algorithm changed to [" << (st_algorithm == OpenHRP::StabilizerService::EEFM?"EEFM":"TPCC") << "]" << std::endl;
  } else {
    std::cerr << "[" << m_profile.instance_name << "]   st_algorithm cannot be changed to [" << (st_algorithm == OpenHRP::StabilizerService::EEFM?"EEFM":"TPCC") << "] during MODE_AIR or MODE_ST." << std::endl;
  }

  std::cerr <<'\n'<<std::endl;
}

void Stabilizer::waitSTTransition()
{
  while (transition_count != 0) usleep(10);
  usleep(10);
}

static double vlimit(double value, double llimit_value, double ulimit_value)
{
  if (value > ulimit_value) {
    return ulimit_value;
  } else if (value < llimit_value) {
    return llimit_value;
  }
  return value;
}

static Vector3 rot2rpy(Matrix3 R)
{
  Vector3 euler =R.eulerAngles(2,1,0);
  Vector3 out(euler(2), euler(1), euler(0));

  return out;
}

/*
static double switching_inpact_absorber(double force, double lower_th, double upper_th)
{
  double gradient, intercept;
  if (force < lower_th) {
    return 0;
  } else if (force > upper_th) {
    return 1;
  } else {
    gradient = 1.0 / (upper_th - lower_th);
    intercept = -lower_th * gradient;
    return gradient * force + intercept;
  }
}
*/

/*
void Stabilizer::calcRUNST() {
  if ( m_robot->numJoints() == m_qRef.data.length() ) {
    std::vector<std::string> target_name;
    target_name.push_back("L_ANKLE_R");
    target_name.push_back("R_ANKLE_R");

    double angvelx_ref;// = (m_rpyRef.data.r - pangx_ref)/dt;
    double angvely_ref;// = (m_rpyRef.data.p - pangy_ref)/dt;
    //pangx_ref = m_rpyRef.data.r;
    //pangy_ref = m_rpyRef.data.p;
    double angvelx = (m_rpy.data.r - pangx)/dt;
    double angvely = (m_rpy.data.r - pangy)/dt;
    pangx = m_rpy.data.r;
    pangy = m_rpy.data.p;

    // update internal robot model
    for ( int i = 0; i < m_robot->numJoints(); i++ ){
      qorg[i] = m_robot->joint(i)->q();
      m_robot->joint(i)->q() = m_qRef.data[i];
      qrefv[i] = m_qRef.data[i];
    }
    //double orgjq = m_robot->link("L_FOOT")->joint->q();
    double orgjq = m_robot->joint(m_robot->link("L_ANKLE_P")->jointId())->q();
    //set root
    m_robot->rootLink()->p() = Vector3(0,0,0);
    //m_robot->rootLink()->R() = hrp::rotFromRpy(m_rpyRef.data.r,m_rpyRef.data.p,m_rpyRef.data.y);
    m_robot->calcForwardKinematics();
    Vector3 target_root_p = m_robot->rootLink()->p();
    Matrix3 target_root_R = m_robot->rootLink()->R();
    Vector3 target_foot_p[2];
    Matrix3 target_foot_R[2];
    for (size_t i = 0; i < 2; i++) {
      target_foot_p[i] = m_robot->link(target_name[i])->p();
      target_foot_R[i] = m_robot->link(target_name[i])->R();
    }
    Vector3 target_fm = (m_robot->link(target_name[0])->p() + m_robot->link(target_name[1])->p())/2;
    //Vector3 org_cm = m_robot->rootLink()->R().transpose() * (m_robot->calcCenterOfMass() - m_robot->rootLink()->p());
    Vector3 org_cm = m_robot->rootLink()->R().transpose() * (target_fm - m_robot->rootLink()->p());

    // stabilizer loop
    if ( ( m_force[ST_LEFT].data.length() > 0 && m_force[ST_RIGHT].data.length() > 0 )
         //( m_force[ST_LEFT].data[2] > m_robot->totalMass()/4 || m_force[ST_RIGHT].data[2] > m_robot->totalMass()/4 )
         ) {

      for ( int i = 0; i < m_robot->numJoints(); i++ ) {
        m_robot->joint(i)->q() = qorg[i];
      }
      // set root
      double rddx;// = k_run_b[0] * (m_rpyRef.data.r - m_rpy.data.r) + d_run_b[0] * (angvelx_ref - angvelx);
      double rddy;// = k_run_b[1] * (m_rpyRef.data.p - m_rpy.data.p) + d_run_b[1] * (angvely_ref - angvely);
      rdx += rddx * dt;
      rx += rdx * dt;
      rdy += rddy * dt;
      ry += rdy * dt;
      //rx += rddx * dt;
      //ry += rddy * dt;
      // if (DEBUGP2) {
      //   std::cerr << "REFRPY " <<  m_rpyRef.data.r << " " << m_rpyRef.data.p << std::endl;
      // }
      // if (DEBUGP2) {
      //   std::cerr << "RPY " <<  m_rpy.data.r << " " << m_rpy.data.p << std::endl;
      //   std::cerr << " rx " << rx << " " << rdx << " " << rddx << std::endl;
      //   std::cerr << " ry " << ry << " " << rdy << " " << rddy << std::endl;
      // }
      Vector3 root_p_s;
      Matrix3 root_R_s;
      rats::rotm3times(root_R_s, hrp::rotFromRpy(rx, ry, 0), target_root_R);
      if (DEBUGP2) {
        Vector3 tmp = hrp::rpyFromRot(root_R_s);
        std::cerr << "RPY2 " <<  tmp(0) << " " << tmp(1) << std::endl;
      }
      root_p_s = target_root_p + target_root_R * org_cm - root_R_s * org_cm;
      //m_robot->calcForwardKinematics();
      // FK
      m_robot->rootLink()->R() = root_R_s;
      m_robot->rootLink()->p() = root_p_s;
      if (DEBUGP2) {
        std::cerr << " rp " << root_p_s[0] << " " << root_p_s[1] << " " << root_p_s[2] << std::endl;
      }
      m_robot->calcForwardKinematics();
      //
      Vector3 current_fm = (m_robot->link(target_name[0])->p() + m_robot->link(target_name[1])->p())/2;

      // 3D-LIP model contorller
      Vector3 dr = target_fm - current_fm;
      //Vector3 dr = current_fm - target_fm ;
      Vector3 dr_vel = (dr - pdr)/dt;
      pdr = dr;
      double tau_y = - m_torque_k[0] * dr(0) - m_torque_d[0] * dr_vel(0);
      double tau_x = m_torque_k[1] * dr(1) + m_torque_d[1] * dr_vel(1);
      if (DEBUGP2) {
        dr*=1e3;
        dr_vel*=1e3;
        std::cerr << "dr " << dr(0) << " " << dr(1) << " " << dr_vel(0) << " " << dr_vel(1) << std::endl;
        std::cerr << "tau_x " << tau_x << std::endl;
        std::cerr << "tau_y " << tau_y << std::endl;
      }

      double gamma = 0.5; // temp
      double tau_xl[2];
      double tau_yl[2];
      double xfront = 0.125;
      double xrear = 0.1;
      double yin = 0.02;
      double yout = 0.15;
      double mg = m_robot->totalMass() * 9.8 * 0.9;// margin
      double tq_y_ulimit = mg * xrear;
      double tq_y_llimit = -1 * mg * xfront;
      double tq_x_ulimit = mg * yout;
      double tq_x_llimit = mg * yin;
      // left
      tau_xl[0] = gamma * tau_x;
      tau_yl[0] = gamma * tau_y;
      tau_xl[0] = vlimit(tau_xl[0], tq_x_llimit, tq_x_ulimit);
      tau_yl[0] = vlimit(tau_yl[0], tq_y_llimit, tq_y_ulimit);
      // right
      tau_xl[1]= (1- gamma) * tau_x;
      tau_yl[1]= (1- gamma) * tau_y;
      tau_xl[1] = vlimit(tau_xl[1], -1*tq_x_ulimit, -1*tq_x_llimit);
      tau_yl[1] = vlimit(tau_yl[1], tq_y_llimit, tq_y_ulimit);

      double dleg_x[2];
      double dleg_y[2];
      double tau_y_total = (m_force[1].data[4] + m_force[0].data[4]) / 2;
      double dpz;
      if (DEBUGP2) {
        std::cerr << "tq limit " << tq_x_ulimit << " " << tq_x_llimit << " " << tq_y_ulimit << " " << tq_y_llimit << std::endl;
      }
      for (size_t i = 0; i < 2; i++) {
        // dleg_x[i] = m_tau_x[i].update(m_force[i].data[3], tau_xl[i]);
        // dleg_y[i] = m_tau_y[i].update(m_force[i].data[4], tau_yl[i]);
        //dleg_x[i] = m_tau_x[i].update(m_force[i].data[3], tau_xl[i]);
        dleg_x[i] = m_tau_x[i].update(0,0);
        dleg_y[i] = m_tau_y[i].update(tau_y_total, tau_yl[i]);
        if (DEBUGP2) {
          std::cerr << i << " dleg_x " << dleg_x[i] << std::endl;
          std::cerr << i << " dleg_y " << dleg_y[i] << std::endl;
          std::cerr << i << " t_x " << m_force[i].data[3] << " "<< tau_xl[i] << std::endl;
          std::cerr << i << " t_y " << m_force[i].data[4] << " "<< tau_yl[i] << std::endl;
        }
      }

      // calc leg rot
      Matrix3 target_R[2];
      Vector3 target_p[2];
      for (size_t i = 0; i < 2; i++) {
        //rats::rotm3times(target_R[i], hrp::rotFromRpy(dleg_x[i], dleg_y[i], 0), target_foot_R[i]);
        rats::rotm3times(target_R[i], hrp::rotFromRpy(0, dleg_y[i], 0), target_foot_R[i]);
        //target_p[i] = target_foot_p[i] + target_foot_R[i] * org_cm - target_R[i] * org_cm;
        //target_p[i] = target_foot_p[i] + target_foot_R[i] * org_cm - target_R[i] * org_cm;
        target_p[i] = target_foot_p[i];
      }
      // 1=>left, 2=>right
      double refdfz = 0;
      dpz = m_f_z.update((m_force[0].data[2] - m_force[1].data[2]), refdfz);
      //target_p[0](2) = target_foot_p[0](2) + dpz/2;
      //target_p[1](2) = target_foot_p[1](2) - dpz/2;
      target_p[0](2) = target_foot_p[0](2);
      target_p[1](2) = target_foot_p[1](2);

      // IK
      for (size_t i = 0; i < 2; i++) {
        hrp::Link* target = m_robot->link(target_name[i]);
        Vector3 vel_p, vel_r;
        vel_p = target_p[i] - target->p();
        rats::difference_rotation(vel_r, target->R(), target_R[i]);
        //manip2[i]->solveLimbIK(vel_p, vel_r, transition_count, 0.001, 0.01, MAX_TRANSITION_COUNT, qrefv, DEBUGP);
        //manip2[i]->solveLimbIK(vel_p, vel_r, transition_count, 0.001, 0.01, MAX_TRANSITION_COUNT, qrefv, false);
        //m_robot->joint(m_robot->link(target_name[i])->jointId)->q() = dleg_y[i] + orgjq;
      }
      // m_robot->joint(m_robot->link("L_ANKLE_P")->jointId)->q() = transition_smooth_gain * dleg_y[0] + orgjq + m_rpy.data.p;
      // m_robot->joint(m_robot->link("R_ANKLE_P")->jointId)->q() = transition_smooth_gain * dleg_y[1] + orgjq + m_rpy.data.p;
      m_robot->joint(m_robot->link("L_ANKLE_P")->jointId())->q() = transition_smooth_gain * dleg_y[0] + orgjq;
      m_robot->joint(m_robot->link("R_ANKLE_P")->jointId())->q() = transition_smooth_gain * dleg_y[1] + orgjq;
    } else {
      // reinitialize
      for (int i = 0; i < ST_NUM_LEGS; i++) {
        m_tau_x[i].reset();
        m_tau_y[i].reset();
        m_f_z.reset();
      }
    }
  }
}
*/

/*
void Stabilizer::calcContactMatrix (MatrixXd& tm, const std::vector<Vector3>& contact_p)
{
  // tm.resize(6,6*contact_p.size());
  // tm.setZero();
  // for (size_t c = 0; c < contact_p.size(); c++) {
  //   for (size_t i = 0; i < 6; i++) tm(i,(c*6)+i) = 1.0;
  //   Matrix3 cm;
  //   rats::outer_product_matrix(cm, contact_p[c]);
  //   for (size_t i = 0; i < 3; i++)
  //     for (size_t j = 0; j < 3; j++) tm(i+3,(c*6)+j) = cm(i,j);
  // }
}
*/

/*
void Stabilizer::calcTorque ()
{
  m_robot->calcForwardKinematics();
  // buffers for the unit vector method
  Vector3 root_w_x_v;
  Vector3 g(0, 0, 9.80665);
  root_w_x_v = m_robot->rootLink()->w.cross(m_robot->rootLink()->vo + m_robot->rootLink()->w.cross(m_robot->rootLink()->p()));
  m_robot->rootLink()->dvo = g - root_w_x_v;   // dv = g, dw = 0
  m_robot->rootLink()->dw.setZero();

  Vector3 root_f;
  Vector3 root_t;
  m_robot->calcInverseDynamics(m_robot->rootLink(), root_f, root_t);
  // if (loop % 200 == 0) {
  //   std::cerr << ":mass " << m_robot->totalMass() << std::endl;
  //   std::cerr << ":cog "; rats::print_vector(std::cerr, m_robot->calcCenterOfMass());
  //   for(int i = 0; i < m_robot->numJoints(); ++i){
  //     std::cerr << "(list :" << m_robot->link(i)->name() << " "
  //               << m_robot->joint(i)->jointId << " "
  //               << m_robot->link(i)->m << " ";
  //     Vector3 tmpc = m_robot->link(i)->p() + m_robot->link(i)->R() * m_robot->link(i)->c;
  //     rats::print_vector(std::cerr, tmpc, false);
  //     std::cerr << " ";
  //     rats::print_vector(std::cerr, m_robot->link(i)->c, false);
  //     std::cerr << ")" << std::endl;
  //   }
  // }
  // if (loop % 200 == 0) {
  //   std::cerr << ":IV1 (list ";
  //   for(int i = 0; i < m_robot->numJoints(); ++i){
  //     std::cerr << "(list :" << m_robot->joint(i)->name() << " " <<  m_robot->joint(i)->u << ")";
  //   }
  //   std::cerr << ")" << std::endl;
  // }
  MatrixXd contact_mat, contact_mat_inv;
  std::vector<Vector3> contact_p;
  for (size_t j = 0; j < 2; j++) contact_p.push_back(m_robot->sensor<hrp::ForceSensor>(sensor_names[j])->link->p());
  calcContactMatrix(contact_mat, contact_p);
  hrp::calcSRInverse(contact_mat, contact_mat_inv, 0.0);
  VectorXd root_ft(6);
  for (size_t j = 0; j < 3; j++) root_ft(j) = root_f(j);
  for (size_t j = 0; j < 3; j++) root_ft(j+3) = root_t(j);
  VectorXd contact_ft(2*6);
  contact_ft = contact_mat_inv * root_ft;
  // if (loop%200==0) {
  //   std::cerr << ":mass " << m_robot->totalMass() << std::endl;
  //   // std::cerr << ":ftv "; rats::print_vector(std::cerr, ftv);
  //   // std::cerr << ":aa "; rats::print_matrix(std::cerr, aa);
  //   // std::cerr << ":dv "; rats::print_vector(std::cerr, dv);
  // }
  for (size_t j = 0; j < 2; j++) {
    hrp::JointPathEx jm = hrp::JointPathEx(m_robot, m_robot->rootLink(), m_robot->sensor<hrp::ForceSensor>(sensor_names[j])->link);
    MatrixXd JJ;
    jm.calcJacobian(JJ);
    VectorXd ft(6);
    for (size_t i = 0; i < 6; i++) ft(i) = contact_ft(i+j*6);
    VectorXd tq_from_extft(jm.numJoints());
    tq_from_extft = JJ.transpose() * ft;
    // if (loop%200==0) {
    //   std::cerr << ":ft "; rats::print_vector(std::cerr, ft);
    //   std::cerr << ":JJ "; rats::print_matrix(std::cerr, JJ);
    //   std::cerr << ":tq_from_extft "; rats::print_vector(std::cerr, tq_from_extft);
    // }
    for (size_t i = 0; i < jm.numJoints(); i++) jm.joint(i)->u -= tq_from_extft(i);
  }
  //MatrixXd MM(6,m_robot->numJoints());
  //m_robot->calcMassMatrix(MM);
  // if (loop % 200 == 0) {
  //   std::cerr << ":INVDYN2 (list "; rats::print_vector(std::cerr, root_f, false);
  //   std::cerr << " "; rats::print_vector(std::cerr, root_t, false);
  //   std::cerr << ")" << std::endl;
  //   // VectorXd tqv(m_robot->numJoints());
  //   // for(int i = 0; i < m_robot->numJoints(); ++i){p
  //   //   tqv[m_robot->joint(i)->jointId] = m_robot->joint(i)->u;
  //   // }
  //   // std::cerr << ":IV2 "; rats::print_vector(std::cerr, tqv);
  //   std::cerr << ":IV2 (list ";
  //   for(int i = 0; i < m_robot->numJoints(); ++i){
  //     std::cerr << "(list :" << m_robot->joint(i)->name() << " " <<  m_robot->joint(i)->u << ")";
  //   }
  //   std::cerr << ")" << std::endl;
  // }
};
*/

void Stabilizer::setEefmParameters()
{
  eefm_pos_control_switch=eefm_rot_control_switch=eefm_att_control_switch=1;
  //ref_zmp_aux = Vector3::Zero();


  RTC::Properties& prop = getProperties();

  std::vector<double> prop_eefm_k;
  coil::stringTo(prop_eefm_k, prop["eefm_k"].c_str());
  eefm_k1[0] = eefm_k1[1] = prop_eefm_k[0];
  eefm_k2[0] = eefm_k2[1] = prop_eefm_k[1];
  eefm_k3[0] = eefm_k3[1] = prop_eefm_k[2];
 
  double prop_eefm_zmp;
  coil::stringTo(prop_eefm_zmp, prop["eefm_zmp"].c_str());
  eefm_zmp_delay_time_const[0] = eefm_zmp_delay_time_const[1] = prop_eefm_zmp;

  std::vector<double> prop_eefm_rot;
  coil::stringTo(prop_eefm_rot, prop["eefm_rot"].c_str());
  eefm_rot_damping_gain[0] = prop_eefm_rot[0];
  eefm_rot_damping_gain[1] = prop_eefm_rot[1];
  eefm_rot_time_const      = prop_eefm_rot[2];

  std::vector<double> prop_eefm_pos;
  coil::stringTo(prop_eefm_pos, prop["eefm_pos"].c_str());
  eefm_pos_damping_gain       = prop_eefm_pos[0];
  eefm_pos_time_const_support = prop_eefm_pos[1];
  eefm_pos_time_const_swing   = prop_eefm_pos[2];
  eefm_pos_transition_time    = prop_eefm_pos[3];//unused
  eefm_pos_margin_time        = prop_eefm_pos[4];//unused

  std::vector<double> prop_eefm_leg;
  coil::stringTo(prop_eefm_leg, prop["eefm_leg"].c_str());
  eefm_leg_inside_margin = prop_eefm_leg[0];
  eefm_leg_front_margin  = prop_eefm_leg[1];
  eefm_leg_rear_margin   = prop_eefm_leg[2];

  std::vector<double> prop_eefm_body;
  coil::stringTo(prop_eefm_body, prop["eefm_body"].c_str());
  eefm_body_attitude_control_gain[0] = eefm_body_attitude_control_gain[1] = prop_eefm_body[0];
  eefm_body_attitude_control_time_const[0] = eefm_body_attitude_control_time_const[1] = prop_eefm_body[1];

  coil::stringTo(eefm_cogvel_cutoff_freq, prop["eefm_cog"].c_str());


  std::cout << "Stabilizer EEFM parameters" << std::endl
	    << "eefm zmp :" << std::endl 
	    << "  delay zmp = " << eefm_zmp_delay_time_const[0] << ", " << eefm_zmp_delay_time_const[1] << std::endl
	    << "eefm k :" << std::endl 
	    << "  k gain = " << eefm_k1[0] << ", " << eefm_k2[0] << ", " << eefm_k3[0] << std::endl 
	    << "eefm rot :" << std::endl
	    << "  damp gain = " << eefm_rot_damping_gain[0] << ", " << eefm_rot_damping_gain[1] << std::endl
	    << "  time const = " << eefm_rot_time_const << std::endl
	    << "eefm pos :" << std::endl
	    << "  damp gain = " << eefm_pos_damping_gain << std::endl
	    << "  time const sup = " << eefm_pos_time_const_support << std::endl
	    << "  time const swg = " << eefm_pos_time_const_swing << std::endl
	    << "  transition time = " << eefm_pos_transition_time << std::endl
	    << "  margin time = " << eefm_pos_margin_time << std::endl
	    << "eefm leg :" << std::endl
	    << "  inside = " << eefm_leg_inside_margin << std::endl
	    << "  front = " << eefm_leg_front_margin << std::endl
	    << "  rear = " << eefm_leg_rear_margin << std::endl
	    << "eefm body :" << std::endl
	    << "  gain = " << eefm_body_attitude_control_gain[0] << std::endl
	    << "  time const = " << eefm_body_attitude_control_time_const[0] << std::endl
	    << "eefm cog : " << std::endl
	    << "  cutoff freq = " << eefm_cogvel_cutoff_freq << std::endl
	    << std::endl;
  prev_act_cog    = Vector3::Zero();
  prev_act_cogvel = Vector3::Zero();
  prev_ref_cog    = Vector3::Zero();
  prev_ref_zmp    = Vector3::Zero();
  prev_act_foot_origin_rot = Matrix3::Identity();
  prev_ref_foot_origin_rot = Matrix3::Identity();
}


extern "C"
{

  void StabilizerInit(RTC::Manager* manager)
  {
    RTC::Properties profile(stabilizer_spec);
    manager->registerFactory(profile,
                             RTC::Create<Stabilizer>,
                             RTC::Delete<Stabilizer>);
  }

};


