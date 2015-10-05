// -*- C++ -*-

#include "creekArmControlCartesian.h"

#include <cnoid/BodyLoader>
#include <cnoid/Link>
#include <cnoid/EigenUtil>

#include "VectorConvert.h"

Eigen::IOFormat vecio(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]");
Eigen::IOFormat rotio(Eigen::StreamPrecision, 0, ", ", "]\n", "", "", "[", "]");

// Module specification
static const char* creekarmcontrolcartesian_spec[] =
  {
    "implementation_id", "creekArmControlCartesian",
    "type_name",         "creekArmControlCartesian",
    "description",       "creekArmControlCartesian",
    "version",           "1.0",
    "vendor",            "s-creek",
    "category",          "example",
    "activity_type",     "SPORADIC",
    "kind",              "creekArmControlCartesian",
    "max_instance",      "0",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    ""
  };

creekArmControlCartesian::creekArmControlCartesian(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    m_qCurIn("qCur", m_qCur),
    m_basePosIn("basePos", m_basePos),
    m_baseRpyIn("baseRpy", m_baseRpy),
    m_axesIn("axes", m_axes),
    m_buttonsIn("buttons", m_buttons),
    m_qRefOut("qRef", m_qRef),
    m_creekArmControlCartesianServicePort("creekArmControlCartesianService")
{
  m_service0.setComponent(this);

  m_dtrans = 0.0002;
  m_domega = 0.001;
  m_delbow = 0.001;
}

creekArmControlCartesian::~creekArmControlCartesian()
{
}


RTC::ReturnCode_t creekArmControlCartesian::onInitialize()
{
  addInPort("qCur", m_qCurIn);
  addInPort("basePos", m_basePosIn);
  addInPort("baseRpy", m_baseRpyIn);

  addInPort("axes", m_axesIn);
  addInPort("buttons", m_buttonsIn);

  addOutPort("qRef", m_qRefOut);

  m_creekArmControlCartesianServicePort.registerProvider("service0", "creekArmControlCartesianService", m_service0);
  addPort(m_creekArmControlCartesianServicePort);


  //
  // init model
  //
  RTC::Properties& prop = getProperties();

  cnoid::BodyLoader bl;
  m_robot = bl.load( prop["model"].c_str());

  std::vector<double> tmp;
  coil::stringTo(tmp, prop["initBasePos"].c_str());
  m_robot->rootLink()->p() << tmp[0], tmp[1], tmp[2];

  tmp.clear();
  coil::stringTo(tmp, prop["initBaseRpy"].c_str());
  m_robot->rootLink()->R() = cnoid::rotFromRpy(tmp[0], tmp[1], tmp[2]);

  m_robot->calcForwardKinematics();

  // m_rarm = getCustomJointPath(m_robot, m_robot->link("WAIST_R"), m_robot->link("R_WRIST_Y"));
  // m_larm = getCustomJointPath(m_robot, m_robot->link("WAIST_R"), m_robot->link("L_WRIST_Y"));
  m_rarm = getCustomJointPath(m_robot, m_robot->link(prop["RARM_BASE"]), m_robot->link(prop["RARM_END"]));
  m_larm = getCustomJointPath(m_robot, m_robot->link(prop["LARM_BASE"]), m_robot->link(prop["LARM_END"]));


  //
  // set velocity
  //
  double trans, omega, elbow;
  coil::stringTo(m_dt, prop["dt"].c_str());
  coil::stringTo(trans, prop["arm.trans_velocity"].c_str());
  coil::stringTo(omega, prop["arm.angular_velocity"].c_str());
  coil::stringTo(elbow, prop["arm.elbow_velocity"].c_str());

  m_dtrans = m_dt * trans;
  m_domega = m_dt * omega;
  m_delbow = m_dt * elbow;


  //
  // init data port
  //
  int dof = m_robot->numJoints();
  m_qRef.data.length(dof);
  m_qCur.data.length(dof);

  m_basePos.data.x = m_robot->rootLink()->p()(0);
  m_basePos.data.y = m_robot->rootLink()->p()(1);
  m_basePos.data.z = m_robot->rootLink()->p()(2);

  m_baseRpy.data.r = tmp[0];
  m_baseRpy.data.p = tmp[1];
  m_baseRpy.data.y = tmp[2];

  m_axes.data.length(29);
  m_buttons.data.length(17);


  std::cout << "creekArmControlCartesian : onInitialize" << std::endl;
  return RTC::RTC_OK;
}


RTC::ReturnCode_t creekArmControlCartesian::onActivated(RTC::UniqueId ec_id)
{
  m_activeArmId = 2;
  m_active = false;

  std::cout << "creekArmControlCartesian : onActivated" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t creekArmControlCartesian::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t creekArmControlCartesian::onExecute(RTC::UniqueId ec_id)
{
  if( m_basePosIn.isNew() )  m_basePosIn.read();
  if( m_baseRpyIn.isNew() )  m_baseRpyIn.read();
  if( m_qCurIn.isNew() )     m_qCurIn.read();
  
  bool update(false);
  if( m_axesIn.isNew() || m_buttonsIn.isNew() ) {
    m_axesIn.read();
    m_buttonsIn.read();

    cnoid::Vector3 vec(-m_axes.data[1], -m_axes.data[0], m_buttons.data[8]-m_buttons.data[10]);  // left stick & L1,L2
    cnoid::Vector3 omg(-m_axes.data[2], m_axes.data[3], m_buttons.data[9]-m_buttons.data[11]);   // right stick & R1,R2
    double elbow = m_buttons.data[4]-m_buttons.data[6];     // top,bottom botton
    double elbowFix = m_buttons.data[5]+m_buttons.data[7];  // left,right button
    
    if( vec.norm() > 0.5 || omg.norm() > 0.5 || fabs(elbow) > 0.5 || fabs(elbowFix) > 0.5 ) {
      bool use_elbow(false);
      if( fabs(elbowFix) > 0.5 ) {
	use_elbow = true;
	elbow = 0.0;
      }
      else if( fabs(elbow) > 0.5) {
	use_elbow = true;
      }

      switch ( m_activeArmId )
	{
	case 0:
	  {
	    if( !m_active )
	      initModel();

	    cnoid::Vector3 refP;
	    cnoid::Matrix3 refR;
	    double refE;
	    // refP = m_rarm->endLink()->p() + m_rarm->endLink()->R() * (0.00005 * vec);
	    // refR = m_rarm->endLink()->R() * cnoid::rotFromRpy(0.005 * omg);
	    refP = m_rarmRef.translation() + m_rarmRef.linear() * (m_dtrans * vec);
	    refR = m_rarmRef.linear() * cnoid::rotFromRpy(m_domega * omg);
	    refE = m_relbowRef + m_delbow * elbow;
	    
	    if( use_elbow ) {
	      if( calcElbowInverseKinematics(m_rarm, refP, refR, refE) ) {
		update = true;
		m_rarmRef.translation() = refP;
		m_rarmRef.linear() = refR;
		m_relbowRef = refE;
	      }
	      else {
		std::cout << "arm : rarm elbow error" << std::endl;
	      }
	    }
	    else if( calcInverseKinematics(m_rarm, refP, refR) ) {
	      update = true;
	      m_rarmRef.translation() = refP;
	      m_rarmRef.linear() = refR;
	      m_relbowRef = getElbowAngle(m_rarm);
	    }
	    else if( m_rarm->calcInverseKinematics(refP, refR) ) {
	      update = true;
	      m_rarmRef.translation() = refP;
	      m_rarmRef.linear() = refR;
	      m_relbowRef = getElbowAngle(m_rarm);
	    }
	    else {
	      // std::cout << "rarm inv error" << std::endl;
	      // std::cout << refP.format(vecio) << std::endl;
	      // std::cout << refR.format(rotio) << std::endl;
	    }

	    break;
	  }
	case 1:
	  {
	    if( !m_active )
	      initModel();

	    cnoid::Vector3 refP;
	    cnoid::Matrix3 refR;
	    double refE;
	    // refP = m_larm->endLink()->p() + m_larm->endLink()->R() * (0.00005 * vec);
	    // refR = m_larm->endLink()->R() * cnoid::rotFromRpy(0.005 * omg);
	    refP = m_larmRef.translation() + m_larmRef.linear() * (m_dtrans * vec);
	    refR = m_larmRef.linear() * cnoid::rotFromRpy(m_domega * omg);
	    refE = m_lelbowRef + m_delbow * elbow;

	    if( use_elbow ) {
	      if( calcElbowInverseKinematics(m_larm, refP, refR, refE) ) {
		update = true;
		m_larmRef.translation() = refP;
		m_larmRef.linear() = refR;
		m_lelbowRef = refE;
	      }
	      else {
		std::cout << "arm : larm elbow error" << std::endl;
	      }
	    }
	    else if( calcInverseKinematics(m_larm, refP, refR) ) {
	      update = true;
	      m_larmRef.translation() = refP;
	      m_larmRef.linear() = refR;
	      m_lelbowRef = getElbowAngle(m_larm);
	    }
	    else if( m_larm->calcInverseKinematics(refP, refR) ) {
	      update = true;
	      m_larmRef.translation() = refP;
	      m_larmRef.linear() = refR;
	      m_lelbowRef = getElbowAngle(m_larm);
	    }
	    else {
	      // std::cout << "larm inv error" << std::endl;
	      // std::cout << refP.format(vecio) << std::endl;
	      // std::cout << refR.format(rotio) << std::endl;
	    }

	    break;
	  }
	default:
	  update = false;
	};
    }
  }


  if( update ) {
    //std::cout << "arm : update" << std::endl;
    m_active = true;

    for(int i=0; i<m_robot->numJoints(); i++) {
      m_qRef.data[i] = m_robot->joint(i)->q();
    }
    m_qRefOut.write();
  }
  else {
    m_active = false;
  }
  

  return RTC::RTC_OK;
}


void creekArmControlCartesian::setArm(int armId)
{
  initModel();
  m_activeArmId = armId;
  std::cout << "active arm = " << m_activeArmId << std::endl;
}


void creekArmControlCartesian::initModel()
{
  //std::cout << "arm : init model" << std::endl;

  m_robot->rootLink()->p() << m_basePos.data.x, m_basePos.data.y, m_basePos.data.z;
  m_robot->rootLink()->R() = cnoid::rotFromRpy(m_baseRpy.data.r, m_baseRpy.data.p, m_baseRpy.data.y);
  for(int i=0; i<m_robot->numJoints(); i++) {
    m_robot->joint(i)->q() = m_qCur.data[i];
  }
  m_robot->calcForwardKinematics();

  m_rarmRef = m_rarm->endLink()->position();
  m_larmRef = m_larm->endLink()->position();

  m_relbowRef = getElbowAngle(m_rarm);
  m_lelbowRef = getElbowAngle(m_larm);
}

typedef Eigen::MatrixXd dmatrix;
typedef Eigen::VectorXd dvector;
bool creekArmControlCartesian::calcInverseKinematics(cnoid::JointPathPtr arm, cnoid::Vector3 &refP, cnoid::Matrix3 &refR)
{
  int n = arm->numJoints();
  double qorg[n];
  for(int i=0; i<n; i++)
    qorg[i] = arm->joint(i)->q();


  static const int MAX_IK_ITERATION = 50;
  static const double LAMBDA = 0.9;


  dmatrix J(6,n), JJ;
  dvector v(6);
  dvector dq(n);

  Eigen::ColPivHouseholderQR<dmatrix> QR;
  double dampingConstantSqr=1e-12;

  double maxIKErrorSqr = 1.0e-16;
  double errsqr = maxIKErrorSqr * 100.0;
  bool converged = false;
  

  for(int k=0; k < MAX_IK_ITERATION; k++) {

    cnoid::Vector3 vel( refP-arm->endLink()->p() );
    cnoid::Vector3 omg( arm->endLink()->R() * cnoid::omegaFromRot( cnoid::Matrix3(arm->endLink()->R().transpose() * refR) ) );
    v.head<3>()     = vel;
    v.segment<3>(3) = omg;


    errsqr = v.squaredNorm();
    if( errsqr < maxIKErrorSqr ) {
      converged = true;
      //std::cout << "it num = " << k << std::endl;
      break;
    }
    else if( MAX_IK_ITERATION == 1) {
      converged = true;
    }


    arm->calcJacobian(J);
    JJ = J *  J.transpose() + dampingConstantSqr * dmatrix::Identity(J.rows(), J.rows());
    dq = J.transpose() * QR.compute(JJ).solve(v);
    for(int i=0; i<n; i++)
      arm->joint(i)->q() += LAMBDA * dq(i);
    arm->calcForwardKinematics();
  }


  if( !converged ) {
    for(int i=0; i<n; i++)
      arm->joint(i)->q() = qorg[i];
    arm->calcForwardKinematics();
  }


  return converged;
}


bool creekArmControlCartesian::calcElbowInverseKinematics(cnoid::JointPathPtr arm, cnoid::Vector3 &refP, cnoid::Matrix3 &refR, double refElbow)
{
  int n = arm->numJoints();
  if( n != 7 )
    return false;


  double qorg[n];
  for(int i=0; i<n; i++)
    qorg[i] = arm->joint(i)->q();


  static const int MAX_IK_ITERATION = 50;
  static const double LAMBDA = 0.9;


  dmatrix J(7,n), JJ;
  dvector v(7);
  dvector dq(n);

  Eigen::ColPivHouseholderQR<dmatrix> QR;
  double dampingConstantSqr=1e-12;

  double maxIKErrorSqr = 1.0e-16;
  double errsqr = maxIKErrorSqr * 100.0;
  bool converged = false;
  

  for(int k=0; k < MAX_IK_ITERATION; k++) {

    cnoid::Vector3 vel( refP-arm->endLink()->p() );
    cnoid::Vector3 omg( arm->endLink()->R() * cnoid::omegaFromRot( cnoid::Matrix3(arm->endLink()->R().transpose() * refR) ) );
    double dElbow = refElbow - getElbowAngle(arm);
    while( dElbow > M_PI || dElbow < -M_PI ) {
      if(dElbow >  M_PI)  dElbow -= (2*M_PI);
      if(dElbow < -M_PI)  dElbow += (2*M_PI);
      std::cout << "arm : fix delbow = " << dElbow << std::endl;
    }


    v.head<3>()     = vel;
    v.segment<3>(3) = omg;
    v(6)            = dElbow;


    errsqr = v.squaredNorm();
    if( errsqr < maxIKErrorSqr ) {
      converged = true;
      //std::cout << "it num = " << k << std::endl;
      break;
    }
    else if( MAX_IK_ITERATION == 1) {
      converged = true;
    }


    calcElbowJacobian(arm, J);
    JJ = J *  J.transpose() + dampingConstantSqr * dmatrix::Identity(J.rows(), J.rows());
    dq = J.transpose() * QR.compute(JJ).solve(v);
    for(int i=0; i<n; i++)
      arm->joint(i)->q() += LAMBDA * dq(i);
    arm->calcForwardKinematics();
  }


  if( !converged ) {
    for(int i=0; i<n; i++)
      arm->joint(i)->q() = qorg[i];
    arm->calcForwardKinematics();
  }


  return converged;
}


double creekArmControlCartesian::getElbowAngle(cnoid::JointPathPtr arm)
{
  // set links
  cnoid::Link *wrist    = arm->endLink();
  cnoid::Link *elbow    = arm->joint(3);
  cnoid::Link *shoulder = arm->joint(0);
  cnoid::Link *base     = arm->baseLink();


  // main
  cnoid::Vector3 w(wrist->p()-shoulder->p());
  cnoid::Vector3 e(elbow->p()-shoulder->p());
  cnoid::Vector3 hw(w);  hw.normalize();
  cnoid::Vector3 hv(hw.cross(cnoid::Vector3(0,-1,0)));  hv.normalize();
  cnoid::Vector3 d(hw*(hw.dot(e)));
  cnoid::Vector3 p(e-d);

  double at0, at1;
  at0 = hw.dot( hv.cross(p) );
  at1 = hv.dot(p);
  return atan2(at0, at1);
}


void creekArmControlCartesian::calcElbowJacobian(cnoid::JointPathPtr arm, dmatrix &out_J)
{
  int n = arm->numJoints();
  

  // set links
  cnoid::Link *wrist    = arm->endLink();
  cnoid::Link *elbow    = arm->joint(3);
  cnoid::Link *shoulder = arm->joint(0);
  cnoid::Link *base     = arm->baseLink();

  cnoid::JointPathPtr base2wrist    = getCustomJointPath(m_robot, base, wrist);
  cnoid::JointPathPtr base2elbow    = getCustomJointPath(m_robot, base, elbow);
  cnoid::JointPathPtr base2shoulder = getCustomJointPath(m_robot, base, shoulder);

  dmatrix Jwrist, Jelbow, Jshoulder;
  base2wrist->calcJacobian(Jwrist);
  base2elbow->calcJacobian(Jelbow);
  base2shoulder->calcJacobian(Jshoulder);


  // translational velocity
  dmatrix Je(3,n), Jw(3,n);
  Je = dmatrix::Zero(3,n);
  Jw = dmatrix::Zero(3,n);

  for(int i=0; i<3; i++) {
    for(int j=0; j<base2elbow->numJoints(); j++)     Je(i,j) += Jelbow(i,j);
    for(int j=0; j<base2shoulder->numJoints(); j++)  Je(i,j) -= Jshoulder(i,j);

    for(int j=0; j<base2wrist->numJoints(); j++)     Jw(i,j) += Jwrist(i,j);
    for(int j=0; j<base2shoulder->numJoints(); j++)  Jw(i,j) -= Jshoulder(i,j);
  }


  // main
  cnoid::Vector3 w(wrist->p()-shoulder->p());
  cnoid::Vector3 e(elbow->p()-shoulder->p());
  cnoid::Vector3 hw(w);  hw.normalize();
  cnoid::Vector3 he(e);  he.normalize();
  cnoid::Vector3 hv(hw.cross(cnoid::Vector3(0,-1,0)));  hv.normalize();
  cnoid::Vector3 d(hw*(hw.dot(e)));
  cnoid::Vector3 p(e-d);
  cnoid::Vector3 l( cnoid::Vector3(w.cross(hv)).cross(w) );
  cnoid::Vector3 hp(p);  hp.normalize();
  cnoid::Vector3 hl(l);  hl.normalize();

  cnoid::Vector3 Je_coefficient, Jw_coefficient;
  Je_coefficient = hw.cross(hp) / p.norm();
  Jw_coefficient = ( (hv.dot(w) / l.norm()) * hw.cross(hl) ) - ( (hw.dot(he) / p.norm()) * hw.cross(hp) );

  dvector Jpsi(n);
  Jpsi = Je_coefficient.transpose() * Je + Jw_coefficient.transpose() * Jw;

  dmatrix Jarm;
  arm->calcJacobian(Jarm);

  out_J.resize(7,n);
  for(int i=0; i<n; i++) {
    for(int j=0; j<6; j++) {
      out_J(j,i) = Jarm(j,i);
    }
    out_J(6,i) = Jpsi(i);
  }
}


void creekArmControlCartesian::setTranslationVelocity(double vtrans)
{
  double v = fabs(vtrans);
  if( v > 0.4 )
    v = 0.4;
  m_dtrans = m_dt * v;
}


void creekArmControlCartesian::setAngularVelocity(double vomega)
{
  double v = fabs(vomega);
  if( v > 2.0 )
    v = 2.0;
  m_domega = m_dt * v;
}


void creekArmControlCartesian::setElbowVelocity(double velbow)
{
  double v = fabs(velbow);
  if( v > 2.0 )
    v = 2.0;
  m_delbow = m_dt * v;
}


extern "C"
{
 
  void creekArmControlCartesianInit(RTC::Manager* manager)
  {
    coil::Properties profile(creekarmcontrolcartesian_spec);
    manager->registerFactory(profile,
                             RTC::Create<creekArmControlCartesian>,
                             RTC::Delete<creekArmControlCartesian>);
  }
  
};



