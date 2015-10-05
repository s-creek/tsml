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

  m_rarm = getCustomJointPath(m_robot, m_robot->link("WAIST_R"), m_robot->link("R_WRIST_Y"));
  m_larm = getCustomJointPath(m_robot, m_robot->link("WAIST_R"), m_robot->link("L_WRIST_Y"));


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

    cnoid::Vector3 vec(-m_axes.data[1], -m_axes.data[0], m_buttons.data[8]-m_buttons.data[10]);
    cnoid::Vector3 omg(-m_axes.data[2], m_axes.data[3], m_buttons.data[9]-m_buttons.data[11]);

    if( vec.norm() > 0.5 || omg.norm() > 0.5 ) {
      switch ( m_activeArmId )
	{
	case 0:
	  {
	    if( !m_active )
	      initModel();

	    cnoid::Vector3 refP;
	    cnoid::Matrix3 refR;
	    // refP = m_rarm->endLink()->p() + m_rarm->endLink()->R() * (0.00005 * vec);
	    // refR = m_rarm->endLink()->R() * cnoid::rotFromRpy(0.005 * omg);
	    refP = m_rarmRef.translation() + m_rarmRef.linear() * (0.000075 * vec);
	    refR = m_rarmRef.linear() * cnoid::rotFromRpy(0.001 * omg);

	    if( calcInverseKinematics(m_rarm, refP, refR) ) {
	      update = true;
	      m_rarmRef.translation() = refP;
	      m_rarmRef.linear() = refR;
	    }
	    else if( m_rarm->calcInverseKinematics(refP, refR) ) {
	      update = true;
	      m_rarmRef.translation() = refP;
	      m_rarmRef.linear() = refR;
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
	    // refP = m_larm->endLink()->p() + m_larm->endLink()->R() * (0.00005 * vec);
	    // refR = m_larm->endLink()->R() * cnoid::rotFromRpy(0.005 * omg);
	    refP = m_larmRef.translation() + m_larmRef.linear() * (0.000075 * vec);
	    refR = m_larmRef.linear() * cnoid::rotFromRpy(0.001 * omg);

	    if( calcInverseKinematics(m_larm, refP, refR) ) {
	      update = true;
	      m_larmRef.translation() = refP;
	      m_larmRef.linear() = refR;
	    }
	    else if( m_larm->calcInverseKinematics(refP, refR) ) {
	      update = true;
	      m_larmRef.translation() = refP;
	      m_larmRef.linear() = refR;
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
  std::cout << "arm : init model" << std::endl;

  m_robot->rootLink()->p() << m_basePos.data.x, m_basePos.data.y, m_basePos.data.z;
  m_robot->rootLink()->R() = cnoid::rotFromRpy(m_baseRpy.data.r, m_baseRpy.data.p, m_baseRpy.data.y);
  for(int i=0; i<m_robot->numJoints(); i++) {
    m_robot->joint(i)->q() = m_qCur.data[i];
  }
  m_robot->calcForwardKinematics();

  m_rarmRef = m_rarm->endLink()->position();
  m_larmRef = m_larm->endLink()->position();
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



