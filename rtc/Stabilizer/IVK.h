#ifndef INVK_H
#define INVK_H 

#define NEAR0 1e-8

#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <rtm/CorbaNaming.h>

#include<stdio.h>
#include<fstream>
#include<math.h>
#include <iostream>
#include <deque>

#include <cnoid/Body>
#include<cnoid/BodyLoader>
#include<cnoid/VRMLBodyLoader>
#include <cnoid/EigenTypes>
#include <cnoid/JointPath>
#include <cnoid/Jacobian>
#include <cnoid/EigenUtil>
#include <cnoid/Sensor>
#include <cnoid/Link>

using namespace std;
using namespace cnoid;
using namespace RTC;
using namespace Eigen;

enum FootType {FSRFsw, FSLFsw, RFsw, LFsw};
enum{RLEG, LLEG, RARM, LARM, WAIST, LINKNUM};

void RenewModel(BodyPtr body,Vector3  *p_now, Matrix3 *R_now, string *end_link);

bool CalcIVK_biped(BodyPtr body,  Matrix3 ref_root_R, TimedBooleanSeq m_contactStates, string *end_link);
void CalJo_biped(BodyPtr body, FootType FT, Eigen::MatrixXd& out_J, string *end_link);


#endif
