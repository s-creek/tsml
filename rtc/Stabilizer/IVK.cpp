#include "IVK.h"

void RenewModel(BodyPtr body,Vector3  *p_now, Matrix3 *R_now, string *end_link)
{
  /*
  string RARM_END,LARM_END;
  if( body->numJoints()==32){
    RARM_END= "RARM_JOINT6";
    LARM_END= "LARM_JOINT6";
  }
  else if( body->numJoints()==30){
    RARM_END= "RARM_JOINT5";
    LARM_END= "LARM_JOINT5";
  }

  p_now[0]=body->link("RLEG_JOINT5")->p();
  p_now[1]=body->link("LLEG_JOINT5")->p();
  p_now[2]=body->link(RARM_END)->p();
  p_now[3]=body->link(LARM_END)->p();
  p_now[4]=body->link("WAIST")->p();
 
  R_now[0]=body->link("RLEG_JOINT5")->R();
  R_now[1]=body->link("LLEG_JOINT5")->R();
  R_now[2]=body->link(RARM_END)->R();
  R_now[3]=body->link(LARM_END)->R();
  R_now[4]=body->link("WAIST")->R();
  */
  ////new///
  p_now[0]=body->link(end_link[RLEG])->p();
  p_now[1]=body->link(end_link[LLEG])->p();
  p_now[2]=body->link(end_link[RARM])->p();
  p_now[3]=body->link(end_link[LARM])->p();
  p_now[4]=body->link(end_link[WAIST])->p();
 
  R_now[0]=body->link(end_link[RLEG])->R();
  R_now[1]=body->link(end_link[LLEG])->R();
  R_now[2]=body->link(end_link[RARM])->R();
  R_now[3]=body->link(end_link[LARM])->R();
  R_now[4]=body->link(end_link[WAIST])->R();
}

bool CalcIVK_biped(BodyPtr body,  Matrix3 ref_root_R, TimedBooleanSeq m_contactStates,  string *end_link)
{
  Link* SupLeg;
  Link* SwLeg;
  JointPathPtr SupLeg2SwLeg,SupLeg2W;
  Vector3 cm=body->calcCenterOfMass();
  Vector3 ref_SwLeg_p; 
  Matrix3 ref_SwLeg_R;  
  Vector3 ref_SupLeg_p; 
  Matrix3 ref_SupLeg_R;  
  FootType FT;

  if(m_contactStates.data[0]==0)
    FT=RFsw;
  else
    FT=LFsw;

  Eigen::ColPivHouseholderQR<MatrixXd> QR;
  //careful
  MatrixXd Jacobian=MatrixXd::Zero(12,12);//leg only
 
  if((FT==FSRFsw)||FT==RFsw){
    SupLeg=body->link(end_link[LLEG]);
    SwLeg=body->link(end_link[RLEG]);
  }
  else if((FT==FSLFsw)||FT==LFsw){
    SupLeg=body->link(end_link[RLEG]);
    SwLeg=body->link(end_link[LLEG]);
  }
  ref_SwLeg_p=SwLeg->p(); 
  ref_SwLeg_R=SwLeg->R(); 
  ref_SupLeg_p=SupLeg->p(); 
  ref_SupLeg_R=SupLeg->R(); 
  SupLeg2SwLeg= getCustomJointPath(body, SupLeg, SwLeg);  
  SupLeg2W= getCustomJointPath(body, SupLeg,body->link(end_link[WAIST]));   
  
  static const int MAX_IK_ITERATION = 50;
  static const double LAMBDA = 0.9;
  
  const int n = 12;//leg only
  
  //Link* target = jpp->endLink();
  
  std::vector<double> qorg(n);
    for(int i=0; i < 12; ++i){
      qorg[i] = body->joint(i)->q();
    }
    
    //careful
    VectorXd dq(n);
    VectorXd v(12);
    double maxIKErrorSqr=1.0e-16;
    bool converged = false;
  
    for(int i=0; i < MAX_IK_ITERATION; i++){
      
      Matrix3 W_R=body->link(end_link[WAIST])->R();
      Vector3 SwLeg_p = SwLeg->p();
      Matrix3 SwLeg_R = SwLeg->R();
      CalJo_biped(body, FT, Jacobian, end_link);        
      
      Vector3 CM_dp=cm - body->calcCenterOfMass();
      Vector3 W_omega=W_R* omegaFromRot(W_R.transpose() * ref_root_R);
      Vector3 SW_dp = ref_SwLeg_p - SwLeg_p;  
      Vector3 SW_omega = SwLeg_R* omegaFromRot(SwLeg_R.transpose() * ref_SwLeg_R);
     
      //v<< CM_dp, W_omega, SW_dp, SW_omega;
      v.head<3>()=CM_dp;
      v.segment<3>(3)=W_omega;
      v.segment<3>(6)=SW_dp;
      v.segment<3>(9)=SW_omega;

      //double errsqr =  CM_dp.dot(CM_dp) + W_omega.dot(W_omega) + SW_dp.dot(SW_dp) + SW_omega.dot( SW_omega);
      double errsqr=v.squaredNorm();

      if(errsqr < maxIKErrorSqr){
	converged = true;
	break;
      }
      
      MatrixXd JJ;
      double dampingConstantSqr=1e-12;
      JJ = Jacobian *  Jacobian.transpose() + dampingConstantSqr * MatrixXd::Identity(Jacobian.rows(), Jacobian.rows());
      dq = Jacobian.transpose() * QR.compute(JJ).solve(v);


      for(int j=0; j < n; ++j){
	body->joint(j)->q() += LAMBDA * dq(j);
      }

      SupLeg->p()=ref_SupLeg_p;
      SupLeg->R()=ref_SupLeg_R;
      SupLeg2SwLeg->calcForwardKinematics();
      body->calcForwardKinematics();
      
    }//for
    
    if(!converged){
      
      for(int j=0; j < n; ++j)
	body->joint(j)->q() = qorg[j];
         
      SupLeg->p()=ref_SupLeg_p;
      SupLeg->R()=ref_SupLeg_R;
      SupLeg2SwLeg->calcForwardKinematics();
      body->calcForwardKinematics();
    }
    
    return converged;          
}


//////ivk_jacobian/////////
void CalJo_biped(BodyPtr body, FootType FT, Eigen::MatrixXd& out_J,  string *end_link)
{ 
  Link* SupLeg;
  Link* SwLeg;
  JointPathPtr  SupLeg2SwLeg,SupLeg2W;
  MatrixXd JSupLeg2SwLeg ,JSupLeg2W,Jcom;

  
  if((FT==FSRFsw)||FT==RFsw){
    SupLeg=body->link(end_link[LLEG]);
    SwLeg=body->link(end_link[RLEG]);
  }
  else if((FT==FSLFsw)||FT==LFsw){
    SupLeg=body->link(end_link[RLEG]);
    SwLeg=body->link(end_link[LLEG]);
  }
  SupLeg2SwLeg= getCustomJointPath(body, SupLeg, SwLeg);  
  SupLeg2W= getCustomJointPath(body, SupLeg,body->link(end_link[WAIST]));   

  body->calcCenterOfMass();
  calcCMJacobian(body, SupLeg, Jcom);

  SupLeg2SwLeg->calcJacobian(JSupLeg2SwLeg);
  SupLeg2W->calcJacobian(JSupLeg2W);
  
  
  //push in
  for(int i=0;i<3;i++){
    for(int j=0;j<12;j++){
      out_J(i,j)=Jcom(i,j);
    }
  }
  
  // SupLeg2W
  for(int i = 0; i <  SupLeg2W->numJoints(); i++) {
    int id =  SupLeg2W->joint(i)->jointId();
    for(int j=3;j<6;j++){
      out_J(j,id)=JSupLeg2W(j,i);
    }
  }
  
  //SupLeg2SwLeg
  for(int i = 0; i < SupLeg2SwLeg->numJoints(); i++) {
    int id = SupLeg2SwLeg->joint(i)->jointId();
    for(int j=6;j<12;j++){
      out_J(j,id)=JSupLeg2SwLeg(j-6,i);
    }
  }
  
}
