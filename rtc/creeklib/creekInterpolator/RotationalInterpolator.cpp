#include "RotationalInterpolator.h"
#include <creekUtil/creekFuncs.h>
#include <cstring> // memcpy

using namespace creek;

RotationalInterpolator::RotationalInterpolator(double in_dt, InterpolationType in_itype) :
  InterpolatorBase(1, in_dt, in_itype)
{
  axis_a  = axis_b  = Vector3(1,0,0);
  angle_a = angle_b = 0.0;
  R_base  = tvmet::identity<Matrix33>();
}


void RotationalInterpolator::get(Matrix33 &out_val, bool popp)
{
  if( !m_seq.empty() ) {
    double *&tmp = m_seq.front();
    memcpy(m_last_out, tmp, m_memsize);

    if(popp) {
	  InterpolatorBase::pop();
    }
  }

  double variation = m_last_out[0];
  
  double   angle_a_tmp, angle_b_tmp;
  Matrix33 R_a_tmp, R_b_tmp;

  angle_a_tmp = variation * angle_a;
  angle_b_tmp = variation * angle_b;
  
  R_a_tmp = rodrigues(axis_a, angle_a_tmp);
  R_b_tmp = rodrigues(axis_b, angle_b_tmp);
  out_val = R_base * R_a_tmp * R_b_tmp;
}


const Matrix33 RotationalInterpolator::at(int index)
{
  double variation = m_seq.at(index)[0];

  double   angle_a_tmp, angle_b_tmp;
  Matrix33 R_a_tmp, R_b_tmp;

  angle_a_tmp = variation * angle_a;
  angle_b_tmp = variation * angle_b;
  
  R_a_tmp = rodrigues(axis_a, angle_a_tmp);
  R_b_tmp = rodrigues(axis_b, angle_b_tmp);

  const Matrix33 out_val(R_base * R_a_tmp * R_b_tmp);
  
  return out_val;
}


bool RotationalInterpolator::calcInterpolation(const Matrix33& R_cur, const Matrix33& R_ref, const Vector3& axis, double in_time, double in_2_delta)
{
  if(!empty()) clear();

  
  double xs = 0, xf = 1;
  if( !InterpolatorBase::calcInterpolation(&xs, 0, 0, &xf, 0, 0, in_time, in_2_delta) )
    return false;


  // 正規化
  Vector3 n_axis(axis/tvmet::norm2(axis));
  

  // 誤差回転行列Rの導出
  Matrix33 dR(tvmet::trans(R_cur) * R_ref);


  // target方向への回転の導出
  double cos_angle_a = tvmet::dot(n_axis, (dR * n_axis));
  if( cos_angle_a > (1.0 - 1.0e-6) )
    angle_a = 0;
  else if(cos_angle_a < -(1.0 - 1.0e-6) )
    angle_a = 3.1415;
  else
    angle_a = acos(cos_angle_a);


  if(angle_a > 1e-3){
    axis_a = tvmet::cross(n_axis, (dR * n_axis));
    axis_a /= tvmet::norm2(axis_a);
  }
  else{
    axis_a = Vector3(1,0,0);
  }


  Matrix33 R_a(rodrigues(axis_a, angle_a));


  // target方向以外への回転の導出
  Matrix33 R_b(tvmet::trans(R_a) * dR);
  Vector3  omega_b(omegaFromRot(R_b));
  

  angle_b = tvmet::norm2(omega_b);


  if(fabs(angle_b) > 1.0e-6) {
    axis_b  = omega_b / angle_b;
  }
  else {
    axis_b = n_axis;
  }


  R_base = R_cur;

  return true;
}
