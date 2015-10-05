#include "RatsMatrix.h"

namespace rats
{
  Vector3 matrix_log(const Matrix3& m) {
    Vector3 mlog;
    double q0, th;
    Vector3 q;
    double norm;
  
    Eigen::Quaternion<double> eiq(m);
    q0 = eiq.w();
    q = eiq.vec();
    norm = q.norm();
    if (norm > 0) {
      if ((q0 > 1.0e-10) || (q0 < -1.0e-10)) {
        th = 2 * std::atan(norm / q0);
      } else if (q0 > 0) {
        th = M_PI / 2;
      } else {
        th = -M_PI / 2;
      }
      mlog = (th / norm) * q ;
    } else {
      mlog = Vector3::Zero();
    }
    return mlog;
  }

  // matrix product using quaternion normalization
  void rotm3times (Matrix3& m12, const Matrix3& m1, const Matrix3& m2) {
    Eigen::Quaternion<double> eiq1(m1);
    Eigen::Quaternion<double> eiq2(m2);
    Eigen::Quaternion<double> eiq3;
    eiq3 = eiq1 * eiq2;
    eiq3.normalize();
    m12 = eiq3.toRotationMatrix();
  }

  void difference_rotation(Vector3& ret_dif_rot, const Matrix3& self_rot, const Matrix3& target_rot)
  {
    //ret_dif_rot = self_rot * hrp::omegaFromRot(self_rot.transpose() * target_rot);
    ret_dif_rot = self_rot * Vector3(rats::matrix_log(Matrix3(self_rot.transpose() * target_rot)));
  }

  void mid_rot(Matrix3& mid_rot, const double p, const Matrix3& rot1, const Matrix3& rot2) {
    Matrix3 r(rot1.transpose() * rot2);
    Vector3 omega(matrix_log(r));
    if (eps_eq(omega.norm(),0.0)) { // c1.rot and c2.rot are same
      mid_rot = rot1;
    } else {
      calcRodrigues(r, omega.normalized(), omega.norm()*p);
      //mid_rot = c1.rot * r;
      rotm3times(mid_rot, rot1, r);
    }
  };

  void mid_coords(coordinates& mid_coords, const double p, const coordinates& c1, const coordinates& c2) {
    Vector3 mid_point, omega;
    Matrix3 mid_rot, r;
  
    mid_point = (1 - p) * c1.pos + p * c2.pos;
    r = c1.rot.transpose() * c2.rot;
    omega = matrix_log(r);
    if (eps_eq(omega.norm(),0.0)) { // c1.rot and c2.rot are same
      mid_rot = c1.rot;
    } else {
      calcRodrigues(r, omega.normalized(), omega.norm()*p);
      //mid_rot = c1.rot * r;
      rotm3times(mid_rot, c1.rot, r);
    }
    mid_coords = coordinates(mid_point, mid_rot);
  };

  //add by wu
  void calcRodrigues(Matrix3& out_R, const Vector3& axis, double q)
  {
    // E + a_hat*sin(q) + a_hat*a_hat*(1-cos(q))
    //
    //    |  0 -az  ay|
    // =E+| az   0 -ax|*s + a_hat*a_hat*v
    //    |-ay  ax   0|
    //
    //    |  0 -az  ay|     |-az*az-ay*ay        ax*ay        az*ax|
    // =E+| az   0 -ax|*s + |       ax*ay -az*az-ax*ax        ay*az|*v
    //    |-ay  ax   0|     |       az*ax        ay*az -ax*ax-ay*ay|
    //
    //  |1-az*az*v-ay*ay*v     -az*s+ax*ay*v      ay*s+az*ax*v|
    // =|     az*s+ax*ay*v 1-az*az*v-ax*ax*v     -ax*s+ay+az*v|
    //  |    -ay*s+az*ax*v      ax*s+ay*az*v 1-ax*ax*v-ay*ay*v|
    //

    const double sth = sin(q);
    const double vth = 1.0 - cos(q);

    double ax = axis(0);
    double ay = axis(1);
    double az = axis(2);

    const double axx = ax*ax*vth;
    const double ayy = ay*ay*vth;
    const double azz = az*az*vth;
    const double axy = ax*ay*vth;
    const double ayz = ay*az*vth;
    const double azx = az*ax*vth;

    ax *= sth;
    ay *= sth;
    az *= sth;

    out_R << 1.0 - azz - ayy, -az + axy,       ay + azx,
      az + axy,        1.0 - azz - axx, -ax + ayz,
      -ay + azx,       ax + ayz,        1.0 - ayy - axx;


    /*
      Eigen::AngleAxisd rot(q ,axis);
      Eigen::Matrix3d m1_rot = rot.toRotationMatrix();
      out_R= m1_rot;
    */
  }




}

