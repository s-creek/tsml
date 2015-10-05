#ifndef RATSMATRIX_H
#define RATSMATRIX_H
//#include <hrpUtil/Eigen3d.h>
#include <cnoid/EigenTypes>
#include <cnoid/EigenUtil>
#include <iostream>

using namespace cnoid;

namespace rats
{
  inline bool eps_eq(const double a, const double b, const double eps = 0.001)
  {
    return fabs((a)-(b)) <= eps;
  };
  Vector3 matrix_log(const Matrix3& m);

  // matrix product using quaternion normalization
  void rotm3times (Matrix3& m12, const Matrix3& m1, const Matrix3& m2);
  void difference_rotation(Vector3& ret_dif_rot, const Matrix3& self_rot, const Matrix3& target_rot);
  void calcRodrigues(Matrix3& out_R, const Vector3& axis, double q);
 
  struct coordinates {
    Vector3 pos;
    Matrix3 rot;
    coordinates() : pos(Vector3::Zero()), rot(Matrix3::Identity()) {};
    coordinates(const Vector3& p, const Matrix3& r) : pos(p), rot(r) {};
    coordinates(const Vector3& p) : pos(p), rot(Matrix3::Identity()) {};
    coordinates(const Matrix3& r) : pos(Vector3::Zero()), rot(r) {};
    coordinates(const coordinates& c) : pos(c.pos), rot(c.rot) {};
    virtual ~coordinates() {
    }
    coordinates& operator=(const coordinates& c) {
      if (this != &c) {
        pos = c.pos;
        rot = c.rot;
      }
      return *this;
    }
    void rotate_with_matrix (const Matrix3& mat, const std::string& wrt = ":local") {
      Matrix3 rot_org(rot);
      if (wrt == ":local") {		  
        //			alias(rot) = rot * mat;
        rotm3times(rot, rot_org, mat);
      } else if(wrt == ":world") {
        //			alias(rot) = mat * rot;
        rotm3times(rot, mat, rot_org);
      } else {
        std::cerr << "**** invalid wrt! ****" << std::endl;
      }
    }
    void rotate (const double theta, const Vector3& axis, const std::string& wrt = ":local") {
      Eigen::AngleAxis<double> tmpr(theta, axis);
      rotate_with_matrix(tmpr.toRotationMatrix(), wrt);
    }
    /* void difference_rotation(Vector3& dif_rot, const coordinates& c) const { */
    /*   dif_rot = rot * matrix_log(rot.transpose() * c.rot); */
    /* } */
    void difference(Vector3& dif_pos, Vector3& dif_rot, const coordinates& c) const {
      dif_pos = c.pos - pos;
      difference_rotation(dif_rot, rot, c.rot);
    }
    //abc
    void inverse_transformation(coordinates& inv) const {
      inv.rot = rot.transpose();
      inv.pos = inv.rot*(-1 * pos);
    }
    void transformation(coordinates& tc, coordinates c, const std::string& wrt = ":local") const {
      tc = *this;
      inverse_transformation(tc);
      if (wrt == ":local") {
        tc.transform(c);
      } else if(wrt == ":world") {
        c.transform(tc);
        tc = c;
      } else {
        std::cerr << "**** invalid wrt! ****" << std::endl;
      }
    }
    void transform(const coordinates& c, const std::string& wrt = ":local") {
      if (wrt == ":local") {
        pos += rot * c.pos;
        // alias(rot) = rot * c.rot;
        Matrix3 rot_org(rot);
        rotm3times(rot, rot_org, c.rot);
      } else if (wrt == ":world") {
        Vector3 p(c.pos);
        Matrix3 r(c.rot);      
        p += r * pos; 
        rotm3times(r, c.rot, rot);      
        pos = p;
        rot = r;
      } else {
        std::cerr << "**** invalid wrt! ****" << std::endl;
      }
    }
  };

  void mid_rot(Matrix3& mid_rot, const double p, const Matrix3& rot1, const Matrix3& rot2);
  void mid_coords(coordinates& mid_coords, const double p, const coordinates& c1, const coordinates& c2);
};
#endif /* RATSMATRIX_H */
