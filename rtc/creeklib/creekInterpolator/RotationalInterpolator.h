// -*- c++ -*-

#ifndef CREEK_ROTATIONAL_INTERPOLATOR_H
#define CREEK_ROTATIONAL_INTERPOLATOR_H

#include <creekInterpolator/InterpolatorBase.h>

namespace creek
{
  class RotationalInterpolator : public InterpolatorBase
  {
  public:
    RotationalInterpolator(double in_dt, InterpolationType in_itype=LINEAR);

    void get(Matrix33 &out_val, bool popp=true);
    inline Matrix33 get(bool popp=true) {
      Matrix33 out;
      get(out, popp);
      return out;
    }

    const Matrix33 at(int index);

    bool calcInterpolation(const Matrix33 &R_cur, const Matrix33 &R_ref, const Vector3 &axis, double in_time, double in_2_delta=0);


  private:
    Vector3  axis_a, axis_b;
    double   angle_a, angle_b;
    Matrix33 R_base;
  };

}

#endif
