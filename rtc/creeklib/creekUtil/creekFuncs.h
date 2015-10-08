// -*- c++ -*-

#ifndef CREEK_FUNCS_H
#define CREEK_FUNCS_H

#include <creekUtil/creekTypes.h>

namespace creek
{
  void calcRodrigues(Matrix33& out_R, const Vector3& axis, double q);

  inline Matrix33 rodrigues(const Vector3 &axis, double q)
  {
    Matrix33 R;
    calcRodrigues(R, axis, q);
    return R;
  }

  Vector3 omegaFromRot(const Matrix33& r);
  Vector3 rpyFromRot(const Matrix33& m);

  double intermediateYaw(const Matrix33& ma, const Matrix33& mb);
}

#endif
