// -*- c++ -*-

#ifndef CREEK_TYPES_H
#define CREEK_TYPES_H

// #include <tvmet/Vector.h>
// #include <tvmet/Matrix.h>

namespace creek
{
  enum FootType {RFOOT=-1, DFOOT, LFOOT};

  enum InterpolationType {
    LINEAR=0,
    CUBIC,
    QUINTIC,
    QUARTIC_LINEAR
  };

  // typedef tvmet::Matrix<double, 3, 3> Matrix33;
  // typedef tvmet::Matrix<double, 3, 2> Matrix32;
  // typedef tvmet::Vector<double, 5>    Vector5;
  // typedef tvmet::Vector<double, 4>    Vector4;
  // typedef tvmet::Vector<double, 3>    Vector3;
  // typedef tvmet::Vector<double, 2>    Vector2;
}

#endif
