// -*- c++ -*-

#ifndef CREEK_TYPES_EIGEN_H
#define CREEK_TYPES_EIGEN_H

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace creek
{
  enum FootType {RFOOT=-1, DFOOT, LFOOT, AIR};
  inline FootType reverseFoot(FootType in_type) {
    FootType ret(DFOOT);
    if( in_type == RFOOT )
      ret = LFOOT;
    else if( in_type == LFOOT )
      ret = RFOOT;
    return ret;
  }

  using Eigen::Matrix3d;
  using Eigen::Vector4d;
  using Eigen::Vector3d;
  using Eigen::Vector2d;

  typedef Eigen::Matrix3d Matrix3;
  typedef Eigen::Vector4d Vector4;
  typedef Eigen::Vector3d Vector3;
  typedef Eigen::Vector2d Vector2;

  typedef Eigen::Transform<double, 3, Eigen::AffineCompact> Position;
  
  struct StepData
  {
    StepData()
    {
      rfoot.translation() = Vector3::Zero();
      rfoot.linear() = Matrix3::Identity();

      lfoot.translation() = Vector3::Zero();
      lfoot.linear() = Matrix3::Identity();

      com = Vector3::Zero();
      zmp = Vector3::Zero();
      cp  = Vector3::Zero();

      sup = DFOOT;
    }
    Position rfoot, lfoot;
    Vector3  com, zmp, cp;
    FootType sup;
  };


  struct CoordinateSystem
  {
    CoordinateSystem()
    {
      pos = Vector3::Zero();
      rot = Matrix3::Identity();
    }
    CoordinateSystem(const Vector3 &in_pos, const Matrix3 &in_rot)
    {
      pos = in_pos;
      rot = in_rot;
    }
    Vector3 pos;
    Matrix3 rot;
  };


  inline Eigen::IOFormat IOvec() {
    return Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]");
  }
  inline Eigen::IOFormat IOmat() {
    return Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", "]\n", "", "", "[", "]");
  }
}

#endif
