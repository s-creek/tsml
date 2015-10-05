// -*- c++ -*-

#ifndef CREEK_KALMAN_FILTER_H
#define CREEK_KALMAN_FILTER_H

//#include <tvmet/Matrix.h>
//#include <tvmet/Vector.h>
#include <Eigen/Core>

namespace creek
{
  //typedef tvmet::Matrix<double, 2, 2> tmatrix;
  //typedef tvmet::Vector<double, 2> tvector;

  using Eigen::Matrix2d;
  using Eigen::Vector2d;
  typedef Eigen::Matrix2d tmatrix;
  typedef Eigen::Vector2d tvector;
};


namespace creek
{
  class KalmanFilter
  {
  public:
    KalmanFilter();

    inline void setP(double p00, double p01, double p10, double p11)  { m_P(0,0) = p00; m_P(0,1) = p01; m_P(1,0) = p10; m_P(1,1) = p11; }
    inline void setQ(double q00, double q01, double q10, double q11)  { m_Q(0,0) = q00; m_Q(0,1) = q01; m_Q(1,0) = q10; m_Q(1,1) = q11; }
    inline void setR(double R)  { m_R = R; }

    inline void setA(double a00, double a01, double a10, double a11)  { m_A(0,0) = a00; m_A(0,1) = a01; m_A(1,0) = a10; m_A(1,1) = a11; }
    inline void setB(double b0, double b1)  { m_B(0) = b0; m_B(1) = b1; }

    inline void setX(double in_x) { m_x(0) = in_x; }

    double filtering(double y, double u=0);
    
    inline double get() { return m_y; };


  private:
    tmatrix m_P;
    tmatrix m_Q;
    double  m_R;
    tvector m_K;

    tmatrix m_A;
    tvector m_B, m_C;

    tvector m_x;
    double  m_y;
  };
};

#endif
