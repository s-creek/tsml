#include "KalmanFilter.h"

using namespace creek;

KalmanFilter::KalmanFilter()
{
  m_P(0,0) = 0.0;  m_P(0,1) = 0.0;
  m_P(1,0) = 0.0;  m_P(1,1) = 0.0;

  m_Q(0,0) = 0.001 * 0.005;  m_Q(0,1) = 0.000 * 0.005;
  m_Q(1,0) = 0.000 * 0.005;  m_Q(1,1) = 0.003 * 0.005;

  //m_R = 0.3;
  m_R = 0.03;

  m_A(0,0) = 1.0;  m_A(0,1) =-0.005;
  m_A(1,0) = 0.0;  m_A(1,1) = 1.0;

  m_B(0) = 0.005;  m_B(1) = 0.0;

  m_C(0) = 1.0;    m_C(1) = 0.0;

  m_x(0) = 0.0;    m_x(1) = 0.0;
}


double KalmanFilter::filtering(double y, double u)
{
  //
  // Time Update (Predict)
  //
  // Predicted (a priori) state estimate
  m_x = m_A * m_x + m_B * u;

  // Predicted (a priori) estimate covariance
  //m_P = m_A * m_P * tvmet::trans(m_A) + m_Q;
  m_P = m_A * m_P * m_A.transpose() + m_Q;
  
  

  //
  // Measurement Update (Correct)
  //
  // Innovation or measurement residual
  //double e = y - dot(m_C, m_x);
  double e = y - m_C.dot(m_x);

  // Innovation (or residual) covariance
  //double S = dot( tvmet::trans(m_P) * m_C, m_C) + m_R;
  double S = tvector(m_P.transpose() * m_C).dot(m_C) + m_R;

  // Optimal Kalman gain
  m_K = m_P * m_C / S;

  // Updated (a posteriori) state estimate
  m_x = m_x + m_K * e;

  // Updated (a posteriori) estimate covariance
  //m_P = (1 - dot(m_K, m_C)) * m_P;
  m_P = (1 - m_K.dot(m_C)) * m_P;

  //m_y = dot(m_C, m_x);
  m_y = m_C.dot(m_x);

  return m_y;
}
