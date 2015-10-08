#include "InterpolatorBase.h"

#include <cstring> // memcpy
#include <cmath>   // pow

using namespace creek;

InterpolatorBase::InterpolatorBase(unsigned int in_dim, double in_dt, InterpolationType in_itype):
  m_dim(in_dim),
  m_dt(in_dt),
  m_memsize(in_dim * sizeof(double)),
  m_itype(in_itype)
{
  m_last_out = new double[in_dim];
  for(unsigned int i = 0; i < in_dim; i++)
    m_last_out[i] = 0.0;


  m_dxs  = new double[in_dim];
  m_dxf  = new double[in_dim];
  m_ddxs = new double[in_dim];
  m_ddxf = new double[in_dim];
  for(unsigned int i = 0; i < in_dim; i++) {
    m_dxs[i]  = 0.0;
    m_dxf[i]  = 0.0;
    m_ddxs[i] = 0.0;
    m_ddxf[i] = 0.0;
  }


  m_eps = in_dt * 0.001;
}


InterpolatorBase::~InterpolatorBase()
{
  clear();
  delete m_last_out;
  delete m_dxs;
  delete m_dxf;
  delete m_ddxs;
  delete m_ddxf;
}

void InterpolatorBase::clear()
{
  for(std::deque<double*>::iterator iter = m_seq.begin(); iter != m_seq.end(); ++iter)
    delete [] * iter;
  m_seq.clear();
}


void InterpolatorBase::pop()
{
  double *&tmp = m_seq.front();
  delete [] tmp;
  m_seq.pop_front();
}


bool InterpolatorBase::linear_interpolation(const double *in_xs, const double *in_xf, double in_time)
{  
  for(double t = 0.0; t < (in_time+m_eps); t += m_dt) {
    double *tmp = new double[m_dim];
    for(unsigned int i = 0; i < m_dim; i++)
      tmp[i] = in_xs[i] + (in_xf[i] - in_xs[i]) / in_time * t;
    m_seq.push_back(tmp);
  }

  return !m_seq.empty();
}


bool InterpolatorBase::cubic_interpolation(const double *in_xs, const double *in_dxs, const double *in_xf, const double *in_dxf, double in_time)
{
  // set velocity
  if(in_dxs == 0)
    for(unsigned int i = 0; i < m_dim; i++)
      m_dxs[i] = 0.0;
  else
    std::memcpy(m_dxs, in_dxs, m_memsize);

  if(in_dxf == 0)
    for(unsigned int i = 0; i < m_dim; i++)
      m_dxf[i] = 0.0;
  else
    std::memcpy(m_dxf, in_dxf, m_memsize);


  // set const value
  double a[m_dim][4];
  for(unsigned int i = 0; i < m_dim; i++) {
    a[i][0] = in_xs[i];
    a[i][1] = m_dxs[i];
    a[i][2] = ( 3*(in_xf[i]-in_xs[i])/in_time - 2*m_dxs[i] - m_dxf[i] )/in_time;
    a[i][3] = ( 2*(in_xs[i]-in_xf[i])/in_time + m_dxs[i] + m_dxf[i] )/in_time/in_time;
  }


  // calc interpolation
  for(double t = 0.0; t < (in_time+m_eps); t += m_dt) {
    double *tmp = new double[m_dim];
    for(unsigned int i = 0; i < m_dim; i++)
      tmp[i] = a[i][0] + ( a[i][1] + ( a[i][2] + a[i][3]*t )*t )*t;
    m_seq.push_back(tmp);
  }

  return !m_seq.empty();
}


bool InterpolatorBase::quintic_interpolation(const double* in_xs, const double* in_dxs, const double* in_ddxs, const double* in_xf, const double* in_dxf, const double* in_ddxf, double in_time)
{
  // set velocity
  if(in_dxs == 0)
    for(unsigned int i = 0; i < m_dim; i++)
      m_dxs[i] = 0.0;
  else
    std::memcpy(m_dxs, in_dxs, m_memsize);

  if(in_dxf == 0)
    for(unsigned int i = 0; i < m_dim; i++)
      m_dxf[i] = 0.0;
  else
    std::memcpy(m_dxf, in_dxf, m_memsize);

  // set accel
  if(in_ddxs == 0)
    for(unsigned int i = 0; i < m_dim; i++)
      m_ddxs[i] = 0.0;
  else
    std::memcpy(m_ddxs, in_ddxs, m_memsize);

  if(in_ddxf == 0)
    for(unsigned int i = 0; i < m_dim; i++)
      m_ddxf[i] = 0.0;
  else
    std::memcpy(m_ddxf, in_ddxf, m_memsize);


  // set const value
  double a[m_dim][6];
  for(unsigned int i = 0; i < m_dim; i++) {
    a[i][0] = in_xs[i];
    a[i][1] = m_dxs[i];
    a[i][2] = m_ddxs[i]/2.0;
    a[i][3] = (20*in_xf[i] - 20*in_xs[i] - ( 8*m_dxf[i] + 12*m_dxs[i])*in_time - (3*m_ddxs[i] -   m_ddxf[i])*pow(in_time,2)) / (2*pow(in_time,3));
    a[i][4] = (30*in_xs[i] - 30*in_xf[i] + (14*m_dxf[i] + 16*m_dxs[i])*in_time + (3*m_ddxs[i] - 2*m_ddxf[i])*pow(in_time,2)) / (2*pow(in_time,4));
    a[i][5] = (12*in_xf[i] - 12*in_xs[i] - ( 6*m_dxf[i] +  6*m_dxs[i])*in_time - (  m_ddxs[i] -   m_ddxf[i])*pow(in_time,2)) / (2*pow(in_time,5));
  }


  // calc interpolation
  for(double t = 0.0; t < (in_time+m_eps); t += m_dt) {
    double *tmp = new double[m_dim];
    for(unsigned int i = 0; i < m_dim; i++)
      tmp[i] = a[i][0] + ( a[i][1] + ( a[i][2] + ( a[i][3] + ( a[i][4] + a[i][5]*t )*t )*t )*t )*t;
    m_seq.push_back(tmp);
  }

  return !m_seq.empty();
}


bool InterpolatorBase::qlq_interpolation(const double *in_xs, const double *in_dxs, const double *in_xf, const double *in_dxf, double in_time, double in_2_delta)
{
  if( in_2_delta < m_eps || (2*in_2_delta-m_eps) > in_time )
    return false;


  // set velocity
  if(in_dxs == 0)
    for(unsigned int i = 0; i < m_dim; i++)
      m_dxs[i] = 0.0;
  else
    std::memcpy(m_dxs, in_dxs, m_memsize);

  if(in_dxf == 0)
    for(unsigned int i = 0; i < m_dim; i++)
      m_dxf[i] = 0.0;
  else
    std::memcpy(m_dxf, in_dxf, m_memsize);


  // set const value
  double delta = in_2_delta/2.0;
  double m[m_dim];
  for(unsigned int i = 0; i < m_dim; i++)
    m[i] = (in_xf[i] - in_xs[i] - delta*(m_dxs[i]+m_dxf[i])) / (in_time - in_2_delta);


  // calc interpolation
  for(double t = 0.0; t < (in_time+m_eps); t += m_dt) {
    double *tmp = new double[m_dim];

    // quartic area
    if( t < in_2_delta )
      for(unsigned int i = 0; i < m_dim; i++)
	tmp[i] = in_xs[i] + m_dxs[i]*t + (m[i]-m_dxs[i])/(16.0*pow(delta,3))*pow(t,3)*(4*delta-t);
    // linear area
    else if( t < (in_time-in_2_delta) )
      for(unsigned int i = 0; i < m_dim; i++)
	tmp[i] = in_xs[i] + m_dxs[i]*delta + m[i]*(t-delta);
    // quartic area
    else
      for(unsigned int i = 0; i < m_dim; i++)
	tmp[i] = in_xf[i] - m_dxf[i]*(in_time-t) - (m[i]-m_dxf[i])/(16.0*pow(delta,3))*pow((in_time-t),3)*(4*delta-in_time+t);

    m_seq.push_back(tmp);
  }

  return !m_seq.empty();
}


bool InterpolatorBase::calcInterpolation(const double* in_xs, const double* in_dxs, const double* in_ddxs, 
			   const double* in_xf, const double* in_dxf, const double* in_ddxf, double in_time, double in_2_delta)
{
  if(in_xs == 0 || in_xf == 0)
    return false;


  switch(m_itype) {

  case LINEAR: {
    return InterpolatorBase::linear_interpolation(in_xs, in_xf, in_time);
  } // end of case LINEAR

  case CUBIC: {
    return InterpolatorBase::cubic_interpolation(in_xs, in_dxs, in_xf, in_dxf, in_time);
  } // end of case CUBIC

  case QUINTIC: {
    return InterpolatorBase::quintic_interpolation(in_xs, in_dxs, in_ddxs, in_xf, in_dxf, in_ddxf, in_time);
  } // end of case QUINTIC

  case QUARTIC_LINEAR: {
    return InterpolatorBase::qlq_interpolation(in_xs, in_dxs, in_xf, in_dxf, in_time, in_2_delta);
  } // end of case QUARTIC_LINEAR

  default:
    return false;
  } // end of switch
}
