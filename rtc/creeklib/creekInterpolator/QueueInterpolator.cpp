#include "QueueInterpolator.h"
#include <cstring> // memcpy
#include <cmath>   // pow

using namespace creek;

QueueInterpolator::QueueInterpolator(unsigned int in_dim, double in_dt, InterpolationType in_itype)
 : InterpolatorBase(in_dim, in_dt, in_itype)
{
  m_pre_x   = new double[in_dim];
  m_pre_dx  = new double[in_dim];
  m_pre_ddx = new double[in_dim];
  for(unsigned int i = 0; i < in_dim; i++) {
	m_pre_x[i]    = 0.0;
	m_pre_dx[i]   = 0.0;
	m_pre_ddx[i]  = 0.0;
  }

  is_init = false;
}


QueueInterpolator::~QueueInterpolator()
{
  delete m_pre_x;
  delete m_pre_dx;
  delete m_pre_ddx;
}


void QueueInterpolator::get(double* out, bool popp)
{
  if( !m_seq.empty() ) {
    double *&tmp = m_seq.front();
    memcpy(m_last_out, tmp, m_memsize);

    if(popp) {
	  InterpolatorBase::pop();
    }
  }

  std::memcpy(out, m_last_out, m_memsize);
}


bool QueueInterpolator::set(const double *in_x0, const double *in_dx, const double *in_ddx, double in_time, double in_2_delta, bool in_abs)
{
  double *in_x = new double[m_dim];
  if( in_x0 == 0 )  for(unsigned int i = 0; i < m_dim; i++)  in_x[i] = 0.0;
  else              for(unsigned int i = 0; i < m_dim; i++)  in_x[i] = in_x0[i];

  
  // set initial data
  if( !is_init ) {
	std::memcpy(m_pre_x, in_x, m_memsize);

	if(in_dx == 0)   for(unsigned int i = 0; i < m_dim; i++)  m_pre_dx[i] = 0.0;
	else             std::memcpy(m_pre_dx, in_dx, m_memsize);

	if(in_ddx == 0)  for(unsigned int i = 0; i < m_dim; i++)  m_pre_ddx[i] = 0.0;
	else             std::memcpy(m_pre_ddx, in_ddx, m_memsize);

	is_init = true;

	memcpy(m_last_out, in_x, m_memsize);

	if( in_time > 0.0 )
	  return set(in_x, in_dx, in_ddx, in_time, in_2_delta);
	else
	  return true;
  }
  else if( in_time < m_eps ) {
	return false;
  }
  else {
	if( !empty() ) {
	  // 接続点が被るので一個削除
	  double *&tmp = m_seq.back();
	  delete [] tmp;
	  m_seq.pop_back();
	}

	
	if( in_2_delta < m_eps )
	  in_2_delta = 0.2 * in_time;


	if( !in_abs )  for(unsigned int i = 0; i < m_dim; i++)  in_x[i] += m_pre_x[i];
	  

	if( !InterpolatorBase::calcInterpolation(m_pre_x, m_pre_dx, m_pre_ddx, in_x, in_dx, in_ddx, in_time, in_2_delta) )
	  return false;

	
	std::memcpy(m_pre_x, in_x, m_memsize);
	
	if(in_dx == 0)   for(unsigned int i = 0; i < m_dim; i++)  m_pre_dx[i] = 0.0;
	else             std::memcpy(m_pre_dx, in_dx, m_memsize);
	
	if(in_ddx == 0)  for(unsigned int i = 0; i < m_dim; i++)  m_pre_ddx[i] = 0.0;
	else             std::memcpy(m_pre_ddx, in_ddx, m_memsize);
  }

  return true;
}
