#include "Interpolator.h"
#include <cstring> // memcpy
#include <cmath>   // pow

using namespace creek;

Interpolator::Interpolator(unsigned int in_dim, double in_dt, InterpolationType in_itype)
  : InterpolatorBase(in_dim, in_dt, in_itype)
{
}


Interpolator::~Interpolator()
{
}


void Interpolator::get(double* out, bool popp)
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
