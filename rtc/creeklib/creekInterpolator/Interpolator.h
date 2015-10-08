// -*- c++ -*-

#ifndef CREEK_INTERPOLATOR_H
#define CREEK_INTERPOLATOR_H

#include <creekInterpolator/InterpolatorBase.h>

namespace creek 
{
  class Interpolator : public InterpolatorBase
  {
  public:
    Interpolator(unsigned int in_dim, double in_dt, InterpolationType in_itype=LINEAR);
    ~Interpolator();

    void get(double* out, bool popp=true);

    inline const double* at(int index) {
      return m_seq.at(index);
    }

    inline unsigned int dimension() {
      return m_dim;
    }

    inline std::deque<double*> *sequence() {
      return &m_seq;
    }
    
    inline double* front() {
      return m_seq.front();
    }
    
    inline double* back() {
      return m_seq.back();
    }

    inline bool calcInterpolation(const double *in_xs, const double *in_xf, double in_time, double in_2_delta=0) {
      if(!empty()) clear();
      return InterpolatorBase::calcInterpolation(in_xs, 0, 0, in_xf, 0, 0, in_time, in_2_delta);
    }
    inline bool calcInterpolation(const double *in_xs, const double *in_dxs, const double *in_xf, const double *in_dxf, double in_time, double in_2_delta=0) {
      if(!empty()) clear();
      return InterpolatorBase::calcInterpolation(in_xs, in_dxs, 0, in_xf, in_dxf, 0, in_time, in_2_delta);
    }
    inline bool calcInterpolation(const double* in_xs, const double* in_dxs, const double* in_ddxs, 
				  const double* in_xf, const double* in_dxf, const double* in_ddxf, double in_time, double in_2_delta=0) {
      if(!empty()) clear();
      return InterpolatorBase::calcInterpolation(in_xs, in_dxs, in_ddxs, in_xf, in_dxf, in_ddxf, in_time, in_2_delta);
    }

    Interpolator& operator=(const Interpolator& org);
  };
}

#endif
