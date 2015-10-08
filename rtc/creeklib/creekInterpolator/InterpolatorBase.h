// -*- c++ -*-

#ifndef CREEK_INTERPOLATOR_BASE_H
#define CREEK_INTERPOLATOR_BASE_H

#include <creekUtil/creekTypes.h>
#include <deque>

namespace creek 
{
  class InterpolatorBase
  {
  public:
    InterpolatorBase(unsigned int in_dim, double in_dt, InterpolationType in_itype=LINEAR);
    ~InterpolatorBase();

    void clear();
    void pop();

    inline bool empty(){ return m_seq.empty(); }
    inline unsigned int numSequence() { return m_seq.size(); }
    inline void setInterpolationType(InterpolationType in_itype) { m_itype = in_itype; }
    inline InterpolationType interpolationType() { return m_itype; }

    InterpolatorBase& operator=(const InterpolatorBase& org);

	
  protected:
    unsigned int m_dim;
    double m_dt;
    long unsigned int m_memsize;
    InterpolationType m_itype;

    std::deque<double*> m_seq;

    double *m_last_out;
    double *m_dxs, *m_dxf;
    double *m_ddxs, *m_ddxf;

    double m_eps;
    bool linear_interpolation(const double *in_xs, const double *in_xf, double in_time);
    bool cubic_interpolation(const double *in_xs, const double *in_dxs, const double *in_xf, const double *in_dxf, double in_time);
    bool quintic_interpolation(const double* in_xs, const double* in_dxs, const double* in_ddxs, const double* in_xf, const double* in_dxf, const double* in_ddxf, double in_time);
    bool qlq_interpolation(const double *in_xs, const double *in_dxs, const double *in_xf, const double *in_dxf, double in_time, double in_2_delta);

    bool calcInterpolation(const double* in_xs, const double* in_dxs, const double* in_ddxs, 
			   const double* in_xf, const double* in_dxf, const double* in_ddxf, double in_time, double in_2_delta=0);
  };
}

#endif
