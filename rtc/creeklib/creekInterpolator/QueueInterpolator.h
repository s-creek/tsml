// -*- c++ -*-

#ifndef CREEK_QUEUE_INTERPOLATOR_H
#define CREEK_QUEUE_INTERPOLATOR_H

#include <creekInterpolator/InterpolatorBase.h>

namespace creek
{
  class QueueInterpolator : public InterpolatorBase
  {
  public:
    QueueInterpolator(unsigned int in_dim, double in_dt, InterpolationType in_itype=LINEAR);
    ~QueueInterpolator();

    void get(double* out, bool popp=true);
	
    inline bool set(const double *in_x, double in_time=0.0, double in_2_delta=0, bool in_abs=true) {
      return set(in_x, 0, 0, in_time, in_2_delta, in_abs);
    }
    inline bool set(const double *in_x, const double *in_dx, double in_time=0.0, double in_2_delta=0, bool in_abs=true) {
      return set(in_x, in_dx, 0, in_time, in_2_delta, in_abs);
    }
    bool set(const double *in_x, const double *in_dx, const double *in_ddx, double in_time=0.0, double in_2_delta=0, bool in_abs=true);

	
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


    void clear() {
      InterpolatorBase::clear();
      is_init = false;
    }

	
  private:
    double *m_pre_x, *m_pre_dx, *m_pre_ddx;
    bool is_init;
  };
}

#endif
