// -*- c++ -*-
/*
 3次スプライン補間クラスライブラリ
 reference program : Spliner.hpp
                     Written by Takaaki Matsumoto
                     All rights reserved.(2004-2005)
*/

#include <vector>
#include <math.h>

namespace creek
{
  class CubicSplineInterpolator
  {
  public:
    CubicSplineInterpolator();
    CubicSplineInterpolator(const std::vector<double> &Xsource, const std::vector<double> &Ysource, double FirstGradient = 0, double LastGradient = 0);
    ~CubicSplineInterpolator();
  
    void clear();
    bool isInit() {return m_isInit;}

    void calcInterpolation(const std::vector<double> &Xsource, const std::vector<double> &Ysource, double FirstGradient = 0, double LastGradient = 0);
    double get(double time);
    std::vector<double> sequence(double sampling);

    CubicSplineInterpolator& operator=(const CubicSplineInterpolator& si);
  

  private:
    bool m_isInit;
  
    int m_nodeNumber;
    std::vector<double> m_x, m_y, m_y2d;
  };
};
