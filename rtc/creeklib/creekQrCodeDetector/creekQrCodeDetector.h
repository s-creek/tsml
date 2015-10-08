// -*- c++ -*-

#ifndef CREEK_QR_CODE_DETECTOR_H
#define CREEK_QR_CODE_DETECTOR_H

#include <opencv2/opencv.hpp>

namespace creek
{
  class creekQrCodeDetector
  {
  public:
    creekQrCodeDetector(int in_requiredChildNum=5);
    inline void set(int in_requiredChildNum) { m_requiredChildNum = in_requiredChildNum; }

    void drawFinderPattern(cv::Mat &in_src, int num=-1);
    void drawVertices(cv::Mat &in_src, int num=-1);

    bool detectQrCode(cv::Mat &in_src, double in_th=0.7, int in_method=0);
    inline cv::Mat getQrImage() { return m_out.clone(); }
    cv::Mat getQrImage(cv::Mat &in_src, int in_method=0);

    inline cv::Point2f getCenter() { return m_center; }

    inline void debug(bool in_flag=false) { m_debug = in_flag; }


  private:
    bool detectFinderPattern(double in_th=0.70);
    int align();
    void getVertices(int in_orientation);
    bool getIntersectionPoint();
    bool cropImage(cv::Mat &in_src);
    bool warpImage(cv::Mat &in_src);

    double minAreaRatio(std::vector<int> &in_contIndex);
    float distance(cv::Point2f &P, cv::Point2f &Q);
    float cross(cv::Point2f v1, cv::Point2f v2);
    int getOrientation();


    int m_requiredChildNum;
    cv::Mat m_gray, m_edge, m_out;

    std::vector< std::vector<cv::Point> > m_contours;
    std::vector< cv::Vec4i > m_hierarchy;
    std::vector< std::vector<int> > m_index;  // finder pattern index (3 x requiredChildNum)
    cv::Point2f m_vertices[3][4];  // L, M, Q
    cv::Point2f m_intersection;    // N
    cv::Point2f m_top, m_center;

    bool m_debug;
  };
};

#endif
