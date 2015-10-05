// -*-C++-*-

#include "creekPointCloudViewerService.hh"

#ifndef CREEKPOINTCLOUDVIEWERSERVICE_IMPL_H
#define CREEKPOINTCLOUDVIEWERSERVICE_IMPL_H 

class creekPointCloudViewer;

class creekPointCloudViewerService_impl
  : public virtual POA_OpenHRP::creekPointCloudViewerService,
    public virtual PortableServer::RefCountServantBase
{
public:
  creekPointCloudViewerService_impl();
  virtual ~creekPointCloudViewerService_impl();

  void start();
  void stop();
  bool detectLandingPoint(double x, double y, double w, int ft);
  void getLandingPoint(double &x, double &y, double &z, double &r, double &p, double &w, int ft);
  void test();

  void changeMode();
  bool autoFittinSwitch();
  void clear();

  void setComponent(creekPointCloudViewer *in_comp) { m_comp = in_comp; }

private:
  creekPointCloudViewer *m_comp;
};



#endif // CREEKPOINTCLOUDVIEWERSERVICE_IMPL_H


