// -*-C++-*-

#include "creekCameraViewerService.hh"

#ifndef CREEKCAMERAVIEWERSERVICE_IMPL_H
#define CREEKCAMERAVIEWERSERVICE_IMPL_H

class creekCameraViewer;
 
class creekCameraViewerService_impl
  : public virtual POA_OpenHRP::creekCameraViewerService,
    public virtual PortableServer::RefCountServantBase
{
public:
  creekCameraViewerService_impl();
  virtual ~creekCameraViewerService_impl();

  void setDraw();
  inline bool draw() { return m_draw; }

  void setSearchFlag(const char *list);
  void show();
  void saveData(const char *path);

  inline void setComponent(creekCameraViewer *in_comp) { m_comp = in_comp; }

private:
  bool m_draw;
  creekCameraViewer *m_comp;
};


#endif // CREEKCAMERAVIEWERSERVICE_IMPL_H


