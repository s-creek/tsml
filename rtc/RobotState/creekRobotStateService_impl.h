// -*-C++-*-

#include "creekRobotStateService.hh"

#ifndef CREEKROBOTSTATESERVICE_IMPL_H
#define CREEKROBOTSTATESERVICE_IMPL_H
 
class creekRobotState;

class creekRobotStateService_impl
  : public virtual POA_OpenHRP::creekRobotStateService,
    public virtual PortableServer::RefCountServantBase
{
public:
  creekRobotStateService_impl();
  virtual ~creekRobotStateService_impl();

  void showState();
  void logStart(const char *date);
  void test();

  void setComponent (creekRobotState * i_comp) {
    m_comp = i_comp;
  }

private:
  creekRobotState *m_comp;
};

#endif // CREEKROBOTSTATESERVICE_IMPL_H


