// -*-C++-*-

#include "creekRobotStateService.hh"

#ifndef CREEKROBOTSTATESERVICE_IMPL_H
#define CREEKROBOTSTATESERVICE_IMPL_H
 
class creekRobotStateService_impl
  : public virtual POA_OpenHRP::creekRobotStateService,
    public virtual PortableServer::RefCountServantBase
{
public:
  creekRobotStateService_impl();
  virtual ~creekRobotStateService_impl();

  void test();
};

#endif // CREEKROBOTSTATESERVICE_IMPL_H


