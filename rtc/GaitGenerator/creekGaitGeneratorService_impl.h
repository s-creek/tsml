// -*-C++-*-

#include "creekGaitGeneratorService.hh"


#ifndef CREEKGAITGENERATORSERVICE_IMPL_H
#define CREEKGAITGENERATORSERVICE_IMPL_H
 
class creekGaitGenerator;

class creekGaitGeneratorService_impl
 : public virtual POA_OpenHRP::creekGaitGeneratorService,
   public virtual PortableServer::RefCountServantBase
{
 private:
  creekGaitGenerator *m_comp;

 public:
   creekGaitGeneratorService_impl();
   virtual ~creekGaitGeneratorService_impl();

   void setTargetPos(::CORBA::Double x, ::CORBA::Double y, ::CORBA::Double th, ::CORBA::Double time);
   void startStepping();
   void stopStepping();
   void test();

  void setComp(creekGaitGenerator *in_comp) { m_comp = in_comp; }
};



#endif // CREEKGAITGENERATORSERVICE_IMPL_H


