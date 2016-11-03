// -*-C++-*-

#include "creekSequencePlayerService.hh"

#ifndef CREEKSEQUENCEPLAYERSERVICE_IMPL_H
#define CREEKSEQUENCEPLAYERSERVICE_IMPL_H

class creekSequencePlayer;

class creekSequencePlayerService_impl
  : public virtual POA_OpenHRP::creekSequencePlayerService,
    public virtual PortableServer::RefCountServantBase
{
public:
  creekSequencePlayerService_impl();
  virtual ~creekSequencePlayerService_impl();
  
  void waitInterpolation();
  ::CORBA::Boolean setJointAngles(const OpenHRP::dSequence& jvs, ::CORBA::Double tm);
  ::CORBA::Boolean setJointAngle(const char* jname, ::CORBA::Double jv, ::CORBA::Double tm);
  ::CORBA::Boolean setBasePos(const OpenHRP::dSequence& pos, ::CORBA::Double tm);
  ::CORBA::Boolean setBaseRpy(const OpenHRP::dSequence& rpy, ::CORBA::Double tm);
  ::CORBA::Boolean setZmp(const OpenHRP::dSequence& zmp, ::CORBA::Double tm);
  ::CORBA::Boolean isEmpty();
  void jointCalib(int scale);

  ::CORBA::Boolean setBasePosRel(const OpenHRP::dSequence& pos, ::CORBA::Double tm);

  void setComponent (creekSequencePlayer * i_comp) {
    m_comp = i_comp;
  }

private:
  creekSequencePlayer *m_comp;
};

#endif // CREEKSEQUENCEPLAYERSERVICE_IMPL_H


