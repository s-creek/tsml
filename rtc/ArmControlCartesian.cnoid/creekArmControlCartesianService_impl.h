// -*- c++ -*-

#include "creekArmControlCartesianService.hh"


#ifndef CREEK_ARMCONTROLCARTESIANSERVICE_IMPL_H
#define CREEK_ARMCONTROLCARTESIANSERVICE_IMPL_H
 
class creekArmControlCartesian;

class creekArmControlCartesianService_impl
  : public virtual POA_OpenHRP::creekArmControlCartesianService,
    public virtual PortableServer::RefCountServantBase
{
public:
  creekArmControlCartesianService_impl();
  virtual ~creekArmControlCartesianService_impl();

  void setArm(int armId);

  void setComponent (creekArmControlCartesian * i_comp) {
    m_comp = i_comp;
  }

private:
  creekArmControlCartesian * m_comp;
};



#endif // CREEK_ARMCONTROLCARTESIANSERVICE_IMPL_H

