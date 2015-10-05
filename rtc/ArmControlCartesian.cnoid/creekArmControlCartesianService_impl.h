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

  void setVelocity(double vtrans, double vomega, double velbow);
  void setTranslationVelocity(double vtrans);
  void setAngularVelocity(double vomega);
  void setElbowVelocity(double velbow);

  void setComponent (creekArmControlCartesian * i_comp) {
    m_comp = i_comp;
  }

private:
  creekArmControlCartesian * m_comp;
};



#endif // CREEK_ARMCONTROLCARTESIANSERVICE_IMPL_H

