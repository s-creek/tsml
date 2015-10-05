// -*- c++ -*-

#include "creekArmControlCartesianService_impl.h"
#include "creekArmControlCartesian.h"

creekArmControlCartesianService_impl::creekArmControlCartesianService_impl()
  : m_comp(NULL)
{
}

creekArmControlCartesianService_impl::~creekArmControlCartesianService_impl()
{
}


void creekArmControlCartesianService_impl::setArm(int armId)
{
  if(m_comp)
    m_comp->setArm(armId);
}


void creekArmControlCartesianService_impl::setVelocity(double vtrans, double vomega, double velbow)
{
  if(m_comp) {
    m_comp->setTranslationVelocity(vtrans);
    m_comp->setAngularVelocity(vomega);
    m_comp->setElbowVelocity(velbow);
  }
}


void creekArmControlCartesianService_impl::setTranslationVelocity(double vtrans)
{
  if(m_comp)
    m_comp->setTranslationVelocity(vtrans);
}


void creekArmControlCartesianService_impl::setAngularVelocity(double vomega)
{
 if(m_comp)
    m_comp->setAngularVelocity(vomega);
}


void creekArmControlCartesianService_impl::setElbowVelocity(double velbow)
{
 if(m_comp)
    m_comp->setElbowVelocity(velbow);
}
