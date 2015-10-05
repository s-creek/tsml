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
