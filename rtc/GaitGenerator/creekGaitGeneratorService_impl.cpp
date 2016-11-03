// -*-C++-*-

#include "creekGaitGeneratorService_impl.h"
#include "creekGaitGenerator.h"

creekGaitGeneratorService_impl::creekGaitGeneratorService_impl()
  : m_comp(NULL)
{
}


creekGaitGeneratorService_impl::~creekGaitGeneratorService_impl()
{
}


void creekGaitGeneratorService_impl::setTargetPos(::CORBA::Double x, ::CORBA::Double y, ::CORBA::Double th, ::CORBA::Double time)
{
}


void creekGaitGeneratorService_impl::startStepping()
{
  if( m_comp != NULL)
    m_comp->startStepping();
}


void creekGaitGeneratorService_impl::stopStepping()
{
  if( m_comp != NULL)
    m_comp->stopStepping();
}

void creekGaitGeneratorService_impl::test()
{
  if( m_comp != NULL)
    m_comp->test();
}


// End of example implementational code



