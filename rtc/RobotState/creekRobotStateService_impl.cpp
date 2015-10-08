// -*-C++-*-

#include "creekRobotStateService_impl.h"
#include "creekRobotState.h"

creekRobotStateService_impl::creekRobotStateService_impl()
  : m_comp(NULL)
{
}


creekRobotStateService_impl::~creekRobotStateService_impl()
{
}


void creekRobotStateService_impl::test()
{
}


void creekRobotStateService_impl::showState()
{
  if( m_comp )
    m_comp->showState();
}


void creekRobotStateService_impl::logStart(const char *date)
{
  if( m_comp )
    m_comp->logStart(date);
}

// End of example implementational code
