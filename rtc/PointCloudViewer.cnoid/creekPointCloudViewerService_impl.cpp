// -*-C++-*-

#include "creekPointCloudViewerService_impl.h"
#include "creekPointCloudViewer.h"

creekPointCloudViewerService_impl::creekPointCloudViewerService_impl()
  : m_comp(NULL)
{
}

creekPointCloudViewerService_impl::~creekPointCloudViewerService_impl()
{
}

void creekPointCloudViewerService_impl::start()
{
  if( m_comp != NULL )
    m_comp->start();
}

void creekPointCloudViewerService_impl::stop()
{
  if( m_comp != NULL )
    m_comp->stop();
}


bool creekPointCloudViewerService_impl::detectLandingPoint(double x, double y, double w, int ft)
{
  if( m_comp != NULL )
    return m_comp->detectLandingPoint(x,y,w,ft);
  else
    return false;
}


void creekPointCloudViewerService_impl::getLandingPoint(double &x, double &y, double &z, double &r, double &p, double &w, int ft)
{
  if( m_comp != NULL )
    return m_comp->getLandingPoint(x,y,z,r,p,w,ft);
}


void creekPointCloudViewerService_impl::test()
{
  if( m_comp != NULL )
    m_comp->test();
}


void creekPointCloudViewerService_impl::changeMode()
{
  if( m_comp != NULL )
    m_comp->changeMode();
}


bool creekPointCloudViewerService_impl::autoFittinSwitch()
{
  if( m_comp != NULL )
    m_comp->autoFittinSwitch();
}


void creekPointCloudViewerService_impl::clear()
{
  if( m_comp != NULL )
    m_comp->clear();
}

