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

bool creekPointCloudViewerService_impl::stop()
{
  if( m_comp != NULL )
    return m_comp->stop();
  else
    return false;
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


void creekPointCloudViewerService_impl::clearWorld()
{
  if( m_comp != NULL )
    m_comp->clearWorld();
}


void creekPointCloudViewerService_impl::clearCloud()
{
  if( m_comp != NULL )
    m_comp->clearCloud();
}


void creekPointCloudViewerService_impl::detectModeOn()
{
  if( m_comp != NULL )
    m_comp->detectModeOn();
}


void creekPointCloudViewerService_impl::detectModeOff()
{
  if( m_comp != NULL )
    m_comp->detectModeOff();
}

void creekPointCloudViewerService_impl::save(const char *name)
{
  if( m_comp != NULL )
    m_comp->save(name);
}


bool creekPointCloudViewerService_impl::matchingMap()
{
  if( m_comp != NULL )
    return m_comp->matchingMap();
  else
    return false;
}


void creekPointCloudViewerService_impl::updateMap()
{
  if( m_comp != NULL )
    m_comp->updateMap();
}
