// -*-C++-*-

#include "creekCameraViewerService_impl.h"
#include "creekCameraViewer.h"

creekCameraViewerService_impl::creekCameraViewerService_impl()
  : m_draw(true),
    m_comp(NULL)
{
}


creekCameraViewerService_impl::~creekCameraViewerService_impl()
{
}


void creekCameraViewerService_impl::setDraw()
{
  m_draw != m_draw;
}


void creekCameraViewerService_impl::setSearchFlag(const char *list)
{
  if( m_comp ) {
    std::string listStr(list);
    m_comp->setSearchFlag(listStr);
  }
}


void creekCameraViewerService_impl::show()
{
  if( m_comp )
    m_comp->show();
}


void creekCameraViewerService_impl::saveData(const char *path)
{
   if( m_comp )
    m_comp->saveData(path);
}


// End of example implementational code



