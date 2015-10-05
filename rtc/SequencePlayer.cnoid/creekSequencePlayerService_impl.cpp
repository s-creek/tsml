// -*-C++-*-

#include "creekSequencePlayerService_impl.h"
#include "creekSequencePlayer.h"

/*
 * Example implementational code for IDL interface OpenHRP::creekSequencePlayerService
 */
creekSequencePlayerService_impl::creekSequencePlayerService_impl()
  : m_comp(NULL)
{
}


creekSequencePlayerService_impl::~creekSequencePlayerService_impl()
{
}


void creekSequencePlayerService_impl::waitInterpolation()
{
  return m_comp->waitInterpolation();
}


::CORBA::Boolean creekSequencePlayerService_impl::setJointAngles(const OpenHRP::dSequence& jvs, ::CORBA::Double tm)
{
  if( jvs.length() != m_comp->dof() ) {
    std::cout << "creekSequencePlayerService : error input num=" << jvs.length() << ", robot dof = " << m_comp->dof() << std::endl;
    return false;
  }
  return m_comp->setJointAngles(jvs.get_buffer(), tm);
}

::CORBA::Boolean creekSequencePlayerService_impl::setJointAngle(const char* jname, ::CORBA::Double jv, ::CORBA::Double tm)
{
  return m_comp->setJointAngle(jname, jv, tm);
}

::CORBA::Boolean creekSequencePlayerService_impl::setBasePos(const OpenHRP::dSequence& pos, ::CORBA::Double tm)
{
  if( pos.length() != 3 ) return false;
  return m_comp->setBasePos(pos.get_buffer(), tm);
}

::CORBA::Boolean creekSequencePlayerService_impl::setBaseRpy(const OpenHRP::dSequence& rpy, ::CORBA::Double tm)
{
  if( rpy.length() != 3 ) return false;
  return m_comp->setBaseRpy(rpy.get_buffer(), tm);
}

::CORBA::Boolean creekSequencePlayerService_impl::setZmp(const OpenHRP::dSequence& zmp, ::CORBA::Double tm)
{
  if( zmp.length() != 3 ) return false;
  return m_comp->setZmp(zmp.get_buffer(), tm);
}

::CORBA::Boolean creekSequencePlayerService_impl::isEmpty()
{
  return m_comp->isEmpty();
}


void creekSequencePlayerService_impl::jointCalib(int scale)
{
  m_comp->jointCalib(scale);
}


::CORBA::Boolean creekSequencePlayerService_impl::setBasePosRel(const OpenHRP::dSequence& pos, ::CORBA::Double tm)
{
  if( pos.length() != 3 ) return false;
  return m_comp->setBasePosRel(pos.get_buffer(), tm);
}

// End of example implementational code



