// -*-C++-*-
#ifndef IMPEDANCESERVICESVC_IMPL_H
#define IMPEDANCESERVICESVC_IMPL_H

#include "ImpedanceControllerService.hh"

using namespace OpenHRP;

class ImpedanceController;

class ImpedanceControllerService_impl 
  : public virtual POA_OpenHRP::ImpedanceControllerService,
    public virtual PortableServer::RefCountServantBase
{
public:
  ImpedanceControllerService_impl();
  virtual ~ImpedanceControllerService_impl();
  //
  CORBA::Boolean setImpedanceControllerParam(const char *i_name_, const OpenHRP::ImpedanceControllerService::impedanceParam &i_param_);
  CORBA::Boolean getImpedanceControllerParam(const char *i_name_, OpenHRP::ImpedanceControllerService::impedanceParam_out i_param_);
  CORBA::Boolean deleteImpedanceController(const char *i_name_);
  void waitDeletingImpedanceController(const char *i_name_);
  CORBA::Boolean deleteImpedanceControllerAndWait(const char *i_name_);
  //
  void impedance(ImpedanceController *i_impedance);
private:
  ImpedanceController *m_impedance;
};				 

#endif
