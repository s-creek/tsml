// -*-C++-*-
/*!
 * @file  creekGaitGeneratorService_impl.h
 * @brief Service implementation header of creekGaitGeneratorService.idl
 *
 */

#include "creekGaitGeneratorService.hh"


#ifndef CREEKGAITGENERATORSERVICE_IMPL_H
#define CREEKGAITGENERATORSERVICE_IMPL_H
 
/*
 * Example class implementing IDL interface OpenHRP::creekGaitGeneratorService
 */
class creekGaitGeneratorService_impl
 : public virtual POA_OpenHRP::creekGaitGeneratorService,
   public virtual PortableServer::RefCountServantBase
{
 private:
   // Make sure all instances are built on the heap by making the
   // destructor non-public
   //virtual ~creekGaitGeneratorService_impl();

 public:
   // standard constructor
   creekGaitGeneratorService_impl();
   virtual ~creekGaitGeneratorService_impl();

   // attributes and operations
   void setTargetPos(::CORBA::Double x, ::CORBA::Double y, ::CORBA::Double th, ::CORBA::Double time);
   void startStepping();
   void stopStepping();
   void test();

};



#endif // CREEKGAITGENERATORSERVICE_IMPL_H


