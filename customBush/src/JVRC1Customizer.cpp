#include <cmath>
#include <cstring>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <cnoid/BodyCustomizerInterface>

#include <iostream>

cnoid::Matrix3 trans(const cnoid::Matrix3& M) { return M.transpose(); }
double dot(const cnoid::Vector3& a, const cnoid::Vector3& b) { return a.dot(b); }
typedef cnoid::Matrix3 Matrix33;


using namespace std;
using namespace boost;
using namespace cnoid;

BodyInterface* bodyInterface = 0;
BodyCustomizerInterface bodyCustomizerInterface;

struct JointValSet
{
  double* valuePtr;
  double* velocityPtr;
  double* torqueForcePtr;
};

struct JVRC1Customizer
{
  BodyHandle bodyHandle;

  bool hasVirtualBushJoints;
  JointValSet jointValSets[2][3];
  double springT;
  double dampingT;
  double springR;
  double dampingR;
};


static const char** getTargetModelNames()
{
  static const char* names[] = {
    "JVRC-TSML",
    0 };
  return names;
}


static void getVirtualbushJoints(JVRC1Customizer* customizer, BodyHandle body)
{
  customizer->hasVirtualBushJoints = true;

  int bushIndices[2][3];

  bushIndices[0][0] = bodyInterface->getLinkIndexFromName(body, "RLEG_BUSH_Z");
  bushIndices[0][1] = bodyInterface->getLinkIndexFromName(body, "RLEG_BUSH_ROLL");
  bushIndices[0][2] = bodyInterface->getLinkIndexFromName(body, "RLEG_BUSH_PITCH");
  bushIndices[1][0] = bodyInterface->getLinkIndexFromName(body, "LLEG_BUSH_Z");
  bushIndices[1][1] = bodyInterface->getLinkIndexFromName(body, "LLEG_BUSH_ROLL");
  bushIndices[1][2] = bodyInterface->getLinkIndexFromName(body, "LLEG_BUSH_PITCH");

  for(int i=0; i < 2; ++i){
    for(int j=0; j < 3; ++j){
      int bushIndex = bushIndices[i][j];
      if(bushIndex < 0){
        customizer->hasVirtualBushJoints = false;
      } else {
        JointValSet& jointValSet = customizer->jointValSets[i][j];
        jointValSet.valuePtr = bodyInterface->getJointValuePtr(body, bushIndex);
        jointValSet.velocityPtr = bodyInterface->getJointVelocityPtr(body, bushIndex);
        jointValSet.torqueForcePtr = bodyInterface->getJointForcePtr(body, bushIndex);
      }
    }
  }
}

static BodyCustomizerHandle create(BodyHandle bodyHandle, const char* modelName)
{
  JVRC1Customizer* customizer = 0;

  std::cerr << "create customizer : " << std::string(modelName) << std::endl;
  customizer = new JVRC1Customizer;

  customizer->bodyHandle = bodyHandle;
  customizer->hasVirtualBushJoints = false;

  // customizer->springT  = 1.0e6; // N/m
  // customizer->dampingT = 1.0e3; // N/(m/s)
  // customizer->springR  = 2.5e3; // Nm / rad
  // customizer->dampingR = 2.5;   // Nm / (rad/s)

  customizer->springT  = 1.3e6; // N/m
  customizer->dampingT = 1.0e3; // N/(m/s)
  customizer->springR  = 3e3; // Nm / rad
  customizer->dampingR = 2.5;   // Nm / (rad/s)

  getVirtualbushJoints(customizer, bodyHandle);

  return static_cast<BodyCustomizerHandle>(customizer);
}


static void destroy(BodyCustomizerHandle customizerHandle)
{
  JVRC1Customizer* customizer = static_cast<JVRC1Customizer*>(customizerHandle);
  if(customizer){
    delete customizer;
  }
}

static void setVirtualJointForces(BodyCustomizerHandle customizerHandle)
{
  JVRC1Customizer* customizer = static_cast<JVRC1Customizer*>(customizerHandle);

  if(customizer->hasVirtualBushJoints){

    for(int i=0; i < 2; ++i){
      JointValSet& trans = customizer->jointValSets[i][0];
      *(trans.torqueForcePtr) = - customizer->springT * (*trans.valuePtr) - customizer->dampingT * (*trans.velocityPtr);
      //std::cerr << i << " " << 0 << " " << *(trans.torqueForcePtr) << " = " << -customizer->springT << " x " << *trans.valuePtr << " + " <<  - customizer->dampingT << " x " << *trans.velocityPtr << std::endl;

      for(int j=1; j < 3; ++j){
        JointValSet& rot = customizer->jointValSets[i][j];
        *(rot.torqueForcePtr) = - customizer->springR * (*rot.valuePtr) - customizer->dampingR * (*rot.velocityPtr);
        //std::cerr << i << " " << j << " " << *(rot.torqueForcePtr) << " = " << -customizer->springR << " x " << *rot.valuePtr << " + " <<  - customizer->dampingR << " x " << *rot.velocityPtr << std::endl;
      }
    }
  }
}

CNOID_BODY_CUSTOMIZER_EXPORT
cnoid::BodyCustomizerInterface* getHrpBodyCustomizerInterface(cnoid::BodyInterface* bodyInterface_)
{
  bodyInterface = bodyInterface_;

  bodyCustomizerInterface.version = cnoid::BODY_CUSTOMIZER_INTERFACE_VERSION;
  bodyCustomizerInterface.getTargetModelNames = getTargetModelNames;
  bodyCustomizerInterface.create = create;
  bodyCustomizerInterface.destroy = destroy;
  bodyCustomizerInterface.initializeAnalyticIk = NULL;
  bodyCustomizerInterface.calcAnalyticIk = NULL;
  bodyCustomizerInterface.setVirtualJointForces = setVirtualJointForces;

  return &bodyCustomizerInterface;
}
