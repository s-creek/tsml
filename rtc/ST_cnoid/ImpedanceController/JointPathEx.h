#ifndef __JOINT_PATH_EX_H__
#define __JOINT_PATH_EX_H__
//#include <hrpModel/Body.h>
//#include <hrpModel/Link.h>
//#include <hrpModel/JointPath.h>

#include <cnoid/Body>
#include<cnoid/BodyLoader>
#include<cnoid/VRMLBodyLoader>
#include <cnoid/EigenTypes>
#include <cnoid/JointPath>
#include <cnoid/Jacobian>
#include <cnoid/EigenUtil>
#include <cnoid/Sensor>
#include <cnoid/Link>

using namespace cnoid;

// hrplib/hrpUtil/MatrixSolvers.h
namespace hrp {
    int calcSRInverse(const MatrixXd& _a, MatrixXd &_a_sr, double _sr_ratio = 1.0, MatrixXd _w = MatrixXd::Identity(0,0));
};

// hrplib/hrpModel/JointPath.h
namespace hrp {
    class JointPathEx : public JointPath {
  public:
    JointPathEx(BodyPtr& robot, Link* base, Link* end);
    bool calcJacobianInverseNullspace(MatrixXd &J, MatrixXd &Jinv, MatrixXd &Jnull);
    bool calcInverseKinematics2Loop(const Vector3& dp, const Vector3& omega, const double LAMBDA, const double avoid_gain = 0.0, const double reference_gain = 0.0, const VectorXd* reference_q = NULL);
    
    //bool calcInverseKinematics2(const Vector3& end_p, const Matrix3& end_R, const double avoid_gain = 0.0, const double reference_gain = 0.0, const dvector* reference_q = NULL);
    double getSRGain() { return sr_gain; }
    bool setSRGain(double g) { sr_gain = g; }
    double getManipulabilityLimit() { return manipulability_limit; }
    bool setManipulabilityLimit(double l) { manipulability_limit = l; }
    bool setManipulabilityGain(double l) { manipulability_gain = l; }
    void setMaxIKError(double epos, double erot);
    void setMaxIKError(double e);
    void setMaxIKIteration(int iter);
    
  protected:                                      //add by wu
    double maxIKPosErrorSqr, maxIKRotErrorSqr, maxIKErrorSqr;
        int maxIKIteration;
        std::vector<Link*> joints;
        std::vector<double> avoid_weight_gain;
	double sr_gain, manipulability_limit, manipulability_gain;
    };

    typedef boost::shared_ptr<JointPathEx> JointPathExPtr;

};

#include <iomanip>

#endif //__JOINT_PATH_EX_H__
