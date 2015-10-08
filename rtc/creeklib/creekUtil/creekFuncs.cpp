#include "creekFuncs.h"

#define USE_MATH_DEFINES
#include <cmath>

using namespace creek;

void creek::calcRodrigues(Matrix33& out_R, const Vector3& axis, double q)
{
  // E + a_hat*sin(q) + a_hat*a_hat*(1-cos(q))
  //
  //    |  0 -az  ay|
  // =E+| az   0 -ax|*s + a_hat*a_hat*v
  //    |-ay  ax   0|
  //
  //    |  0 -az  ay|     |-az*az-ay*ay        ax*ay        az*ax|
  // =E+| az   0 -ax|*s + |       ax*ay -az*az-ax*ax        ay*az|*v
  //    |-ay  ax   0|     |       az*ax        ay*az -ax*ax-ay*ay|
  //
  //  |1-az*az*v-ay*ay*v     -az*s+ax*ay*v      ay*s+az*ax*v|
  // =|     az*s+ax*ay*v 1-az*az*v-ax*ax*v     -ax*s+ay+az*v|
  //  |    -ay*s+az*ax*v      ax*s+ay*az*v 1-ax*ax*v-ay*ay*v|
  //
  
  const double sth = sin(q);
  const double vth = 1.0 - cos(q);
  
  double ax = axis(0);
  double ay = axis(1);
  double az = axis(2);
  
  const double axx = ax*ax*vth;
  const double ayy = ay*ay*vth;
  const double azz = az*az*vth;
  const double axy = ax*ay*vth;
  const double ayz = ay*az*vth;
  const double azx = az*ax*vth;
  
  ax *= sth;
  ay *= sth;
  az *= sth;
  
  out_R = 1.0 - azz - ayy, -az + axy,       ay + azx,
    az + axy,        1.0 - azz - axx, -ax + ayz,
    -ay + azx,       ax + ayz,        1.0 - ayy - axx;
}


Vector3 creek::omegaFromRot(const Matrix33& r)
{
  using ::std::numeric_limits;
    
  double alpha = (r(0,0) + r(1,1) + r(2,2) - 1.0) / 2.0;
  
  if(fabs(alpha - 1.0) < 1.0e-6) {
    return Vector3(0.0);
    
  }
  else {
    double th = acos(alpha);
    double s = sin(th);
    
    if (s < numeric_limits<double>::epsilon()) {
      return Vector3(0.0);
    }
    
    double k = -0.5 * th / s;
    
    return Vector3( (r(1,2) - r(2,1)) * k,
		    (r(2,0) - r(0,2)) * k,
		    (r(0,1) - r(1,0)) * k );
  }
}


Vector3 creek::rpyFromRot(const Matrix33& m)
{
  double roll, pitch, yaw;
    
  if ((fabs(m(0,0))<fabs(m(2,0))) && (fabs(m(1,0))<fabs(m(2,0)))) {
    // cos(p) is nearly = 0
    double sp = -m(2,0);
    if (sp < -1.0) {
      sp = -1;
    } else if (sp > 1.0) {
      sp = 1;
    }
    pitch = asin(sp); // -pi/2< p < pi/2
	
    roll = atan2(sp*m(0,1)+m(1,2),  // -cp*cp*sr*cy
		 sp*m(0,2)-m(1,1)); // -cp*cp*cr*cy
	
    if (m(0,0)>0.0) { // cy > 0
      (roll < 0.0) ? (roll += M_PI) : (roll -= M_PI);
    }
    double sr=sin(roll), cr=cos(roll);
    if (sp > 0.0) {
      yaw = atan2(sr*m(1,1)+cr*m(1,2), //sy*sp
		  sr*m(0,1)+cr*m(0,2));//cy*sp
    } else {
      yaw = atan2(-sr*m(1,1)-cr*m(1,2),
		  -sr*m(0,1)-cr*m(0,2));
    }
  } else {
    yaw = atan2(m(1,0), m(0,0));
    const double sa = sin(yaw);
    const double ca = cos(yaw);
    pitch = atan2(-m(2,0), ca*m(0,0)+sa*m(1,0));
    roll = atan2(sa*m(0,2)-ca*m(1,2), -sa*m(0,1)+ca*m(1,1));
  }
  return Vector3(roll, pitch, yaw);
}


double creek::intermediateYaw(const Matrix33& ma, const Matrix33& mb)
{
  Vector3 rpya( rpyFromRot(ma) );
  Vector3 rpyb( rpyFromRot(mb) );

  double yawa = rpya(2);
  double yawb = rpyb(2);

  double e = yawb-yawa;
  while( e > M_PI ) {
    e -= (2*M_PI);
  }
  while( e < -M_PI ) {
    e += (2*M_PI);
  }

  return (yawa + e/2.0);
}
