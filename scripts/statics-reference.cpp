// Compile against unmodified firmware sources to check signs, units and allocation.
#include <iostream>
#include <iomanip>
#include <cmath>
#include "ik_static_analysis.hpp"
int main() {
  double x,y,z,roll,pitch,twist,weight,ax,ay,az;
  while(std::cin>>x>>y>>z>>roll>>pitch>>twist>>weight>>ax>>ay>>az) {
    ik_static_analysis::Pose p;
    p.x=x*100;p.y=y*100;p.z=z*100;
    p.roll=roll*18000/M_PI;p.pitch=pitch*18000/M_PI;p.twist=twist;
    ik_static_analysis::Forces gravity,inertia;
    const double mass=weight/9.80665;
    const bool g=ik_static_analysis::computeCenteredPayloadFeedforward(p,mass,gravity);
    ik_static_analysis::Wrench f;
    f.Fx=mass*ax*1e-5;f.Fy=mass*ay*1e-5;f.Fz=mass*az*1e-5;
    const bool a=ik_static_analysis::staticForceAnalysis(p,f,inertia);
    std::cout<<std::setprecision(12)<<g<<' '<<a;
    for(float v:gravity.f)std::cout<<' '<<v;
    for(float v:inertia.f)std::cout<<' '<<v;
    std::cout<<'\n';
  }
}
