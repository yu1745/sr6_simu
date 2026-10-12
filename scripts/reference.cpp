// Read-only independent reference: compile against original firmware sources.
#include <iostream>
#include <iomanip>
#include "ik_static_analysis.hpp"
#include "sr6_forward_kinematics.hpp"
int main() {
  double x,y,z,roll,pitch,twist;
  while(std::cin>>x>>y>>z>>roll>>pitch>>twist) {
    ik_static_analysis::Pose p;
    p.x=x*100;p.y=y*100;p.z=z*100;p.roll=roll*18000/M_PI;p.pitch=pitch*18000/M_PI;p.twist=twist;
    ik_static_analysis::Joints joints;
    ik_static_analysis::computeIK(p,joints);
    sr6_forward_kinematics::Joints j; j.q=joints.q;
    auto f=sr6_forward_kinematics::solve(j,twist,{});
    std::cout<<std::setprecision(12);
    for(auto a:j.q)std::cout<<a<<' ';
    std::cout<<f.pose.x/100<<' '<<f.pose.y/100<<' '<<f.pose.z/100<<' '
      <<f.pose.roll*M_PI/18000<<' '<<f.pose.pitch*M_PI/18000<<' '<<f.converged<<'\n';
  }
}
