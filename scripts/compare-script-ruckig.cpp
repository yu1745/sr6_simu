// Host adapter: link to the firmware's production smoother/backend objects.
#include "tcode_trajectory_smoother.hpp"
#include <fstream>
#include <iostream>
#include <cstdlib>
int main(int argc,char** argv) {
    if(argc!=10)return 2;
    std::ifstream in(argv[1],std::ios::binary);std::ofstream out(argv[2],std::ios::binary);
    const int dt=std::atoi(argv[3]);
    TrajectorySmoothingLimits tr(std::atof(argv[4]),std::atof(argv[5]),std::atof(argv[6]));
    TrajectorySmoothingLimits ro(std::atof(argv[7]),std::atof(argv[8]),std::atof(argv[9]));
    TCodeTrajectorySmoother smoother;float target[6],p[6],state[18];size_t n=0,rejected=0;
    while(in.read(reinterpret_cast<char*>(target),sizeof(target))) {
        if(!smoother.apply(target,p,6,n*int64_t(dt),TrajectorySmoothingMode::Ruckig,tr,ro))++rejected;
        for(size_t i=0;i<6;++i){state[i]=p[i];state[6+i]=smoother.velocity(i);state[12+i]=smoother.acceleration(i);}
        out.write(reinterpret_cast<char*>(state),sizeof(state));++n;
    }
    const auto d=smoother.ruckigDiagnostics();
    std::cout<<"{\"samples\":"<<n<<",\"rejected\":"<<rejected<<",\"errors\":"<<d.errors
             <<",\"fallbacks\":"<<d.fallbacks<<",\"fallback_errors\":"<<d.fallback_errors<<"}\n";
    return !in.eof()||!out||rejected?1:0;
}
