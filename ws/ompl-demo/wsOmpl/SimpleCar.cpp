#include "SimpleCar.h"
#include "AMPCore.h"
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/sst/SST.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>

namespace ob = ompl::base;

SimpleCar::SimpleCar(std::string name, std::string dyn, std::vector<double> shape, std::vector<double> s, std::vector<double> g, double x_min, double x_max, double y_min, double y_max) {
    name_ = name;
    dynamics_ = dyn;
    std::cout<<"DYNAMICS: "<<dynamics_<<std::endl;
    shape_ = shape;
    start_ = s;
    goal_ = g;
    phi_min_ = -M_PI/10;
    phi_max_ = M_PI/10;
    v_min_ = 0;
    v_max_ = 1;
}