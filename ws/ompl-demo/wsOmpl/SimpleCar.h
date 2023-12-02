# pragma once
# include "AMPCore.h"
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/sst/SST.h>

namespace ob = ompl::base;
namespace oc = ompl::control;

class SimpleCar{

    public:
        // constructor and destructor:
        SimpleCar(std::string name, std::string dyn, std::vector<double> shape, std::vector<double> s, std::vector<double> g, double x_min, double x_max, double y_min, double y_max);
        ~SimpleCar();
        std::string name_;
        std::string dynamics_;
        std::vector<double> shape_;
        std::vector<double> start_;
        std::vector<double> goal_;
        double phi_min_;
        double phi_max_;
        double v_min_;
        double v_max_;
};