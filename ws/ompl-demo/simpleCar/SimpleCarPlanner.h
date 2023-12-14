# pragma once
#include "AMPCore.h"
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/SimpleSetup.h>
#include "SimpleCar.h"

namespace ob = ompl::base;
namespace oc = ompl::control;

class SimpleCarPlanner{
public:
    // constructor and destructor:
    SimpleCarPlanner();
    ~SimpleCarPlanner();

    // fields:
    std::vector<double> safetyMargin_ = {0.0, 0.0, 0.0, 0.0}; // safety margin for: x, y, v, phi
    ob::RealVectorBounds bounds_; // 4 dimensional state space. x, y, v, phi

    // methods:
    amp::Path2D planKinodynamic(const amp::Problem2D& prob, const std::vector<double>& safetyMargin);
    oc::SimpleSetupPtr kinodynamicSimpleSetUp(const amp::Problem2D *prob);
    oc::SimpleSetupPtr LTLsimpleSetUp(const amp::Problem2D *prob);

    void planLTLTest(amp::Problem2D& prob);
    void planLTL(amp::Problem2D& prob, const std::vector<double>& safetyMargin);
    
    ob::StateSpacePtr createStateSpace(const amp::Problem2D& prob, const SimpleCar& car);
    oc::ControlSpacePtr createControlSpace(ob::StateSpacePtr &space);

};