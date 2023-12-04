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
    double safetyMargin_ = 0.0;

    // methods:
    amp::Path2D planKinodynamic(const amp::Problem2D& prob, const double& safetyMargin = 0.0);
    oc::SimpleSetupPtr kinodynamicSimpleSetUp(const amp::Problem2D *prob);
    ob::StateSpacePtr createStateSpace(const amp::Problem2D& prob, const SimpleCar& car);
    oc::ControlSpacePtr createControlSpace(ob::StateSpacePtr &space);
};