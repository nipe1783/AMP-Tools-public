# pragma once
#include "AMPCore.h"
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/SimpleSetup.h>
#include "../wsOmpl/SimpleCar.h"

namespace ob = ompl::base;
namespace oc = ompl::control;

class SimpleCarPlanner{
public:
    // constructor and destructor:
    SimpleCarPlanner();
    ~SimpleCarPlanner();

    // methods:
    void test();
    amp::Path2D planKinodynamic(const amp::Problem2D& prob);
    oc::SimpleSetupPtr kinodynamicSimpleSetUp(const amp::Problem2D *prob);
    ob::StateSpacePtr createStateSpace(const amp::Problem2D& prob, const SimpleCar& car);
    oc::ControlSpacePtr createControlSpace(ob::StateSpacePtr &space);
};