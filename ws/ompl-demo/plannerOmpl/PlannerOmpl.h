#  pragma once

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/sst/SST.h>
#include "../wsOmpl/StateValidityCheckerOmpl.h"
#include "AMPCore.h"

namespace fs = std::filesystem;
namespace ob = ompl::base;
namespace og = ompl::geometric;

class PlannerOmpl {

    public:
        // methods:
        amp::Path2D planGeometric(const amp::Problem2D& prob);
        std::tuple<amp::Path2D, double, double> planGeometric(const amp::Problem2D& prob, bool verbose);
        og::SimpleSetupPtr geometricSimpleSetUp(const amp::Problem2D *prob);
};