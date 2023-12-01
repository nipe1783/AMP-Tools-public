#include "AMPCore.h"
#include "hw/HW2.h"
#include <yaml-cpp/yaml.h>
#include <vector>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/sst/SST.h>
#include "plannerOmpl/PlannerOmpl.h"

using namespace amp;
namespace fs = std::filesystem;
namespace ob = ompl::base;
namespace og = ompl::geometric;

int main(int argc, char** argv) {

    Problem2D prob;
    amp::Path2D path;
    PlannerOmpl planner;

    prob = HW2::getWorkspace1();
    path = planner.plan(prob);
    Visualizer::makeFigure(prob, path);

    prob = HW2::getWorkspace2();
    path = planner.plan(prob);
    Visualizer::makeFigure(prob, path);
    Visualizer::showFigures();

    return 0;
}