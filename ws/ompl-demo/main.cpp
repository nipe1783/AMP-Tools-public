#include "AMPCore.h"
#include "hw/HW2.h"
#include <yaml-cpp/yaml.h>
#include <vector>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/sst/SST.h>
#include "plannerOmpl/PlannerOmpl.h"
#include "hw/HW5.h"
#include "hw/HW7.h"
#include "benchmark/Benchmark.h"
#include "plannerOmpl/GradePlanner.h"

using namespace amp;
namespace fs = std::filesystem;
namespace ob = ompl::base;
namespace og = ompl::geometric;

int main(int argc, char** argv) {

    Problem2D prob;
    amp::Path2D path;
    PlannerOmpl planner;
    std::string title;

    // prob = HW5::getWorkspace1();
    // path = planner.planGeometric(prob);

    prob = HW2::getWorkspace1();
    path = planner.planGeometric(prob);
    Visualizer::makeFigure(prob, path);

    // prob = HW2::getWorkspace2();
    // path = planner.planGeometric(prob);
    // Visualizer::makeFigure(prob, path);
    

    // computing stats for OMPL planner to compare to the amp RRT planner I made:
    // std::system("export OMPL_INFORM=false"); // setting ompl messages off
    // int iterations = 100;

    // std::vector<std::vector<double>> allTimesRRT;
    // std::vector<std::vector<double>> allLengthsRRT;
    // std::vector<std::vector<bool>> allSuccessRRT;

    // // workspace 1:
    // prob = HW2::getWorkspace1();
    // std::tuple<std::vector<double>, std::vector<double>, std::vector<bool>> results = Benchmark::runBenchmark(prob, planner, iterations);
    // allTimesRRT.push_back(std::get<0>(results));
    // allLengthsRRT.push_back(std::get<1>(results));
    // allSuccessRRT.push_back(std::get<2>(results));

    // std::list<std::vector<double>> allTimesRRTList(allTimesRRT.begin(), allTimesRRT.end());

    // title = "OMPL RRT Benchmark results for AMP HW workspaces";
    // Visualizer::makeBoxPlot(allTimesRRTList, {"WS1"}, title, "WS", "Time");

    Visualizer::showFigures();

    // amp::HW7::grade<GradePlanner, GradePlanner>("nipe1783@colorado.edu", argc, argv);
    

    return 0;
}