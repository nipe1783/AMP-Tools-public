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
#include "simpleCar/SimpleCarPlanner.h"

using namespace amp;
namespace fs = std::filesystem;
namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace oc = ompl::control;

int main(int argc, char** argv) {

    ompl::msg::setLogLevel(ompl::msg::LOG_INFO);

    Problem2D prob;
    amp::Path2D path;
    PlannerOmpl planner;
    std::string title;

    // prob = HW2::getWorkspace1();
    // path = planner.planGeometric(prob);
    // Visualizer::makeFigure(prob, path);

    // prob = HW2::getWorkspace1();
    // path = planner.planGeometric(prob);
    // Visualizer::makeFigure(prob, path);

    // prob = HW2::getWorkspace2();
    // path = planner.planGeometric(prob);
    // Visualizer::makeFigure(prob, path);
    

    // computing stats for OMPL planner to compare to the amp RRT planner I made:
    // std::system("export OMPL_INFORM=false"); // setting ompl messages off
    // int iterations = 100;

    // std::vector<std::vector<double>> allTimesRRT;
    // std::vector<std::vector<double>> allLengthsRRT;
    // std::vector<std::vector<double>> allNumNodesRRT;
    // std::vector<std::vector<bool>> allSuccessRRT;

    // // workspace 1:
    // prob = HW2::getWorkspace1();
    // std::tuple<std::vector<double>, std::vector<double>, std::vector<double>, std::vector<bool>> results = Benchmark::runBenchmark(prob, planner, iterations);
    // allTimesRRT.push_back(std::get<0>(results));
    // allLengthsRRT.push_back(std::get<1>(results));
    // allNumNodesRRT.push_back(std::get<2>(results));
    // allSuccessRRT.push_back(std::get<3>(results));

    // // workspace 2:
    // prob = HW2::getWorkspace2();
    // results = Benchmark::runBenchmark(prob, planner, iterations);
    // allTimesRRT.push_back(std::get<0>(results));
    // allLengthsRRT.push_back(std::get<1>(results));
    // allNumNodesRRT.push_back(std::get<2>(results));
    // allSuccessRRT.push_back(std::get<3>(results));

    // std::list<std::vector<double>> allTimesRRTList(allTimesRRT.begin(), allTimesRRT.end());
    // std::list<std::vector<double>> allLengthsRRTList(allLengthsRRT.begin(), allLengthsRRT.end());
    // std::list<std::vector<double>> allNumNodesRRTList(allNumNodesRRT.begin(), allNumNodesRRT.end());
    // std::list<std::vector<bool>> allSuccessRRTList(allSuccessRRT.begin(), allSuccessRRT.end());

    // title = "OMPL RRT Benchmark results for AMP HW2 workspaces";
    // Visualizer::makeBoxPlot(allTimesRRTList, {"WS1", "WS2"}, title, "WS", "Time (Seconds)");

    // title = "OMPL RRT Benchmark results for AMP HW2 workspaces";
    // Visualizer::makeBoxPlot(allLengthsRRTList, {"WS1", "WS2"}, title, "WS", "Path Length");

    // title = "OMPL RRT Benchmark results for AMP HW2 workspaces";
    // Visualizer::makeBoxPlot(allNumNodesRRTList, {"WS1", "WS2"}, title, "WS", "Number of Nodes");

    // amp::HW7::grade<GradePlanner, GradePlanner>("nipe1783@colorado.edu", argc, argv);
    
    SimpleCarPlanner carPlanner;
    // prob = HW2::getWorkspace1();
    // path = carPlanner.planKinodynamic(prob);
    // Visualizer::makeFigure(prob, path);


    // Custom prob
    prob = HW5::getWorkspace1();
    std::vector<Eigen::Vector2d> vertices = {Eigen::Vector2d(6, 1), Eigen::Vector2d(6, 3), Eigen::Vector2d(6.25, 3), Eigen::Vector2d(6.25, 1)};
    amp::Obstacle2D obs(vertices);
    prob.obstacles[0] = obs;
    std::vector<Eigen::Vector2d> vertices2 = {Eigen::Vector2d(6, -1), Eigen::Vector2d(6, -3), Eigen::Vector2d(6.25, -3), Eigen::Vector2d(6.25, -1)};
    amp::Obstacle2D obs2(vertices2);
    prob.obstacles[1] = obs2;
    path = carPlanner.planKinodynamic(prob, {1.5, 1.5, .2, .5});
    Visualizer::makeFigure(prob, path);

    Visualizer::showFigures();

    return 0;
}