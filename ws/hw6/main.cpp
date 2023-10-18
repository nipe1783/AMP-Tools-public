// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW2.h"
#include "hw/HW5.h"
#include "hw/HW6.h"

// Include files
#include "MyPointWaveFrontAlgorithm.h"
#include "MyManipulatorWaveFrontAlgorithm.h"
#include "MyConfigurationSpace.h"
#include "MyLinkManipulator.h"
#include "MyGridCSpace2DConstructor.h"

using namespace amp;

int main(int argc, char** argv) {
    
    // problem 1: workspace 1
    // Problem2D problem = HW2::getWorkspace2();
    // MyPointWaveFrontAlgorithm algo1;
    // Path2D path = algo1.plan(problem);
    // std::unique_ptr<amp::GridCSpace2D> cSpace_c = algo1.constructDiscretizedWorkspace(problem);
    // Visualizer::makeFigure(*cSpace_c);
    // Visualizer::makeFigure(problem, path);
    // amp::Visualizer::showFigures();
    

    // problem 2: HW 4 WS 1

    // make robot
    std::vector<double> link_lengths_3 = {1.0, 1.0};
    std::vector<double> state_3 = {0.4 * 2 *M_PI, 0.8 * 2 *M_PI};
    amp::MyLinkManipulator manipulator_3(link_lengths_3);
    ManipulatorState q_init = manipulator_3.getConfigurationFromIK(Eigen::Vector2d(-2, 0));
    ManipulatorState q_goal = manipulator_3.getConfigurationFromIK(Eigen::Vector2d(2, 0));
    MyManipulatorWaveFrontAlgorithm algo2;
    amp::MyGridCSpace2DConstructor configurationSpaceConstructor;

    // env a:
    amp::Environment2D environment_a;
    amp::Obstacle2D obstacle_a;
    obstacle_a.verticesCCW().push_back(Eigen::Vector2d(0.25, 0.25));
    obstacle_a.verticesCCW().push_back(Eigen::Vector2d(0, 0.75));
    obstacle_a.verticesCCW().push_back(Eigen::Vector2d(-0.25, 0.25)); 
    environment_a.obstacles.push_back(obstacle_a);

    // planner env a:
    std::unique_ptr<amp::GridCSpace2D> grid_a = configurationSpaceConstructor.construct(manipulator_3, environment_a);
    amp::Path2D path_a = algo2.planInCSpace(Eigen::Vector2d(q_init[0], q_init[1]), Eigen::Vector2d(q_goal[0], q_goal[1]), *grid_a);
    

    
    std::unique_ptr<amp::GridCSpace2D> cSpace_b = configurationSpaceConstructor.construct(manipulator_3, environment_a);
    
    amp::Visualizer::makeFigure(environment_a, manipulator_3, state_3);
    amp::Visualizer::makeFigure(*cSpace_b);
    amp::Visualizer::showFigures();

    // HW6::grade(algo, "nipe1783@colorado.edu", argc, argv);
    return 0;
}