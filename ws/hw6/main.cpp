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
#include "MyAstarAlgorithm.h"

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
    
    // Random2DEnvironmentSpecification spec;
    // spec.max_obstacle_region_radius = 1;
    // spec.n_obstacles = 20;
    // spec.path_clearance = 0.01;
    // spec.d_sep = 5;
    // //Randomly generate the environment;
    // Problem2D problem = EnvironmentTools::generateRandomPointAgentProblem(spec, 2); // Random environment
    // MyPointWaveFrontAlgorithm algo1;
    // Path2D path = algo1.plan(problem);
    // std::unique_ptr<amp::GridCSpace2D> cSpace_c = algo1.constructDiscretizedWorkspace(problem);
    // Visualizer::makeFigure(*cSpace_c);
    // Visualizer::makeFigure(problem, path);
    // amp::Visualizer::showFigures();

    // // problem 2: HW 4 WS 1
    //     // make robot
    //     std::vector<double> link_lengths = {1.0, 1.0};
    //     Eigen::Vector2d state = {0.4 * 2 *M_PI, 0.8 * 2 *M_PI};
    //     amp::MyLinkManipulator manipulator(link_lengths);
    //     ManipulatorState q_init = manipulator.getConfigurationFromIK(Eigen::Vector2d(-2, 0));
    //     ManipulatorState q_goal = manipulator.getConfigurationFromIK(Eigen::Vector2d(2, 0));
    //     MyManipulatorWaveFrontAlgorithm algo2;
    //     amp::MyGridCSpace2DConstructor configurationSpaceConstructor;

    //     // env a:
    //         amp::Problem2D problem_a = HW6::getHW4Problem1();
    //         std::unique_ptr<amp::GridCSpace2D> grid_a = configurationSpaceConstructor.construct(manipulator, problem_a);
    //         amp::Path2D path_a = algo2.planInCSpace(Eigen::Vector2d(q_init[0], q_init[1]), Eigen::Vector2d(q_goal[0], q_goal[1]), *grid_a);
    //         amp::Visualizer::makeFigure(*grid_a, path_a);
    //         amp::Visualizer::makeFigure(problem_a, manipulator, path_a);

    //     // env b:
    //         amp::Problem2D problem_b = HW6::getHW4Problem2();
    //         std::unique_ptr<amp::GridCSpace2D> grid_b = configurationSpaceConstructor.construct(manipulator, problem_b);
    //         amp::Path2D path_b = algo2.planInCSpace(Eigen::Vector2d(q_init[0], q_init[1]), Eigen::Vector2d(q_goal[0], q_goal[1]), *grid_b);
    //         amp::Visualizer::makeFigure(*grid_b, path_b);
    //         amp::Visualizer::makeFigure(problem_b, manipulator, path_b);

    //     // env c:
    //         // planner env c:
    //         amp::Problem2D problem_c = HW6::getHW4Problem3();
    //         std::unique_ptr<amp::GridCSpace2D> grid_c = configurationSpaceConstructor.construct(manipulator, problem_c);
    //         amp::Path2D path_c = algo2.planInCSpace(Eigen::Vector2d(q_init[0], q_init[1]), Eigen::Vector2d(q_goal[0], q_goal[1]), *grid_c);
    //         amp::Visualizer::makeFigure(*grid_c, path_c);
    //         amp::Visualizer::makeFigure(problem_c, manipulator, path_c);

    //     bool prob_2_a = HW6::checkLinkManipulatorPlan(path_a, manipulator, problem_a);

    // prob 3
        // MyAStarAlgorithm algo3;
        // amp::ShortestPathProblem problem = HW6::getEx3SPP();
        // amp::LookupSearchHeuristic heuristic = HW6::getEx3Heuristic();
        // AStar::GraphSearchResult result = algo3.search(problem, heuristic);
        // bool prob_3 = HW6::checkGraphSearchResult(result, problem, heuristic);
        // bool prob_3_test = HW6::generateAndCheck(algo3, true, 1);


    // amp::Visualizer::showFigures();
    // std::cout<<"Problem 2 a: "<<prob_2_a<<std::endl;
    amp::HW6::grade<MyPointWaveFrontAlgorithm, MyManipulatorWaveFrontAlgorithm, MyAStarAlgorithm>("nipe1783@colorado.edu", argc, argv, std::make_tuple(), std::make_tuple("hey therre"), std::make_tuple());
    return 0;
}