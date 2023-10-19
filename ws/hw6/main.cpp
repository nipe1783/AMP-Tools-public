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
    //         amp::Problem2D problem_a;
    //         amp::Obstacle2D obstacle_a;
    //         obstacle_a.verticesCCW().push_back(Eigen::Vector2d(0.25, 0.25));
    //         obstacle_a.verticesCCW().push_back(Eigen::Vector2d(0, 0.75));
    //         obstacle_a.verticesCCW().push_back(Eigen::Vector2d(-0.25, 0.25)); 
    //         problem_a.obstacles.push_back(obstacle_a);
    //         // planner env a:
    //         std::unique_ptr<amp::GridCSpace2D> grid_a = configurationSpaceConstructor.construct(manipulator, problem_a);
    //         amp::Path2D path_a = algo2.planInCSpace(Eigen::Vector2d(q_init[0], q_init[1]), Eigen::Vector2d(q_goal[0], q_goal[1]), *grid_a);
    //         amp::Visualizer::makeFigure(*grid_a, path_a);
    //         amp::Visualizer::makeFigure(problem_a, manipulator, path_a);

    //     // env b:
    //         amp::Problem2D problem_b;
    //         amp::Obstacle2D obstacle_b1;
    //         obstacle_b1.verticesCCW().push_back(Eigen::Vector2d(-0.25, 1.1));
    //         obstacle_b1.verticesCCW().push_back(Eigen::Vector2d(-0.25, 2));
    //         obstacle_b1.verticesCCW().push_back(Eigen::Vector2d(0.25, 2));
    //         obstacle_b1.verticesCCW().push_back(Eigen::Vector2d(0.25, 1.1));
    //         amp::Obstacle2D obstacle_b2;
    //         obstacle_b2.verticesCCW().push_back(Eigen::Vector2d(-2, -2));
    //         obstacle_b2.verticesCCW().push_back(Eigen::Vector2d(-2, -1.8));
    //         obstacle_b2.verticesCCW().push_back(Eigen::Vector2d(2, -1.8));
    //         obstacle_b2.verticesCCW().push_back(Eigen::Vector2d(2, -2));
    //         problem_b.obstacles.push_back(obstacle_b1);
    //         problem_b.obstacles.push_back(obstacle_b2);
    //         // planner env b:
    //         std::unique_ptr<amp::GridCSpace2D> grid_b = configurationSpaceConstructor.construct(manipulator, problem_b);
    //         amp::Path2D path_b = algo2.planInCSpace(Eigen::Vector2d(q_init[0], q_init[1]), Eigen::Vector2d(q_goal[0], q_goal[1]), *grid_b);
    //         amp::Visualizer::makeFigure(*grid_b, path_b);
    //         amp::Visualizer::makeFigure(problem_b, manipulator, path_b);

    //     // env c:
    //         amp::Problem2D problem_c;
    //         amp::Obstacle2D obstacle_c1;
    //         obstacle_c1.verticesCCW().push_back(Eigen::Vector2d(-0.25, 1.1));
    //         obstacle_c1.verticesCCW().push_back(Eigen::Vector2d(-0.25, 2));
    //         obstacle_c1.verticesCCW().push_back(Eigen::Vector2d(0.25, 2));
    //         obstacle_c1.verticesCCW().push_back(Eigen::Vector2d(0.25, 1.1));
    //         amp::Obstacle2D obstacle_c2;
    //         obstacle_c2.verticesCCW().push_back(Eigen::Vector2d(-2, -0.5));
    //         obstacle_c2.verticesCCW().push_back(Eigen::Vector2d(-2, -0.3));
    //         obstacle_c2.verticesCCW().push_back(Eigen::Vector2d(2, -0.3));
    //         obstacle_c2.verticesCCW().push_back(Eigen::Vector2d(2, -0.5));
    //         problem_c.obstacles.push_back(obstacle_c1);
    //         problem_c.obstacles.push_back(obstacle_c2);
    //         // planner env c:
    //         std::unique_ptr<amp::GridCSpace2D> grid_c = configurationSpaceConstructor.construct(manipulator, problem_c);
    //         amp::Path2D path_c = algo2.planInCSpace(Eigen::Vector2d(q_init[0], q_init[1]), Eigen::Vector2d(q_goal[0], q_goal[1]), *grid_c);
    //         amp::Visualizer::makeFigure(*grid_c, path_c);
    //         amp::Visualizer::makeFigure(problem_c, manipulator, path_c);

    //     bool prob_2_a = HW6::checkLinkManipulatorPlan(path_a, manipulator, problem_a);

    // prob 3
    
    // amp::Visualizer::showFigures();
    // std::cout<<"Problem 2 a: "<<prob_2_a<<std::endl;
    //  amp::HW6::grade<MyPointWaveFrontAlgorithm, MyManipulatorWaveFrontAlgorithm, MyAStarAlgo>("nonhuman.biologic@myspace.edu", argc, argv, std::make_tuple(), std::make_tuple("hey therre"), std::make_tuple());
    return 0;
}