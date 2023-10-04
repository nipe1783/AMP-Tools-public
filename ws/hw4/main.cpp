// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW4.h"

// Include the header of the shared class
#include "HelpfulClass.h"

#include "threeLinkManipulator/ThreeLinkManipulator.h"
#include "twoLinkManipulator/TwoLinkManipulator.h"
#include "twoLinkManipulatorProblem/TwoLinkManipulatorProblem.h"
#include "twoLinkConfigurationSpace/TwoLinkConfigurationSpace.h"

using namespace amp;
#include <cmath> 

int main(int argc, char** argv) {
    /* Include this line to have different randomized environments every time you run your code (NOTE: this has no affect on grade()) */
    amp::RNG::seed(amp::RNG::randiUnbounded());

    // 1.
        // a:
            // std::vector<amp::Polygon> polygons2D;

            // amp::Polygon obstacle;
            // obstacle.verticesCCW().push_back(Eigen::Vector2d(0.0, 0.0));
            // obstacle.verticesCCW().push_back(Eigen::Vector2d(1.0, 2.0));
            // obstacle.verticesCCW().push_back(Eigen::Vector2d(0.0, 2.0));
            
            // amp::Polygon robot;
            // robot.verticesCCW().push_back(Eigen::Vector2d(1.0, 1.0));
            // robot.verticesCCW().push_back(Eigen::Vector2d(2.0, 3.0));
            // robot.verticesCCW().push_back(Eigen::Vector2d(1.0, 3.0));
            
            // amp::Polygon cSpaceObstacle;
            // cSpaceObstacle = Helper().minkowskiDiff(obstacle, robot);
            // polygons2D.push_back(cSpaceObstacle);

            // amp::Visualizer::makeFigure(polygons2D, 1);
            // amp::Visualizer::showFigures();

        // b:
            // std::vector<amp::Polygon> polygons3D;
            // amp::Polygon rotatedObstacle;
            // std::vector<double> thetas;
            // const int count = 12;
            // double step = 2 * M_PI / count;

            // for (int i = 0; i < count; ++i) {
            //     thetas.push_back(i * step);
            // }

            // for(int i = 0; i < thetas.size(); i++){
            //     rotatedObstacle = Helper().rotatePolygon(robot, robot.verticesCCW()[0], thetas[i]);
            //     cSpaceObstacle = Helper().minkowskiDiff(obstacle, rotatedObstacle);
            //     polygons3D.push_back(cSpaceObstacle);
            // }
            // amp::Visualizer::makeFigure(polygons3D, thetas);
            // amp::Visualizer::showFigures();

    // 2.

        // a:
            // std::vector<double> link_lengths = {0.5, 1.0, 0.5};
            // std::vector<double> state = {M_PI / 6, M_PI / 3, 7 * M_PI / 6};
            // ThreeLinkManipulator manipulator(link_lengths);
            // Eigen::Vector2d jointLocation = manipulator.getJointLocation(state, 3);
            // std::cout << " x: " << jointLocation[0] << " y: " << jointLocation[1] << std::endl;
            // amp::Visualizer::makeFigure(manipulator, state);
            // amp::Visualizer::showFigures();

        // b:
            // std::vector<double> link_lengths = {1, .5, 1};
            // ThreeLinkManipulator manipulator(link_lengths);
            // std::vector<double> state = manipulator.getConfigurationFromIK(Eigen::Vector2d(2.0, 0));
            // std::cout<< "theta1: " << state[0] << " theta2: " << state[1] << " theta3: " << state[2] << std::endl;
            // amp::Visualizer::makeFigure(manipulator, state);
            // amp::Visualizer::showFigures();

    // 3.
        // a:
            std::vector<double> link_lengths = {1.0, 1.0};
            std::vector<double> state = {0, 0};
            amp::TwoLinkManipulator manipulator(link_lengths);
            amp::Environment2D environment;
            amp::Obstacle2D obstacle;
            obstacle.verticesCCW().push_back(Eigen::Vector2d(0.25, 0.25));
            obstacle.verticesCCW().push_back(Eigen::Vector2d(0, 0.75));
            obstacle.verticesCCW().push_back(Eigen::Vector2d(-0.25, 0.25)); 
            environment.obstacles.push_back(obstacle);
            amp::TwoLinkConfigurationSpace configurationSpace(0, 2*M_PI, 0, 2*M_PI);
            configurationSpace.setGridCSpace(manipulator, environment);
            amp::Visualizer::makeFigure(configurationSpace.gridCSpace);
            amp::Visualizer::showFigures();

    // Grade method
    //amp::HW4::grade<MyLinkManipulator>(constructor, "nonhuman.biologic@myspace.edu", argc, argv);
    return 0;
}