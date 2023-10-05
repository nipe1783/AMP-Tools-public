// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW4.h"

// Include the header of the shared class
#include "HelpfulClass.h"

#include "threeLinkManipulator/ThreeLinkManipulator.h"
#include "twoLinkManipulator/TwoLinkManipulator.h"
#include "twoLinkConfigurationSpace/TwoLinkConfigurationSpace.h"
#include "gridCSpace2D2LinkConstructor/GridCSpace2D2LinkConstructor.h"

using namespace amp;
#include <cmath> 

int main(int argc, char** argv) {
    /* Include this line to have different randomized environments every time you run your code (NOTE: this has no affect on grade()) */
    amp::RNG::seed(amp::RNG::randiUnbounded());

    // // 1.
    //     // a:
    //         std::vector<amp::Polygon> polygons2D;

    //         amp::Polygon obstacle;
    //         obstacle.verticesCCW().push_back(Eigen::Vector2d(0.0, 0.0));
    //         obstacle.verticesCCW().push_back(Eigen::Vector2d(1.0, 2.0));
    //         obstacle.verticesCCW().push_back(Eigen::Vector2d(0.0, 2.0));
            
    //         amp::Polygon robot;
    //         robot.verticesCCW().push_back(Eigen::Vector2d(0.0, 0.0));
    //         robot.verticesCCW().push_back(Eigen::Vector2d(1.0, 2.0));
    //         robot.verticesCCW().push_back(Eigen::Vector2d(0.0, 2.0));
            
    //         amp::Polygon cSpaceObstacle;
    //         cSpaceObstacle = Helper().minkowskiDiff(obstacle, robot);
    //         polygons2D.push_back(cSpaceObstacle);

    //         amp::Visualizer::makeFigure(polygons2D, 1);
    //         amp::Visualizer::showFigures();

    //     // b:
    //         std::vector<amp::Polygon> polygons3D;
    //         amp::Polygon rotatedObstacle;
    //         std::vector<double> thetas;
    //         const int count = 12;
    //         double step = 2 * M_PI / count;

    //         for (int i = 0; i < count; ++i) {
    //             thetas.push_back(i * step);
    //         }

    //         for(int i = 0; i < thetas.size(); i++){
    //             rotatedObstacle = Helper().rotatePolygon(robot, robot.verticesCCW()[0], thetas[i]);
    //             cSpaceObstacle = Helper().minkowskiDiff(obstacle, rotatedObstacle);
    //             polygons3D.push_back(cSpaceObstacle);
    //         }
    //         amp::Visualizer::makeFigure(polygons3D, thetas);
    //         amp::Visualizer::showFigures();

    // // 2.

    //     // a:
    //         std::vector<double> link_lengths_a = {0.5, 1.0, 0.5};
    //         std::vector<double> state_a = {M_PI / 6, M_PI / 3, 7 * M_PI / 6};
    //         ThreeLinkManipulator manipulator_a(link_lengths_a);
    //         Eigen::Vector2d jointLocation = manipulator_a.getJointLocation(state_a, 3);
    //         std::cout << " x: " << jointLocation[0] << " y: " << jointLocation[1] << std::endl;
    //         amp::Visualizer::makeFigure(manipulator_a, state_a);
    //         amp::Visualizer::showFigures();

    //     // b:
    //         std::vector<double> link_lengths_b = {1, .5, 1};
    //         ThreeLinkManipulator manipulator_b(link_lengths_b);
    //         std::vector<double> state_b = manipulator_b.getConfigurationFromIK(Eigen::Vector2d(2.0, 0));
    //         std::cout<< "theta1: " << state_b[0] << " theta2: " << state_b[1] << " theta3: " << state_b[2] << std::endl;
    //         amp::Visualizer::makeFigure(manipulator_b, state_b);
    //         amp::Visualizer::showFigures();

    // 3.
        std::vector<double> link_lengths_3 = {1.0, 1.0};
        std::vector<double> state_3 = {0.4 * 2 *M_PI, 0.8 * 2 *M_PI};
        amp::TwoLinkManipulator manipulator_3(link_lengths_3);
        amp::GridCSpace2D2LinkConstructor configurationSpaceConstructor;
        // a:
            // amp::Environment2D environment_a;
            // amp::Obstacle2D obstacle_a;
            // obstacle_a.verticesCCW().push_back(Eigen::Vector2d(0.25, 0.25));
            // obstacle_a.verticesCCW().push_back(Eigen::Vector2d(0, 0.75));
            // obstacle_a.verticesCCW().push_back(Eigen::Vector2d(-0.25, 0.25)); 
            // environment_a.obstacles.push_back(obstacle_a);
            // std::unique_ptr<amp::GridCSpace2D> cSpace_a = configurationSpaceConstructor.construct(manipulator_3, environment_a);
            // amp::GridCSpace2D& cSpace_a_ref = *cSpace_a;
            // amp::Visualizer::makeFigure(environment_a, manipulator_3, state_3);
            // amp::Visualizer::makeFigure(cSpace_a_ref);
            // amp::Visualizer::showFigures();

    //     // b:
    //         amp::Environment2D environment_b;
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
    //         environment_b.obstacles.push_back(obstacle_b1);
    //         environment_b.obstacles.push_back(obstacle_b2);
    //         std::unique_ptr<amp::GridCSpace2D> cSpace_b = configurationSpaceConstructor.construct(manipulator_3, environment_b);
    //         amp::GridCSpace2D& cSpace_b_ref = *cSpace_b;
    //         amp::Visualizer::makeFigure(environment_b, manipulator_3, state_3);
    //         amp::Visualizer::makeFigure(cSpace_b_ref);
    //         amp::Visualizer::showFigures();
        
    //     // c:
    //         amp::Environment2D environment_c;
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
    //         environment_c.obstacles.push_back(obstacle_c1);
    //         environment_c.obstacles.push_back(obstacle_c2);
    //         std::unique_ptr<amp::GridCSpace2D> cSpace_c = configurationSpaceConstructor.construct(manipulator_3, environment_c);
    //         amp::GridCSpace2D& cSpace_c_ref = *cSpace_c;
    //         amp::Visualizer::makeFigure(environment_c, manipulator_3, state_3);
    //         amp::Visualizer::makeFigure(cSpace_c_ref);
    //         amp::Visualizer::showFigures();

    // Grade method
    amp::HW4::grade<TwoLinkManipulator>(configurationSpaceConstructor, "nipe1783@colorado.edu", argc, argv);
    return 0;
}