// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW4.h"

// Include the header of the shared class
#include "HelpfulClass.h"

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



    // Grade method
    //amp::HW4::grade<MyLinkManipulator>(constructor, "nonhuman.biologic@myspace.edu", argc, argv);
    return 0;
}