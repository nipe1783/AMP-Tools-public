// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW4.h"

// Include the header of the shared class
#include "HelpfulClass.h"

using namespace amp;

int main(int argc, char** argv) {
    /* Include this line to have different randomized environments every time you run your code (NOTE: this has no affect on grade()) */
    amp::RNG::seed(amp::RNG::randiUnbounded());

    // 1.
    std::vector<amp::Polygon> polygons;

    amp::Polygon obstacle;
    obstacle.verticesCCW().push_back(Eigen::Vector2d(0.0, 0.0));
    obstacle.verticesCCW().push_back(Eigen::Vector2d(1.0, 2.0));
    obstacle.verticesCCW().push_back(Eigen::Vector2d(0.0, 2.0));
    
    amp::Polygon robot;
    robot.verticesCCW().push_back(Eigen::Vector2d(0.0, 0.0));
    robot.verticesCCW().push_back(Eigen::Vector2d(1.0, 2.0));
    robot.verticesCCW().push_back(Eigen::Vector2d(0.0, 2.0));
    

    amp::Polygon cSpaceObstacle;
    cSpaceObstacle = Helper().minkowskiDiff(obstacle, robot);
    polygons.push_back(cSpaceObstacle);

    amp::Visualizer::makeFigure(polygons, 0);
    amp::Visualizer::showFigures();


    // Grade method
    //amp::HW4::grade<MyLinkManipulator>(constructor, "nonhuman.biologic@myspace.edu", argc, argv);
    return 0;
}