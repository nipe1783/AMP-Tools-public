#include "AMPCore.h"
#include "hw/HW2.h"

using namespace amp;

int main(int argc, char** argv) {

    Problem2D problem = HW2::getWorkspace1();
    amp::Path2D path;

    path.waypoints.push_back(problem.q_init);
    path.waypoints.push_back(problem.q_goal);

    Visualizer::makeFigure(problem, path);
    Visualizer::showFigures();

    return 0;
}