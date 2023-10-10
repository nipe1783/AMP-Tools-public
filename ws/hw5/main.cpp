// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW5.h"
#include "hw/HW2.h"

// Include the header of the shared class
#include "HelpfulClass.h"

// Include the header of your algorithm
#include "myGDAlgorithm/MyGDAlgorithm.h"

using namespace amp;

int main(int argc, char** argv) {
    
    // problem 1: workspace 1
        Problem2D problem = HW2::getWorkspace1();

        MyGDAlgorithm algo;
        Path2D path = algo.plan(problem);

        // Visualize the path and environment
        // Visualizer::makeFigure(problem, path);
        // Visualizer::showFigures();

    return 0;
}