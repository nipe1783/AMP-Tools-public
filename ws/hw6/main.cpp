// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW2.h"
#include "hw/HW5.h"
#include "hw/HW6.h"

// Include files
#include "MyPointWaveFrontAlgorithm.h"

using namespace amp;

int main(int argc, char** argv) {
    
    // problem 1: workspace 1
    Problem2D problem = HW5::getWorkspace1();
    MyPointWaveFrontAlgorithm algo;
    Path2D path = algo.plan(problem);

    // Visualizer::makeFigure(problem, path);
    // amp::Visualizer::showFigures();
    // HW6::grade(algo, "nipe1783@colorado.edu", argc, argv);

    return 0;
}