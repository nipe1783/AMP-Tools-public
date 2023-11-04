#include "AMPCore.h"
#include "hw/HW2.h"
#include "hw/HW5.h"
#include "hw/HW6.h"
#include "hw/HW7.h"
#include "hw/HW8.h"
#include "Helper.h"
#include <iostream>
#include "MyCentralizedMultiAgentRRT.h"

using namespace amp;

int main(int argc, char** argv) {

    MultiAgentProblem2D prob = HW8::getWorkspace1();
    MyCentralizedMultiAgentRRT rrt;
    MultiAgentPath2D path = rrt.plan(prob);
    // Visualizer::makeFigure(prob, path);
    // Visualizer::showFigures();

    return 0;
}







