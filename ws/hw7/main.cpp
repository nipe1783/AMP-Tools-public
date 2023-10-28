// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW2.h"
#include "hw/HW5.h"
#include "hw/HW6.h"
#include "hw/HW7.h"

// Include files
#include "MyPRM2D.h"

using namespace amp;

int main(int argc, char** argv) {
    
    Problem2D problem = HW5::getWorkspace1();
    MyPRM2D prm;
    prm.plan(problem);
    
    // amp::HW7::grade<MyPointWaveFrontAlgorithm, MyManipulatorWaveFrontAlgorithm, MyAStarAlgorithm>("nipe1783@colorado.edu", argc, argv, std::make_tuple(), std::make_tuple("hey therre"), std::make_tuple());
    return 0;
}