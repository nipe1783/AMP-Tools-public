#pragma once

#include "AMPCore.h"
#include "../twoLinkManipulator/TwoLinkManipulator.h"
#include "../twoLinkConfigurationSpace/TwoLinkConfigurationSpace.h"
#include <cmath>

class TwoLinkManipulatorProblem {
    public :
        // constructor/destructor
        TwoLinkManipulatorProblem();
        ~TwoLinkManipulatorProblem();

        // methods:

        // fields:
        amp::TwoLinkManipulator manipulator;
        // amp::TwoLinkConfigurationSpace configurationSpace;
        amp::Environment2D environment;
};