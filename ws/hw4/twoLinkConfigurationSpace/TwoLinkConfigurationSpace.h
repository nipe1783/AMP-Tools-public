#pragma once

#include "AMPCore.h"
#include "../twoLinkManipulator/TwoLinkManipulator.h"

namespace amp{
    class TwoLinkConfigurationSpace : public amp::ConfigurationSpace2D{
        public:
            // Constructor
            TwoLinkConfigurationSpace(double x0_min, double x0_max, double x1_min, double x1_max);
            ~TwoLinkConfigurationSpace();

            // methods
            bool inCollision(double x0, double x1) const override;
            void setGridCSpace(TwoLinkManipulator manipulator, Environment2D environment);

            // fields:
            DenseArray2D<bool> gridCSpace;
            double resolution_x0;
            double resolution_x1;
    };
};