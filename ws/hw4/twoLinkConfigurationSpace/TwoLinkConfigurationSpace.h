#pragma once

#include "AMPCore.h"
#include "../twoLinkManipulator/TwoLinkManipulator.h"

namespace amp {
    class TwoLinkConfigurationSpace : public amp::GridCSpace2D {
        public:
            // Constructor
            TwoLinkConfigurationSpace(std::size_t x0_cells, std::size_t x1_cells, double x0_min, double x0_max, double x1_min, double x1_max);
            ~TwoLinkConfigurationSpace();

            // methods
            bool inCollision(double x0, double x1) const override;
            void setGridCSpace(TwoLinkManipulator manipulator, Environment2D environment);

            // fields
            double resolution_x0;
            double resolution_x1;
    };
};
