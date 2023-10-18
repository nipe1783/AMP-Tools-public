#pragma once

#include "AMPCore.h"

namespace amp {
    class MyConfigurationSpace : public amp::GridCSpace2D {
        public:
            // Constructor
            MyConfigurationSpace(std::size_t x0_cells, std::size_t x1_cells, double x0_min, double x0_max, double x1_min, double x1_max);
            ~MyConfigurationSpace();

            // methods
            bool inCollision(double x0, double x1) const override;
            std::pair<std::size_t, std::size_t> getCellFromPoint(double x0, double x1) const override;
            Eigen::Vector2d getPointFromCell(std::size_t x0, std::size_t x1) const;
            
            // fields
            double resolution_x0;
            double resolution_x1;
    };
};
