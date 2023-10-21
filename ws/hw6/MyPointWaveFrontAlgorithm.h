#pragma once

#include "AMPCore.h"
#include "hw/HW6.h"
#include "Tree.h"

namespace amp{
    class MyPointWaveFrontAlgorithm : public PointWaveFrontAlgorithm{
        public:
            // constructor and destructor
            MyPointWaveFrontAlgorithm();
            ~MyPointWaveFrontAlgorithm();

            // methods
            std::unique_ptr<amp::GridCSpace2D> constructDiscretizedWorkspace(const amp::Environment2D& environment) override;
            amp::Path2D planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace) override;
            std::vector<std::pair<std::size_t, std::size_t>> constructTree(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace);

    };
};