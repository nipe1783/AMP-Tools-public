#pragma once

#include "AMPCore.h"
#include "hw/HW6.h"
#include "Tree.h"
#include "MyGridCSpace2DConstructor.h"

namespace amp{
    class MyManipulatorWaveFrontAlgorithm : public amp::ManipulatorWaveFrontAlgorithm {
        public:
            // Default ctor
            MyManipulatorWaveFrontAlgorithm()
                : amp::ManipulatorWaveFrontAlgorithm(std::make_shared<MyGridCSpace2DConstructor>()) {}

            // You can have custom ctor params for all of these classes
            MyManipulatorWaveFrontAlgorithm(const std::string& beep) 
                : amp::ManipulatorWaveFrontAlgorithm(std::make_shared<MyGridCSpace2DConstructor>()) {LOG("construcing... " << beep);}
            
            // You need to implement here
            amp::Path2D planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace) override;
            std::vector<std::pair<std::size_t, std::size_t>> constructTree(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace);
            bool isCellAndNeighborsFree(int x, int y, const amp::GridCSpace2D& grid_cspace, int x0_grid, int x1_grid, int n);
    };
};