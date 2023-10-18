#include "MyConfigurationSpace.h"
#include "AMPCore.h"
#include "Helper.h"
#include <cmath> 

namespace amp {

    MyConfigurationSpace::MyConfigurationSpace(std::size_t x0_cells, std::size_t x1_cells, double x0_min, double x0_max, double x1_min, double x1_max) 
        : GridCSpace2D(x0_cells, x1_cells, x0_min, x0_max, x1_min, x1_max) 
    {
        resolution_x0 = (x0_max - x0_min) / this->size().first;
        resolution_x1 = (x1_max - x1_min) / this->size().second;
    }


    MyConfigurationSpace::~MyConfigurationSpace() {}

    bool MyConfigurationSpace::inCollision(double x0, double x1) const {
        std::size_t x0_grid = std::round(x0 / resolution_x0);
        std::size_t x1_grid = std::round(x1 / resolution_x1);
        if(x0_grid >= size().first){
            x0_grid = size().first - 1;
        }
        if(x1_grid >= size().second){
            x1_grid = size().second - 1;
        }
        return (*this)(x0_grid, x1_grid);
    }

    std::pair<std::size_t, std::size_t> MyConfigurationSpace::getCellFromPoint(double x0, double x1) const{
        int x0_grid = std::ceil((x0 - this->x0Bounds().first)/ resolution_x0) - 1;
        int x1_grid = std::ceil((x1 - this->x1Bounds().first) / resolution_x1) - 1;
        return std::make_pair(x0_grid, x1_grid);
    }

    Eigen::Vector2d MyConfigurationSpace::getPointFromCell(std::size_t x0_grid, std::size_t x1_grid) const {
        double x0 = (x0_grid + 1) * resolution_x0 + this->x0Bounds().first;
        double x1 = (x1_grid + 1) * resolution_x1 + this->x1Bounds().first;
        return Eigen::Vector2d(x0, x1);
    }

}
