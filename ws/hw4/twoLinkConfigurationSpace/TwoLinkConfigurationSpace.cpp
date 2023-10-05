#include "TwoLinkConfigurationSpace.h"
#include "../twoLinkManipulator/TwoLinkManipulator.h"
#include "AMPCore.h"
#include "HelpfulClass.h"
#include <cmath> 

namespace amp {

    TwoLinkConfigurationSpace::TwoLinkConfigurationSpace(std::size_t x0_cells, std::size_t x1_cells, double x0_min, double x0_max, double x1_min, double x1_max) 
        : GridCSpace2D(x0_cells, x1_cells, x0_min, x0_max, x1_min, x1_max) 
    {
        resolution_x0 = (x0_max - x0_min) / this->size().first;
        resolution_x1 = (x1_max - x1_min) / this->size().second;
    }


    TwoLinkConfigurationSpace::~TwoLinkConfigurationSpace() {}

    void TwoLinkConfigurationSpace::setGridCSpace(TwoLinkManipulator manipulator, Environment2D environment) {
        Eigen::Vector2d endPoint;
        Eigen::Vector2d startPoint;
        double x0;
        double x1;
        for (double i = 0; i < m_x0_bounds.second; i += resolution_x0) {
            for (double j = 0; j < m_x1_bounds.second; j += resolution_x1) {
                for (int k = 1; k <= manipulator.nLinks(); k++) {
                    if(k > 1){
                        startPoint = endPoint;
                    }
                    else{
                        startPoint = manipulator.getBaseLocation();
                    }
                    endPoint = manipulator.getJointLocation({i, j}, k);
                    if (Helper().intersects(environment, startPoint, endPoint)) {
                        x0 = std::round(i / resolution_x0);
                        x1 = std::round(j / resolution_x1);
                        if(x0 >= this->size().first){
                            x0 = this->size().first - 1;
                        }
                        if(x1 >= this->size().second){
                            x1 = this->size().second - 1;
                        }
                        (*this)(x0, x1) = true;
                    }
                }
            }
        }
    }

    bool TwoLinkConfigurationSpace::inCollision(double x0, double x1) const {
        std::size_t x0_grid = std::round(x0 / resolution_x0);
        std::size_t x1_grid = std::round(x1 / resolution_x1);
        return (*this)(x0_grid, x1_grid);
    }
}
