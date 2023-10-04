#include "TwoLinkConfigurationSpace.h"
#include "../twoLinkManipulator/TwoLinkManipulator.h"
#include "AMPCore.h"
#include "HelpfulClass.h"
#include <cmath> 

namespace amp{

    TwoLinkConfigurationSpace::TwoLinkConfigurationSpace(double x0_min, double x0_max, double x1_min, double x1_max) : ConfigurationSpace2D(x0_min, x0_max, x1_min, x1_max), gridCSpace(10, 10, true) {
        double resolution_x0 = gridCSpace.size().first / (2 * M_PI);
        double resolution_x1 = gridCSpace.size().second / (2 * M_PI);
    }

    TwoLinkConfigurationSpace::~TwoLinkConfigurationSpace() {}

    void TwoLinkConfigurationSpace::setGridCSpace(TwoLinkManipulator manipulator, Environment2D environment){
        Eigen::Vector2d endPoint;
        Eigen::Vector2d startPoint = manipulator.getBaseLocation();
        for(double i = 0; i < m_x0_bounds.second; i += resolution_x0){
            for(double j = 0; j < m_x1_bounds.second; j += resolution_x1){
                for(int k = 0; k < manipulator.nLinks(); k++){
                    endPoint = manipulator.getJointLocation({i, j}, k);
                    if(Helper().intersects(environment, startPoint, endPoint)){
                        gridCSpace(i, j) = false;
                    }
                }
            }
        }
    }

    bool TwoLinkConfigurationSpace::inCollision(double x0, double x1) const {
        double x0_grid = std::round(x0 / resolution_x0) * resolution_x0;
        double x1_grid = std::round(x1 / resolution_x1) * resolution_x1;
        return !gridCSpace(x0_grid, x1_grid);
    }
}
