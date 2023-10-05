#include "GridCSpace2D2LinkConstructor.h"
#include "AMPCore.h"
#include "../twoLinkConfigurationSpace/TwoLinkConfigurationSpace.h"
#include "HelpfulClass.h"
#include <cmath>

namespace amp{
    
    std::unique_ptr<amp::GridCSpace2D> GridCSpace2D2LinkConstructor::construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env){

        Eigen::Vector2d endPoint;
        Eigen::Vector2d startPoint;
        double density_x0 = 100;
        double density_x1 = 100;
        double x0_min = 0;
        double x0_max = 2*M_PI;
        double x1_min = 0;
        double x1_max = 2*M_PI;
        double resolution_x1 = (x1_max - x1_min) / density_x1;
        double resolution_x0 = (x0_max - x0_min) / density_x0;

        std::unique_ptr<amp::GridCSpace2D> cSpace = std::make_unique<amp::TwoLinkConfigurationSpace>(density_x0, density_x1, x0_min, x0_max, x1_min, x1_max);

        double x0;
        double x1;
        for (double i = 0; i < cSpace->x0Bounds().second; i += resolution_x0) {
            for (double j = 0; j < cSpace->x1Bounds().second; j += resolution_x1) {
                for (int k = 1; k <= manipulator.nLinks(); k++) {
                    if(k > 1){
                        startPoint = endPoint;
                    }
                    else{
                        startPoint = manipulator.getBaseLocation();
                    }
                    endPoint = manipulator.getJointLocation({i, j}, k);
                    if (Helper().intersects(env, startPoint, endPoint)) {
                        x0 = std::round(i / resolution_x0);
                        x1 = std::round(j / resolution_x1);
                        if(x0 >= cSpace->size().first){
                            x0 = cSpace->size().first - 1;
                        }
                        if(x1 >= cSpace->size().second){
                            x1 = cSpace->size().second - 1;
                        }
                        cSpace->operator()(x0, x1) = true;
                    }
                }
            }
        }
        return cSpace;
    }

}