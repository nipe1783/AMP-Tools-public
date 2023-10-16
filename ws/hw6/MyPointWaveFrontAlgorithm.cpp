#include "MyPointWaveFrontAlgorithm.h"
#include "AMPCore.h"

namespace amp{

    std::unique_ptr<amp::GridCSpace2D> MyPointWaveFrontAlgorith::constructDiscretizedWorkspace(const amp::Environment2D& environment){
        
        double density_x0 = 100;
        double density_x1 = 100;
        double x0_min = 0;
        double x0_max = 2*M_PI;
        double x1_min = 0;
        double x1_max = 2*M_PI;
        double resolution_x1 = (x1_max - x1_min) / density_x1;
        double resolution_x0 = (x0_max - x0_min) / density_x0;

        std::unique_ptr<amp::GridCSpace2D> cSpace = std::make_unique<amp::GridCSpace2D>(density_x0, density_x1, x0_min, x0_max, x1_min, x1_max);
        return cSpace;
    }
}