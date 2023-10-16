#pragma once

#include "AMPCore.h"
#include "hw/HW6.h"

namespace amp{
    class MyPointWaveFrontAlgorithm : public PointWaveFrontAlgorithm{
        public:
            // constructor and destructor
            MyPointWaveFrontAlgorithm();
            ~MyPointWaveFrontAlgorithm();

            // methods
            std::unique_ptr<amp::GridCSpace2D> constructDiscretizedWorkspace(const amp::Environment2D& environment) override;
            Path2D plan(const Problem2D& prob) override;
    };
};