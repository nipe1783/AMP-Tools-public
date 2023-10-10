#pragma once
#include "AMPCore.h"
#include "hw/HW5.h"

namespace amp{
    class MyGDAlgorithm : public GDAlgorithm{
        public:
            // constructor and destructor
            MyGDAlgorithm();
            ~MyGDAlgorithm();

            Path2D plan(const Problem2D& prob) override;
    };
};