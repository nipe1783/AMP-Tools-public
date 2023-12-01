#pragma once
#include "AMPCore.h"
#include "hw/HW7.h"
#include "hw/HW6.h"

namespace amp {
    class GradePlanner : public amp::GoalBiasRRT2D {
        public:
            /**
             * @brief Plan a path from the start to the goal.
             * 
             * @param prob Problem to plan
             * @return Path2D Path from start to goal
             **/
            Path2D plan(const Problem2D& prob) override;
    };
}
