#pragma once
#include "AMPCore.h"
#include "hw/HW7.h"
#include "hw/HW6.h"
#include "hw/HW8.h"

namespace amp {
    class MyCentralizedMultiAgentRRT : public amp::CentralizedMultiAgentRRT {
        public:
        
        void generateRandomSample(const MultiAgentProblem2D& multiAgentprob, std::vector<Eigen::VectorXd>& nodes, const double p, const double r);
    };
}
