# pragma once
#include "AMPCore.h"

namespace amp{
    class NDConfigurationSpace : public amp::ConfigurationSpace {

        public:

        NDConfigurationSpace(const Eigen::VectorXd& lower_bounds, const Eigen::VectorXd& upper_bounds);
        void construct(const amp::MultiAgentProblem2D prob);
        bool inCollision(const Eigen::VectorXd& cspace_state) const override;
        bool inCollision(const Eigen::VectorXd& cspace_state, const amp::MultiAgentProblem2D& prob) const;

    };
}