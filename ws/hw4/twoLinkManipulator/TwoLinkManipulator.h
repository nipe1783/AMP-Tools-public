#pragma once

#include "AMPCore.h"
#include <cmath>

namespace amp {
    class TwoLinkManipulator : public amp::LinkManipulator2D{
        public:
            // Constructor
            TwoLinkManipulator(const std::vector<double>& link_lengths);
            TwoLinkManipulator();
            ~TwoLinkManipulator();

            // methods
            Eigen::Vector2d getJointLocation(const ManipulatorState& state, uint32_t joint_index) const override;
            ManipulatorState getConfigurationFromIK(const Eigen::Vector2d& end_effector_location) const override;
    };
};