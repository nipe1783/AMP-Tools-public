#pragma once

#include "AMPCore.h"

namespace amp {
    class MyLinkManipulator : public amp::LinkManipulator2D{
        public:
            // Constructor
            MyLinkManipulator(const std::vector<double>& link_lengths);
            MyLinkManipulator();
            ~MyLinkManipulator();
            
            // methods
            Eigen::Vector2d getJointLocation(const ManipulatorState& state, uint32_t joint_index) const override;
            ManipulatorState getConfigurationFromIK(const Eigen::Vector2d& end_effector_location) const override;
        
        private:
            // methods
            ManipulatorState getConfigurationFromIK2Link(const Eigen::Vector2d& end_effector_location) const;
            ManipulatorState getConfigurationFromIK3Link(const Eigen::Vector2d& end_effector_location) const;

    };
};