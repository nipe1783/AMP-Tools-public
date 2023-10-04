#include "AMPCore.h"
#include <cmath>

namespace amp {
    class ThreeLinkManipulator : public amp::LinkManipulator2D{
    public:
        // Constructor
        ThreeLinkManipulator(const std::vector<double>& link_lengths);
        ~ThreeLinkManipulator();

        // methods
        Eigen::Vector2d getJointLocation(const ManipulatorState& state, uint32_t joint_index) const override;
        ManipulatorState getConfigurationFromIK(const Eigen::Vector2d& end_effector_location) const override;


    };
};