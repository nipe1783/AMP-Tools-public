#include "MyLinkManipulator.h"
#include "AMPCore.h"
#include <cmath> 

namespace amp{

    MyLinkManipulator::MyLinkManipulator(const std::vector<double>& linkLengths) : LinkManipulator2D(linkLengths) {}

    MyLinkManipulator::MyLinkManipulator() : LinkManipulator2D() {}
    MyLinkManipulator::~MyLinkManipulator() {}

    Eigen::Vector2d MyLinkManipulator::getJointLocation(const ManipulatorState& state, uint32_t joint_index) const {
        Eigen::Vector2d jointLocation = Eigen::Vector2d(0, 0);
        double theta = 0;
        for(int i = 0; i < joint_index; i++){
            theta += state[i];
            jointLocation[0] += getLinkLengths()[i] * cos(theta);
            jointLocation[1] += getLinkLengths()[i] * sin(theta);
        }
        return jointLocation;
    }

    ManipulatorState MyLinkManipulator::getConfigurationFromIK2Link(const Eigen::Vector2d& end_effector_location) const {
        double theta2;
        double theta1;
        double c_2;
        double s_2;

        c_2 = (pow(end_effector_location[0], 2) + pow(end_effector_location[1], 2) - pow(getLinkLengths()[0], 2) - pow(getLinkLengths()[1], 2)) / (2 * getLinkLengths()[0] * getLinkLengths()[1]);
        s_2 = sqrt(1 - pow(c_2, 2));

        // check if goal location is reachable
        if(!std::isnan(s_2) && !std::isnan(c_2)){
            theta1 = atan2(end_effector_location[1], end_effector_location[0]) - atan2(getLinkLengths()[1] * s_2, getLinkLengths()[0] + getLinkLengths()[1] * c_2);
            theta2 = atan2(s_2, c_2);
            Eigen::Vector2d vec(theta1, theta2);
            return ManipulatorState(vec);
        }

        std::cout << "No solution found" << std::endl;
        return ManipulatorState({0, 0});
    }

    ManipulatorState MyLinkManipulator::getConfigurationFromIK3Link(const Eigen::Vector2d& end_effector_location) const {
        double theta3;
        double theta2;
        double theta1;
        double c_2;
        double s_2;
        Eigen::Vector2d joint3location;

        // check all possible psi values (psi is angle from arm 3 to x axis.)
        for(double psi = 0; psi <= 2*M_PI; psi += 0.1){
            // compute location of joint 3 to perform 2DOF manipulator IK
            joint3location[0] = end_effector_location[0] - getLinkLengths()[2] * cos(psi);
            joint3location[1] = end_effector_location[1] - getLinkLengths()[2] * sin(psi);

            c_2 = (pow(joint3location[0], 2) + pow(joint3location[1], 2) - pow(getLinkLengths()[0], 2) - pow(getLinkLengths()[1], 2)) / (2 * getLinkLengths()[0] * getLinkLengths()[1]);
            s_2 = sqrt(1 - pow(c_2, 2));

            // check if joint 3 location is reachable
            if(!std::isnan(s_2) && !std::isnan(c_2)){
                theta1 = atan2(joint3location[1], joint3location[0]) - atan2(getLinkLengths()[1] * s_2, getLinkLengths()[0] + getLinkLengths()[1] * c_2);
                theta2 = atan2(s_2, c_2);
                theta3 = psi - theta1 - theta2;
                Eigen::Vector3d vec(theta1, theta2, theta3);
                return ManipulatorState(vec);
            }

        }
        std::cout << "No solution found" << std::endl;
        Eigen::Vector3d vec(double(0), double(0), double(0));
        return ManipulatorState(vec);
    }

    ManipulatorState MyLinkManipulator::getConfigurationFromIK(const Eigen::Vector2d& end_effector_location) const {
        if(getLinkLengths().size() == 3){
            return getConfigurationFromIK3Link(end_effector_location);
        }
        else if (getLinkLengths().size() == 2)
        {
            return getConfigurationFromIK2Link(end_effector_location);
        }
        else{
            std::cout << "Error: Invalid number of links" << std::endl;
            return ManipulatorState({0, 0});
        }
    }


}