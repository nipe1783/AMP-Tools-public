#include "ThreeLinkManipulator.h"
#include "AMPCore.h"
#include <cmath> 

namespace amp{

    ThreeLinkManipulator::ThreeLinkManipulator(const std::vector<double>& linkLengths) : LinkManipulator2D(linkLengths) {}
    ThreeLinkManipulator::~ThreeLinkManipulator() {}

    Eigen::Vector2d ThreeLinkManipulator::getJointLocation(const ManipulatorState& state, uint32_t joint_index) const {
        Eigen::Vector2d jointLocation = Eigen::Vector2d(0, 0);
        double theta = 0;
        for(int i = 0; i < joint_index; i++){
            theta += state[i];
            jointLocation[0] += getLinkLengths()[i] * cos(theta);
            jointLocation[1] += getLinkLengths()[i] * sin(theta);
        }
        return jointLocation;
    }

    ManipulatorState ThreeLinkManipulator::getConfigurationFromIK(const Eigen::Vector2d& end_effector_location) const {
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
            std::cout<<"c_2: "<<c_2<<std::endl;
            s_2 = sqrt(1 - pow(c_2, 2));

            // check if joint 3 location is reachable
            if(!std::isnan(s_2) && !std::isnan(c_2)){
                theta1 = atan2(joint3location[1], joint3location[0]) - atan2(getLinkLengths()[1] * s_2, getLinkLengths()[0] + getLinkLengths()[1] * c_2);
                theta2 = atan2(s_2, c_2);
                theta3 = psi - theta1 - theta2;
                return ManipulatorState({theta1, theta2, theta3});
            }

        }
        std::cout << "No solution found" << std::endl;
        return ManipulatorState({0, 0, 0});
        
    }

}