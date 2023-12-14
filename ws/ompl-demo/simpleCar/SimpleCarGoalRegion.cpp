#include "SimpleCarGoalRegion.h"
#include <ompl/base/goals/GoalRegion.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <utility>
#include <cmath> 

namespace ob = ompl::base;


GoalRegion2ndOrderCar::GoalRegion2ndOrderCar(const ob::SpaceInformationPtr &si, double gx, double gy, double gtheta) : ob::GoalRegion(si), gx_(gx), gy_(gy), gtheta_(gtheta)
{
    threshold_ = 0.3;
}
    
double GoalRegion2ndOrderCar::distanceGoal(const ob::State *st) const
{
    const double* robot_pos = st->as<ob::CompoundStateSpace::StateType>()->as<ob::RealVectorStateSpace::StateType>(0)->values;
    double dist = sqrt(pow(robot_pos[0] - gx_, 2) + pow(robot_pos[1] - gy_, 2)) + fabs(robot_pos[4] - gtheta_);
    return dist;
}
