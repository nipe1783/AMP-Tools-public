#include <ompl/base/goals/GoalRegion.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <utility>

namespace ob = ompl::base;

class GoalRegion2ndOrderCar: public ob::GoalRegion
{
public:
    GoalRegion2ndOrderCar(const ob::SpaceInformationPtr &si, double gx, double gy): 
        ob::GoalRegion(si), gx_(gx), gy_(gy)
    {
        threshold_ = 0.5;
    }
    
    double distanceGoal(const ob::State *st) const override
    {
        const double* robot_pos = st->as<ob::CompoundStateSpace::StateType>()->as<ob::RealVectorStateSpace::StateType>(0)->values;
        return sqrt(pow(robot_pos[0] - gx_, 2) + pow(robot_pos[1] - gy_, 2));
    }
private:
    const double gx_;
    const double gy_;
};
