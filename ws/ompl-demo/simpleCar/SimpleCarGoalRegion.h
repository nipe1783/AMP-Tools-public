#include <ompl/base/goals/GoalRegion.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <utility>

namespace ob = ompl::base;

class GoalRegion2ndOrderCar: public ob::GoalRegion
{
    public:
        // constructor:
        GoalRegion2ndOrderCar(const ob::SpaceInformationPtr &si, double gx, double gy);
        // methods:
        /**
         * @brief determines agent's distance to goal
         *
         * @param st current state of the agent.
         * @return double, distance to goal.
         */
        
        double distanceGoal(const ob::State *st) const override;

    private:
        const double gx_;
        const double gy_;
};
