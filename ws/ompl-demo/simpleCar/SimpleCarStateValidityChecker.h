#include <boost/geometry.hpp>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <set>
#include "AMPCore.h"
#include "../wsOmpl/ObstacleOmpl.h"
#include "SimpleCar.h"

namespace ob = ompl::base;

typedef boost::geometry::model::d2::point_xy<double> point;
typedef boost::geometry::model::polygon<point> polygon;

class SimpleCarStateValidityChecker : public ob::StateValidityChecker
{
    public:
        SimpleCarStateValidityChecker(const ob::SpaceInformationPtr &si, const amp::Problem2D *prob, const SimpleCar *car, const std::vector<double> &safetyMargin);

        /**
         * @brief Determines if a state is valid
         *
         * @param state current state of the agent.
         * @return bool, true if state is valid, false otherwise.
         */
        bool isValid(const ob::State *state) const override;

    private:
        const ob::SpaceInformation *si_;
        const amp::Problem2D *prob_;
        const SimpleCar *car_;
        std::vector<double> safetyMargin_;
    
};