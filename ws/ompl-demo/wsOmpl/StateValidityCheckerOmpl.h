#include <boost/geometry.hpp>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <set>
#include "AMPCore.h"
#include "ObstacleOmpl.h"
#include "AgentOmpl.h"

namespace ob = ompl::base;

typedef boost::geometry::model::d2::point_xy<double> point;
typedef boost::geometry::model::polygon<point> polygon;

class isStateValid_2D : public ob::StateValidityChecker
{
    public:
        isStateValid_2D(const ob::SpaceInformationPtr &si, const amp::Problem2D *prob, const AgentOmpl *a) :
            ob::StateValidityChecker(si)
            {
                si_ = si.get();
                prob_ = prob;
                a_ = a;
            }
    
        bool isValid(const ob::State *state) const override
        {
            auto compState = state->as<ob::CompoundStateSpace::StateType>();
            auto xyState = compState->as<ob::RealVectorStateSpace::StateType>(0);
            const double x = xyState->values[0];
            const double y = xyState->values[1];

            if (!si_->satisfiesBounds(state))
                return false;

            // Define the width and height of the square
            double width = .1/* your width value */;
            double height = .1/* your height value */;

            // Calculate corner points of the square
            polygon agent;
            boost::geometry::append(agent.outer(), point(x - width / 2, y - height / 2));
            boost::geometry::append(agent.outer(), point(x + width / 2, y - height / 2));
            boost::geometry::append(agent.outer(), point(x + width / 2, y + height / 2));
            boost::geometry::append(agent.outer(), point(x - width / 2, y + height / 2));
            boost::geometry::append(agent.outer(), point(x - width / 2, y - height / 2));

            // Check agent is disjoint from all obstacles
            for(const amp::Obstacle2D &o : prob_->obstacles){
                polygon poly;
                ObstacleOmpl obs(o);
                poly = obs.poly_;
                if (!boost::geometry::disjoint(agent, poly))
                    return false;
            }

            return true;
        }

    private:
        const ob::SpaceInformation *si_;
        const amp::Problem2D *prob_;
        const AgentOmpl *a_;
    
};