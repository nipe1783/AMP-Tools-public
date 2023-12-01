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
            if (!si_->satisfiesBounds(state))
                return false;
            // Get xy state and theta state (in rad [0, 2*pi]) from state object
            auto compState = state->as<ob::CompoundStateSpace::StateType>();
            auto xyState = compState->as<ob::RealVectorStateSpace::StateType>(0);
            const double cx = xyState->values[0];
            const double cy = xyState->values[1];
            const double theta = compState->as<ob::SO2StateSpace::StateType>(1)->value;

            // Get important params from car object
            const double carWidth = a_->getShape()[0];
            const double carHeight = a_->getShape()[1];

            // turn (x,y, theta), width, length to a polygon object
            // see https://stackoverflow.com/questions/41898990/find-corners-of-a-rotated-rectangle-given-its-center-point-and-rotation
            // TOP RIGHT VERTEX:
            const double TR_x = cx + ((carWidth / 2) * cos(theta)) - ((carHeight / 2) * sin(theta));
            const double TR_y = cy + ((carWidth / 2) * sin(theta)) + ((carHeight / 2) * cos(theta));
            std::string top_right = std::to_string(TR_x) + " " + std::to_string(TR_y);
            // TOP LEFT VERTEX:
            const double TL_x = cx - ((carWidth / 2) * cos(theta)) - ((carHeight / 2) * sin(theta));
            const double TL_y = cy - ((carWidth / 2) * sin(theta)) + ((carHeight / 2) * cos(theta));
            std::string top_left = std::to_string(TL_x) + " " + std::to_string(TL_y);
            // BOTTOM LEFT VERTEX:
            const double BL_x = cx - ((carWidth / 2) * cos(theta)) + ((carHeight / 2) * sin(theta));
            const double BL_y = cy - ((carWidth / 2) * sin(theta)) - ((carHeight / 2) * cos(theta));
            std::string bottom_left = std::to_string(BL_x) + " " + std::to_string(BL_y);
            // BOTTOM RIGHT VERTEX:
            const double BR_x = cx + ((carWidth / 2) * cos(theta)) + ((carHeight / 2) * sin(theta));
            const double BR_y = cy + ((carWidth / 2) * sin(theta)) - ((carHeight / 2) * cos(theta));
            std::string bottom_right = std::to_string(BR_x) + " " + std::to_string(BR_y);

            // convert to string for easy initializataion
            std::string points = "POLYGON((" + bottom_left + "," + bottom_right + "," + top_right + "," + top_left + "," + bottom_left + "))";
            polygon agent;
            boost::geometry::read_wkt(points,agent);
            
            // check agent is disjoint from all obstacles
            for(amp::Obstacle2D o : prob_->obstacles){
                polygon poly;
                ObstacleOmpl obs(o);
                poly = obs.poly_;
                if (! boost::geometry::disjoint(agent, poly))
                    return false;
            }
            return true;
        }

    private:
        const ob::SpaceInformation *si_;
        const amp::Problem2D *prob_;
        const AgentOmpl *a_;
    
};