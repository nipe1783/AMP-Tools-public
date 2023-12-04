#include "SimpleCarStateValidityChecker.h"
#include <boost/geometry.hpp>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <set>
#include "AMPCore.h"
#include "../wsOmpl/ObstacleOmpl.h"
#include "SimpleCar.h"


SimpleCarStateValidityChecker::SimpleCarStateValidityChecker(const ob::SpaceInformationPtr &si, const amp::Problem2D *prob, const SimpleCar *car) :
    ob::StateValidityChecker(si)
    {
        si_ = si.get();
        prob_ = prob;
        car_ = car;
    }

bool SimpleCarStateValidityChecker::isValid(const ob::State *state) const
{
    auto compState = state->as<ob::CompoundStateSpace::StateType>();
    auto xyState = compState->as<ob::RealVectorStateSpace::StateType>(0);
    const double x = xyState->values[0];
    const double y = xyState->values[1];

    if (!si_->satisfiesBounds(state))
        return false;
    

    // Define the width and height of the collision box.
    double width = car_->shape_[0];
    double height = car_->shape_[1];

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