# pragma once
#include <boost/geometry.hpp>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <set>
#include "AMPCore.h"
#include "ObstacleOmpl.h"

class AgentOmpl
{
    public:
        AgentOmpl(std::string name, std::string dyn, std::vector<double> shape, std::vector<double> s, std::vector<double> g) {
            name_ = name;
            dynamics_ = dyn;
            shape_ = shape;
            start_ = s;
            goal_ = g;
        }
        std::string getName() const {return name_;};
        std::string getDynamics() const {return dynamics_;};
        std::vector<double> getShape() const {return shape_;};
        std::vector<double> getStartLocation() const {return start_;};
        std::vector<double> getGoalLocation() const {return goal_;};
    private:
        std::string name_;
        std::string dynamics_;
        std::vector<double> shape_;
        std::vector<double> start_;
        std::vector<double> goal_;
};