# pragma once
#include "AMPCore.h"
#include <ompl/control/ODESolver.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/sst/SST.h>

namespace ob = ompl::base;
namespace oc = ompl::control;

class SimpleCar{

    public:
        // constructor and destructor:
        SimpleCar(std::string name, std::string dyn, std::vector<double> shape, std::vector<double> s, std::vector<double> g);
        ~SimpleCar();

        // fields:
        std::string name_;
        std::string dynamics_;
        std::vector<double> shape_; // [width, length]
        std::vector<double> start_; // [x, y, theta]
        std::vector<double> goal_; // [x, y, theta]
        double phi_min_;
        double phi_max_;
        double v_min_;
        double v_max_;

        // methods:
        /**
         * @brief Describes the dynamics of the simple car
         *
         * @param q current state of the system
         * @param control control input
         * @param qdot derivative of the state
         * @return void
         */
        void SecondOrderODE(const oc::ODESolver::StateType& q, const oc::Control* control, oc::ODESolver::StateType& qdot);

        /**
         * @brief Applies the controls to the agent. Wraps the angle to [0, 2pi].
         *
         * @param state current state of agent.
         * @param control controls applied to agent.
         * @param duration duration of control application.
         * @param result result of integration.
         * @return Return Type Description of return value
         */
        void SecondOrderCarODEPostIntegration (const ob::State* /*state*/, const oc::Control* /*control*/, const double /*duration*/, ob::State *result);
};