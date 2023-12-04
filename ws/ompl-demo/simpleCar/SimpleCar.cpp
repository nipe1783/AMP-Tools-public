#include "SimpleCar.h"
#include "AMPCore.h"
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/sst/SST.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <stdexcept> 

namespace ob = ompl::base;
namespace oc = ompl::control;

SimpleCar::SimpleCar(std::string name, std::string dyn, std::vector<double> shape, std::vector<double> s, std::vector<double> g) {
    name_ = name;
    dynamics_ = dyn;
    std::cout<<"DYNAMICS: "<<dynamics_<<std::endl;
    shape_ = shape;
    start_ = s;
    goal_ = g;
    phi_min_ = -M_PI/10;
    phi_max_ = M_PI/10;
    v_min_ = 0;
    v_max_ = 1;
}

void SimpleCar::SecondOrderODE (const oc::ODESolver::StateType& q, const oc::Control* control, oc::ODESolver::StateType& qdot){
    // q = x, y, v, phi, theta
    // c = a, phidot
    const double *u = control->as<oc::RealVectorControlSpace::ControlType>()->values;
    // state params
    const double v = q[2];
    const double phi = q[3];
    const double theta = q[4];

    // Zero out qdot
    qdot.resize (q.size (), 0);
 
    // vehicle model
    qdot[0] = v * cos(theta); // xdot
    qdot[1] = v * sin(theta); // ydot
    qdot[2] = u[0]; // vdot
    qdot[3] = u[1]; // phidot
    qdot[4] = (v / shape_[1]) * tan(phi); // thetadot
}

void SimpleCar::SecondOrderCarODEPostIntegration (const ob::State* /*state*/, const oc::Control* /*control*/, const double /*duration*/, ob::State *result)
{
    // wrap the angle
    ob::CompoundState* cs = result->as<ob::CompoundState>();
    ob::SO2StateSpace::StateType* angleState1 = cs->as<ob::SO2StateSpace::StateType>(1);
    ob::SO2StateSpace SO2;
    SO2.enforceBounds(angleState1);
}