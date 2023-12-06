#include "SimpleCarPlanner.h"
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/sst/SST.h>
#include <ompl/control/SimpleSetup.h>
#include "AMPCore.h"
#include "SimpleCarStateValidityChecker.h"
#include  "SimpleCar.h"
#include "SimpleCarGoalRegion.h"
#include "../benchmark/PostProcessing.h"
#include <functional>
#include <stdexcept>
#include <cassert>


namespace fs = std::filesystem;
namespace ob = ompl::base;
namespace oc = ompl::control;

SimpleCarPlanner::SimpleCarPlanner() {
}

SimpleCarPlanner::~SimpleCarPlanner() {
}

ob::StateSpacePtr SimpleCarPlanner::createStateSpace(const amp::Problem2D& prob, const SimpleCar& car) {
    // setting simpleCar state space:
    ob::StateSpacePtr space = std::make_shared<ob::CompoundStateSpace>();
    // x, y, velocity, turning angle
    space->as<ob::CompoundStateSpace>()->addSubspace(ob::StateSpacePtr(new ob::RealVectorStateSpace(4)), 1.0);
    // car heading
    space->as<ob::CompoundStateSpace>()->addSubspace(ob::StateSpacePtr(new ob::SO2StateSpace()), 1.0);
    space->as<ob::CompoundStateSpace>()->lock();

    // set the bounds for the RealVectorStateSpace 
    ob::RealVectorBounds bounds(4);
    bounds.setLow(0, prob.x_min); //  x lower bound
    bounds.setHigh(0, prob.x_max); // x upper bound
    bounds.setLow(1, prob.y_min);  // y lower bound
    bounds.setHigh(1, prob.y_max); // y upper bound
    bounds.setLow(2, car.v_min_ * (1 - safetyMargin_[2])/2);  // v lower bound
    bounds.setHigh(2, car.v_max_ * (1 - safetyMargin_[2])/2); // v upper bound
    bounds.setLow(3, car.phi_min_ * (1 - safetyMargin_[3])/2);  // phi lower bound
    bounds.setHigh(3, car.phi_max_ * (1 - safetyMargin_[3])/2); // phi upper bound
    std::cout<<"pre phi min: "<< car.phi_min_<<std::endl; // debug
    std::cout<<"pre phi max: "<< car.phi_max_<<std::endl; // debug
    std::cout<<"phi min: "<< car.phi_min_ * (1 - safetyMargin_[3])/2<<std::endl;
    std::cout<<"phi max: "<< car.phi_max_ * (1 - safetyMargin_[3])/2<<std::endl;
    space->as<ob::CompoundStateSpace>()->as<ob::RealVectorStateSpace>(0)->setBounds(bounds);

    return space;
}

amp::Path2D SimpleCarPlanner::planKinodynamic(const amp::Problem2D& prob, const std::vector<double>& safetyMargin) {
    // checking safety margin vector:
    assert(safetyMargin.size() == 4 && "Vector length is not 4, [x, y, v, phi]");
    assert(safetyMargin[0] >= 0 && "x safety margin is negative");
    assert(safetyMargin[1] >= 0 && "y safety margin is negative");
    assert(safetyMargin[2] >= 0 && safetyMargin[2] <= 1 && "v safety margin is not between 0 and 1");
    assert(safetyMargin[3] >= 0 && safetyMargin[3] <= 1 && "phi safety margin is not between 0 and 1");
    safetyMargin_ = safetyMargin;
    // create simple setup object
    oc::SimpleSetupPtr ss = kinodynamicSimpleSetUp(&prob);
    oc::PathControl pathOmpl(ss->getSpaceInformation()); // create empty path
    amp::Path2D path; // empty amp path.
    // set planner
    ob::PlannerPtr planner = nullptr;
    planner = std::make_shared<oc::SST>(ss->getSpaceInformation());
    ss->setPlanner(planner);
    // run automated setup routine
    ss->setup();
    // solve the instance
    bool solved = ss->solve(5.0);
    if (solved){
        pathOmpl = ss->getSolutionPath();
        // translate ompl path to amp path
        for(int i = 0; i < pathOmpl.getStates().size(); i++){
            path.waypoints.push_back(Eigen::Vector2d(pathOmpl.getStates()[i]->as<ob::SE2StateSpace::StateType>()->getX(), pathOmpl.getStates()[i]->as<ob::SE2StateSpace::StateType>()->getY()));
        }
        write2sys(ss);
    }
    else{
        OMPL_ERROR("No solution found");
    }
    return path;
}

oc::ControlSpacePtr SimpleCarPlanner::createControlSpace(ob::StateSpacePtr &space) {
    // setting simpleCar control space:
    auto cspace(std::make_shared<oc::RealVectorControlSpace>(space, 2));
    
    // set the bounds for the control space
    ob::RealVectorBounds cbounds(2);
    cbounds.setLow(-1);
    cbounds.setHigh(1);
    cspace->setBounds(cbounds);

    return cspace;
}

oc::SimpleSetupPtr SimpleCarPlanner::kinodynamicSimpleSetUp(const amp::Problem2D *prob) {

    // Setting up car agent:
    SimpleCar *car = new SimpleCar(
        "car", 
        "car", 
        {0.5 ,0.5 }, 
        {prob->q_init[0], prob->q_init[1]}, 
        {prob->q_goal[0], prob->q_init[1]}
    );

    // creating state space:
    ob::StateSpacePtr space = createStateSpace(*prob, *car);
    // creating control space:
    oc::ControlSpacePtr cspace = createControlSpace(space);
    // creating simple setup:
    auto ss = std::make_shared<oc::SimpleSetup>(cspace);
    // adding validity checker:
    ss->setStateValidityChecker(ob::StateValidityCheckerPtr(new SimpleCarStateValidityChecker(ss->getSpaceInformation(), prob, car, safetyMargin_)));
    // simulating system dynamics:
    auto odeFunction = std::bind(&SimpleCar::SecondOrderODE, car, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    auto odeSolver = std::make_shared<oc::ODEBasicSolver<>>(ss->getSpaceInformation(), odeFunction);
    auto postIntegrationFunction = std::bind(&SimpleCar::SecondOrderCarODEPostIntegration, car, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4);
    ss->setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver, postIntegrationFunction));

    // set starting state:
    ob::ScopedState<> start(space);
    start[0] = car->start_[0]; // x
    start[1] = car->start_[1]; // y
    start[2] = 0; // v
    start[3] = 0; // phi
    start[4] = 0; // theta
    // create goal region
    ob::GoalPtr goal (new GoalRegion2ndOrderCar(ss->getSpaceInformation(), prob->q_goal[0], prob->q_goal[1]));
    // save start and goal
    ss->setStartState(start);
    ss->setGoal(goal);

    OMPL_INFORM("Successfully Setup the problem instance");
    return ss;
}