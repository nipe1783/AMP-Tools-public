#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/sst/SST.h>
#include <ompl/base/Planner.h>
#include <ompl/base/PlannerStatus.h>
#include "AMPCore.h"
#include "PlannerOmpl.h"
#include "../wsOmpl/AgentOmpl.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

amp::Path2D PlannerOmpl::planGeometric(const amp::Problem2D& prob){

    amp::Path2D path;
    
    // create simple setup
    og::SimpleSetupPtr ss = geometricSimpleSetUp(&prob);
    og::PathGeometric pathOmpl(ss->getSpaceInformation()); // create empty path

    // set planner
    ob::PlannerPtr planner = std::make_shared<og::RRT>(ss->getSpaceInformation());
    ss->setPlanner(planner);

    // run automated setup routine
    ss->setup();

    // solve the instance
    bool solved = ss->solve(5.0);
    if (solved)
    {
        ss->simplifySolution();
        pathOmpl = ss->getSolutionPath();
        // translate ompl path to amp path
        for(int i = 0; i < pathOmpl.getStates().size(); i++){
            path.waypoints.push_back(Eigen::Vector2d(pathOmpl.getStates()[i]->as<ob::SE2StateSpace::StateType>()->getX(), pathOmpl.getStates()[i]->as<ob::SE2StateSpace::StateType>()->getY()));
        }
    }
    else
    {
        OMPL_ERROR("No solution found");
    }
    return path;
}

std::tuple<amp::Path2D, double, double> PlannerOmpl::planGeometric(const amp::Problem2D& prob, bool verbose){

    amp::Path2D path;
    double time = 0;
    double pathLength = 0;
    
    // create simple setup
    og::SimpleSetupPtr ss = geometricSimpleSetUp(&prob);
    og::PathGeometric pathOmpl(ss->getSpaceInformation()); // create empty path

    // set planner
    ob::PlannerPtr planner = std::make_shared<og::RRT>(ss->getSpaceInformation());
    ss->setPlanner(planner);

    // run automated setup routine
    ss->setup();

    // solve the instance
    ompl::base::PlannerStatus solved = ss->solve(5.0);
    if (solved)
    {
        ss->simplifySolution();
        pathOmpl = ss->getSolutionPath();

        // get time of planner
        time = ss->getLastPlanComputationTime();

        // translate ompl path to amp path
        for(int i = 0; i < pathOmpl.getStates().size(); i++){
            path.waypoints.push_back(Eigen::Vector2d(pathOmpl.getStates()[i]->as<ob::SE2StateSpace::StateType>()->getX(), pathOmpl.getStates()[i]->as<ob::SE2StateSpace::StateType>()->getY()));
            if(i > 0){
                pathLength += (path.waypoints[i] - path.waypoints[i-1]).norm();
            }
        }
    }
    else
    {
        OMPL_ERROR("No solution found");
    }
    return std::make_tuple(path, time, pathLength);
}

og::SimpleSetupPtr PlannerOmpl::geometricSimpleSetUp(const amp::Problem2D *prob){
    // define agent
    AgentOmpl* a = new AgentOmpl(
        "None",
        "None",
        {0.001, 0.001},
        {prob->q_init[0], prob->q_init[1]},
        {prob->q_goal[0], prob->q_goal[1]}
    );

    // create state space (with bounds)
    auto space(std::make_shared<ob::SE2StateSpace>());
    ob::RealVectorBounds bounds(2);
    bounds.setLow(0, prob->x_min);
    bounds.setHigh(0,prob->x_max);
    bounds.setLow(1, prob->y_min);
    bounds.setHigh(1, prob->y_max);
    space->setBounds(bounds);
 
    // define a simple setup class
    auto ss = std::make_shared<og::SimpleSetup>(space);
    
    // set state validity checker
    ss->setStateValidityChecker(std::make_shared<isStateValid_2D>(ss->getSpaceInformation(), prob, a));
    
    // create start
    ob::ScopedState<ob::SE2StateSpace> start(space);
    start->setX(a->getStartLocation()[0]);
    start->setY(a->getStartLocation()[1]);
    start->setYaw(0.0);

    // create goal
    ob::ScopedState<ob::SE2StateSpace> goal(space);
    goal->setX(a->getGoalLocation()[0]);
    goal->setY(a->getGoalLocation()[1]);
    goal->setYaw(0.0);

    // save start and goal with tolerance
    ss->setStartAndGoalStates(start, goal, 0.1);

    OMPL_INFORM("Successfully Setup the problem instance");
    return ss;
}