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
#include <ompl/extensions/triangle/PropositionalTriangularDecomposition.h>
#include <ompl/control/planners/ltl/PropositionalDecomposition.h>
#include <ompl/control/planners/ltl/Automaton.h>
#include <ompl/control/planners/ltl/ProductGraph.h>
#include <ompl/control/planners/ltl/LTLPlanner.h>
#include <ompl/control/planners/ltl/LTLProblemDefinition.h>
#include <fstream>


namespace fs = std::filesystem;
namespace ob = ompl::base;
namespace oc = ompl::control;

using Polygon = oc::PropositionalTriangularDecomposition::Polygon;
using Vertex = oc::PropositionalTriangularDecomposition::Vertex;

SimpleCarPlanner::SimpleCarPlanner() : bounds_(4) {
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
    bounds_.setLow(0, prob.x_min); //  x lower bound
    bounds_.setHigh(0, prob.x_max); // x upper bound
    bounds_.setLow(1, prob.y_min);  // y lower bound
    bounds_.setHigh(1, prob.y_max); // y upper bound
    bounds_.setLow(2, car.v_min_ * (1 - safetyMargin_[2])/2);  // v lower bound
    bounds_.setHigh(2, car.v_max_ * (1 - safetyMargin_[2])/2); // v upper bound
    bounds_.setLow(3, car.phi_min_ * (1 - safetyMargin_[3])/2);  // phi lower bound
    bounds_.setHigh(3, car.phi_max_ * (1 - safetyMargin_[3])/2); // phi upper bound
    space->as<ob::CompoundStateSpace>()->as<ob::RealVectorStateSpace>(0)->setBounds(bounds_);

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
    bool solved = ss->solve(30.0);
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

oc::SimpleSetupPtr SimpleCarPlanner::LTLsimpleSetUp(const amp::Problem2D *prob){
    // Setting up car agent:
    SimpleCar *car = new SimpleCar(
        "car", 
        "car", 
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

 class MyDecomposition : public oc::PropositionalTriangularDecomposition
 {
 public:
     MyDecomposition(const ob::RealVectorBounds& bounds)
         : oc::PropositionalTriangularDecomposition(bounds) { }
     ~MyDecomposition() override = default;
  
     void project(const ob::State* s, std::vector<double>& coord) const override
     {
         coord.resize(2);
         coord[0] = s->as<ob::SE2StateSpace::StateType>()->getX();
         coord[1] = s->as<ob::SE2StateSpace::StateType>()->getY();
     }
  
     void sampleFullState(const ob::StateSamplerPtr& sampler, const std::vector<double>& coord, ob::State* s) const override
     {
        sampler->sampleUniform(s);
        auto* ws = s->as<ob::SE2StateSpace::StateType>();
        ws->setXY(coord[0], coord[1]);
     }
 };

void addObstaclesAndPropositions(std::shared_ptr<oc::PropositionalTriangularDecomposition> &decomp, amp::Problem2D& prob)
{
    // adding obstacles from problem:
    for(int i = 0; i < prob.obstacles.size(); i++){
        Polygon obstacle(prob.obstacles[i].verticesCCW().size());
        for(int j = 0; j < prob.obstacles[i].verticesCCW().size(); j++){
            obstacle.pts[j] = Vertex(prob.obstacles[i].verticesCCW()[j][0], prob.obstacles[i].verticesCCW()[j][1]);
        }
        decomp->addHole(obstacle);
    }

    // // adding propositions:
    Polygon start(4);
    start.pts[0] = Vertex(prob.q_init[0] - 0.1, prob.q_init[1] - 0.1);
    start.pts[1] = Vertex(prob.q_init[0] + 0.1, prob.q_init[1] - 0.1);
    start.pts[2] = Vertex(prob.q_init[0] + 0.1, prob.q_init[1] + 0.1);
    start.pts[3] = Vertex(prob.q_init[0] - 0.1, prob.q_init[1] + 0.1);
    decomp->addProposition(start);

    Polygon goal(4);
    goal.pts[0] = Vertex(prob.q_goal[0] - 0.1, prob.q_goal[1] - 0.1);
    goal.pts[1] = Vertex(prob.q_goal[0] + 0.1, prob.q_goal[1] - 0.1);
    goal.pts[2] = Vertex(prob.q_goal[0] + 0.1, prob.q_goal[1] + 0.1);
    goal.pts[3] = Vertex(prob.q_goal[0] - 0.1, prob.q_goal[1] + 0.1);
    decomp->addProposition(goal);
}

void SimpleCarPlanner::planLTL(amp::Problem2D& prob, const std::vector<double>& safetyMargin){
    // checking safety margin vector:
    assert(safetyMargin.size() == 4 && "Vector length is not 4, [x, y, v, phi]");
    assert(safetyMargin[0] >= 0 && "x safety margin is negative");
    assert(safetyMargin[1] >= 0 && "y safety margin is negative");
    assert(safetyMargin[2] >= 0 && safetyMargin[2] <= 1 && "v safety margin is not between 0 and 1");
    assert(safetyMargin[3] >= 0 && safetyMargin[3] <= 1 && "phi safety margin is not between 0 and 1");
    safetyMargin_ = safetyMargin;
    std::cout<<"LTL PLANNING"<<std::endl;
    // create simple setup ptr
    oc::SimpleSetupPtr ss = LTLsimpleSetUp(&prob);
    oc::PathControl pathOmpl(ss->getSpaceInformation()); // create empty path
    amp::Path2D path; // empty amp path.

    // create triangulation that ignores obstacle and respects propositions
    std::shared_ptr<oc::PropositionalTriangularDecomposition> ptd = std::make_shared<MyDecomposition>(bounds_);
    // helper method that adds an obstacle, as well as three propositions p0,p1,p2
    addObstaclesAndPropositions(ptd, prob);
    ptd->setup();

    // visit goal then go back to starting position
    auto cosafety = oc::Automaton::SequenceAutomaton(1, {1, 0});
    auto safety = oc::Automaton::AvoidanceAutomaton(1, {});
    // construct product graph (propDecomp x A_{cosafety} x A_{safety})
    auto product(std::make_shared<oc::ProductGraph>(ptd, cosafety));
    // LTLSpaceInformation creates a hybrid space of robot state space x product graph.
    // It takes the validity checker from SpaceInformation and expands it to one that also
    // rejects any hybrid state containing rejecting automaton states.
    // It takes the state propagator from SpaceInformation and expands it to one that
    // follows continuous propagation with setting the next decomposition region
    // and automaton states accordingly.
    //
    // The robot state space, given by SpaceInformation, is referred to as the "lower space".
    auto ltlsi(std::make_shared<oc::LTLSpaceInformation>(ss->getSpaceInformation(), product));
    // LTLProblemDefinition creates a goal in hybrid space, corresponding to any
    // state in which both automata are accepting
    auto pdef(std::make_shared<oc::LTLProblemDefinition>(ltlsi));
    // create a start state
    ob::ScopedState<ob::SE2StateSpace> start(ss->getStateSpace());
    start->setX(prob.q_init[0]);
    start->setY(prob.q_init[1]);
    start->setYaw(0.0);
    // addLowerStartState accepts a state in lower space, expands it to its
    // corresponding hybrid state (decomposition region containing the state, and
    // starting states in both automata), and adds that as an official start state.
    pdef->addLowerStartState(start.get());
    //LTL planner (input: LTL space information, product automaton)
    oc::LTLPlanner ltlPlanner(ltlsi, product);
    ltlPlanner.setProblemDefinition(pdef);
    // attempt to solve the problem within thirty seconds of planning time
    // considering the above cosafety/safety automata, a solution path is any
    // path that visits p2 followed by p0 while avoiding obstacles and avoiding p1.
    ob::PlannerStatus solved = ltlPlanner.ob::Planner::solve(30.0);
    if (solved)
    {
        std::cout << "Found solution:" << std::endl;
        static_cast<oc::PathControl &>(*pdef->getLowerSolutionPath()).printAsMatrix(std::cout);
        std::ofstream outfile("Output.txt");
        if(outfile.is_open()) {
        static_cast<oc::PathControl &>(*pdef->getLowerSolutionPath()).printAsMatrix(outfile);
            outfile.close();
        } 
        else {
            std::cerr << "Unable to open file 'output.txt' for writing." << std::endl;
        }
    }
    else
        std::cout << "No solution found" << std::endl;
    std::cout<<"Finished LTL Planning"<<std::endl;
}