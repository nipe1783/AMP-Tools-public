#include "MyCentralizedMultiAgentRRT.h"

#include <random>
#include <chrono>
#include "Helper.h"
#include "EnvironmentHelper.h"
#include "NDConfigurationSpace.h"
#include <xtensor/xtensor.hpp>
#include <vector>
#include "MyAstarAlgorithm.h"

namespace amp{

    MultiAgentPath2D MyCentralizedMultiAgentRRT::plan(const amp::MultiAgentProblem2D& problem){

        MultiAgentPath2D path(problem.numAgents());
        amp::Graph<double> graph;
        amp::LookupSearchHeuristic heuristic;
        std::vector<Eigen::VectorXd> nodes;

        double p = 0.05;
        double r = 0.1;

        // construct composed cSpace
        int dofSystem = problem.numAgents() * ( problem.agent_properties.size() - 1);
        int dofRobot = problem.agent_properties.size() - 1;
        Eigen::VectorXd lowerBounds(dofSystem);
        Eigen::VectorXd upperBounds(dofSystem);
        for(int i = 0; i < upperBounds.size(); i += dofSystem)
        {   
            lowerBounds[i] = problem.x_min;
            lowerBounds[i+1] = problem.y_min;
            upperBounds[i] = problem.x_max;
            upperBounds[i+1] = problem.y_max;

        }
        NDConfigurationSpace cSpace(lowerBounds, upperBounds);
        cSpace.construct(problem);

        // Sample CSpace
        Eigen::VectorXd nodeStart(problem.numAgents() * dofRobot);
        for(int i = 0; i < problem.numAgents() *  dofRobot; i += dofRobot) {
            nodeStart[i] = problem.agent_properties[i/dofRobot].q_init[0];
            nodeStart[i+1] = problem.agent_properties[i/dofRobot].q_init[1];
        }
        nodes.push_back(nodeStart);
        Eigen::VectorXd nodeGoal(problem.numAgents() * dofRobot);
        for(int i = 0; i < problem.numAgents() *  dofRobot; i += dofRobot) {
            nodeGoal[i] = problem.agent_properties[i/dofRobot].q_goal[0];
            nodeGoal[i+1] = problem.agent_properties[i/dofRobot].q_goal[1];
        }
        heuristic.heuristic_values[0] = Helper().NDDistance(nodeStart, nodeGoal);

        for(int i = 0; i < 100; i ++){
            // generate a randome sample in the centralized cspace
            generateRandomSample(cSpace, problem, nodes, p, r);

            // find nearest node in the centralized cspace and update graph.
            findNearestNode(problem, nodes, graph, heuristic);

            // check if the goal is reached
            if(Helper().NDDistance(nodes[nodes.size() - 1], nodeGoal) < 0.1) {
                std::cout<<"goal reached"<<std::endl;
                break;
            }
        }

        // use A* to find the shortest path between the start and goal nodes.
        std::cout<<"node size: "<<nodes.size()<<std::endl;
        std::cout<<"heurisitc size: "<<heuristic.heuristic_values.size()<<std::endl;
        MyAStarAlgorithm aStar;
        amp::ShortestPathProblem prob_astar = {std::make_shared<amp::Graph<double>>(graph), 0, static_cast<amp::Node>(nodes.size() - 1)};
        AStar::GraphSearchResult result = aStar.search(prob_astar, heuristic);

        // create robot paths from samples.
        for(const auto& node : result.node_path) {
            Eigen::VectorXd currentNode = nodes[node];
            for (int j = 0; j < problem.numAgents(); j++) {
                int index = j * dofRobot; // Calculate the starting index for the current agent's DOF in the node
                Eigen::Vector2d waypoint(currentNode[index], currentNode[index + 1]);
                path.agent_paths[j].waypoints.push_back(waypoint);
            }
        }

        return path;
    }

    void MyCentralizedMultiAgentRRT::findNearestNode(const NDConfigurationSpace& cSpace, const MultiAgentProblem2D& problem, std::vector<Eigen::VectorXd>& nodes, amp::Graph<double>& graph, amp::LookupSearchHeuristic& heuristic, const double& stepSize, const int& numberOfChecks){
        double min_distance = std::numeric_limits<double>::max();
        int nearest_node_idx = -1;

        for(int i = 0; i < nodes.size() - 1; i++) {
            double distance = Helper().NDDistance(nodes[i], nodes[nodes.size() - 1]);
            if(distance < min_distance) {
                min_distance = distance;
                nearest_node_idx = i;
            }
        }
        if(nearest_node_idx == -1) {
            nodes.pop_back();
            return;
        }
        else{
            // check if path along two c space states is valid.
            Eigen::VectorXd stepNode = nodes[nearest_node_idx];
            for(int j = 0; j < numberOfChecks; j++){
                stepNode = Helper().interpolate(stepNode, nodes[nodes.size() - 1], stepSize);
                if(cSpace.inCollision(stepNode, problem)) {
                    nodes.pop_back();
                    return;
                }
            }
            nodes.pop_back();
            nodes.push_back(stepNode);
            std::cout<<nearest_node_idx<< "->"<<nodes.size() - 1<<std::endl;
            graph.connect(nearest_node_idx, nodes.size() - 1, min_distance);
            heuristic.heuristic_values[nodes.size() - 1] = Helper().NDDistance(nodes[nodes.size() - 1], nodes[0]);
        }
    }

    void MyCentralizedMultiAgentRRT::generateRandomSample(const NDConfigurationSpace& cSpace, const MultiAgentProblem2D& problem, std::vector<Eigen::VectorXd>& nodes, const double p, const double r){
        std::random_device rd;
        std::default_random_engine generator(rd());
        double x, y, theta;
        std::uniform_real_distribution<double> distributionX(problem.x_min, problem.x_max);
        std::uniform_real_distribution<double> distributionY(problem.y_min, problem.y_max);
        std::uniform_real_distribution<double> distributionTheta(0, 2*M_PI);
        std::uniform_real_distribution<double> distributionGoal(0, 1);

        int dofRobot = problem.agent_properties.size() - 1;
        Eigen::VectorXd sample(problem.numAgents() * dofRobot);

        if(distributionGoal(generator) < p) {
            for(int i = 0; i < dofRobot * problem.numAgents(); i += dofRobot) {
                sample[i] = problem.agent_properties[i/dofRobot].q_goal[0];
                sample[i+1] = problem.agent_properties[i/dofRobot].q_goal[1];
            }
        } else {
            for(int i = 0; i < dofRobot * problem.numAgents(); i += dofRobot) {
                sample[i] = distributionX(generator);
                sample[i+1] = distributionY(generator);
            }
        }
        // for(int i = 0; i < sample.size(); i += dofRobot) {
        //     x = sample[i];
        //     y = sample[i+1];
        //     // check to see if robot i is in collision of any obstacles
        //     for(const auto& obstacle : problem.obstacles) {
        //         Obstacle2D expandedObstacle = Helper().expandObstacle(obstacle, problem.agent_properties[i/dofRobot].radius);
        //         if(EnvironmentHelper().inCollision(Eigen::Vector2d(x, y), expandedObstacle)) {
        //             std::cout<<"robot "<<i/dofRobot<<" is in collision with obstacle."<<std::endl;
        //             generateRandomSample(problem, nodes, p, r);
        //             return;
        //         }
        //     }
        //     // check to see if robot i is in collision with any other robots
        //     for(int j = 0; j < sample.size(); j += dofRobot) {
        //         if(i != j) {
        //         }
        //     }
        // }

        if(cSpace.inCollision(sample, problem)) {
            std::cout<<"sample is in collision"<<std::endl;
            generateRandomSample(cSpace, problem, nodes, p, r);
            return;
        }

        // sample is valid, add it to nodes.
        nodes.push_back(sample);
    }

}
