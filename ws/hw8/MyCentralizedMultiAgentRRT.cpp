#include "MyCentralizedMultiAgentRRT.h"
#include <random>
#include <chrono>
#include "Helper.h"
#include "EnvironmentHelper.h"
#include "NDConfigurationSpace.h"
#include <xtensor/xtensor.hpp>
#include <vector>
#include "MyAstarAlgorithm.h"
#include <chrono>

namespace amp{

    MultiAgentPath2D MyCentralizedMultiAgentRRT::plan(const amp::MultiAgentProblem2D& problem){

        MultiAgentPath2D path(problem.numAgents());
        amp::Graph<double> graph;
        amp::LookupSearchHeuristic heuristic;
        std::vector<Eigen::VectorXd> nodes;

        double p = 0.05;
        double r = 0.1;
        double epsilon = .25;
        int n = 75000;
        double stepSize = .1;
        double padding = .2;
        int numberOfChecks = 1;

        // construct composed cSpace
        int dofSystem = problem.numAgents() * 2;
        int dofRobot = 2;
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
        for(int i = 0; i < n; i ++){
            // generate a randome sample in the centralized cspace
            generateRandomSample(cSpace, problem, nodes, p, r, padding);
            
            // find nearest node in the centralized cspace and update graph.
            findNearestNode(cSpace, problem, nodes, graph, heuristic, stepSize, numberOfChecks, padding);

            // check if the goal is reached
            if(Helper().NDDistance(nodes[nodes.size() - 1], nodeGoal) < epsilon) {
                nodes.push_back(nodeGoal);
                graph.connect(nodes.size() - 2, nodes.size() - 1, Helper().NDDistance(nodes[nodes.size() - 2], nodes[nodes.size() - 1]));
                heuristic.heuristic_values[nodes.size() - 1] = 0;
                std::cout<<"goal reached"<<std::endl;
                break;
            }
        }

        // add goal node
        nodes.push_back(nodeGoal);
        graph.connect(nodes.size() - 2, nodes.size() - 1, Helper().NDDistance(nodes[nodes.size() - 2], nodes[nodes.size() - 1]));
        heuristic.heuristic_values[nodes.size() - 1] = 0;
        
        // use A* to find the shortest path between the start and goal nodes.
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

        // Visualizer::makeFigure(problem, path);
        // Visualizer::showFigures();
        return path;
    }

    std::tuple<MultiAgentPath2D, double, double> MyCentralizedMultiAgentRRT::plan(const amp::MultiAgentProblem2D& problem, bool verbose = true){

        // start timer:
        auto start_time = std::chrono::high_resolution_clock::now();

        MultiAgentPath2D path(problem.numAgents());
        amp::Graph<double> graph;
        amp::LookupSearchHeuristic heuristic;
        std::vector<Eigen::VectorXd> nodes;

        double p = 0.05;
        double r = 0.1;
        double epsilon = 0.25;
        int n = 7500;
        double stepSize = .05;
        double padding = .075;
        int numberOfChecks = 10;

        // construct composed cSpace
        int dofSystem = problem.numAgents() * 2;
        int dofRobot = 2;
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
        for(int i = 0; i < n; i ++){
            // generate a randome sample in the centralized cspace
            generateRandomSample(cSpace, problem, nodes, p, r, padding);
            
            // find nearest node in the centralized cspace and update graph.
            findNearestNode(cSpace, problem, nodes, graph, heuristic, stepSize, numberOfChecks, padding);

            // check if the goal is reached
            if(Helper().NDDistance(nodes[nodes.size() - 1], nodeGoal) < epsilon) {
                nodes.push_back(nodeGoal);
                graph.connect(nodes.size() - 2, nodes.size() - 1, Helper().NDDistance(nodes[nodes.size() - 2], nodes[nodes.size() - 1]));
                heuristic.heuristic_values[nodes.size() - 1] = 0;
                std::cout<<"goal reached"<<std::endl;
                break;
            }
        }
        
        // use A* to find the shortest path between the start and goal nodes.
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

        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        return std::make_tuple(path, nodes.size(), duration.count());
    }

    void MyCentralizedMultiAgentRRT::findNearestNode(const NDConfigurationSpace& cSpace, const MultiAgentProblem2D& problem, std::vector<Eigen::VectorXd>& nodes, amp::Graph<double>& graph, amp::LookupSearchHeuristic& heuristic, const double& stepSize, const int& numberOfChecks, const double& padding){
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
                if(cSpace.inCollision(stepNode, problem, padding)) {
                    nodes.pop_back();
                    return;
                }
            }
            if(!cSpace.inCollision(stepNode, problem, padding)){
                nodes.pop_back();
                nodes.push_back(stepNode);
                graph.connect(nearest_node_idx, nodes.size() - 1, min_distance);
                heuristic.heuristic_values[nodes.size() - 1] = Helper().NDDistance(nodes[nodes.size() - 1], nodes[0]);
            }
        }
    }

    void MyCentralizedMultiAgentRRT::generateRandomSample(const NDConfigurationSpace& cSpace, const MultiAgentProblem2D& problem, std::vector<Eigen::VectorXd>& nodes, const double p, const double r, const double& padding){
        std::random_device rd;
        std::default_random_engine generator(rd());
        std::uniform_real_distribution<double> distributionX(problem.x_min, problem.x_max);
        std::uniform_real_distribution<double> distributionY(problem.y_min, problem.y_max);
        std::uniform_real_distribution<double> distributionGoal(0, 1);

        int dofRobot = 2;
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

        nodes.push_back(sample);
    }

}
