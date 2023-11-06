#include "MyDecentralizedMultiAgentRRT.h"
#include "Helper.h"
#include "MyAstarAlgorithm.h"

using namespace amp;

amp::MultiAgentPath2D MyDecentralizedMultiAgentRRT::plan(const amp::MultiAgentProblem2D& problem){

    MultiAgentPath2D path(problem.numAgents());
    double p = 0.25;
    double r = 0.1;
    double epsilon = 0.25;
    int n = 7500;
    double stepSize = .02;
    double padding = 2;
    int numberOfChecks = 20;
    int dofSystem = 2;

    // initializing agent locations for each timeStep
    std::vector<std::vector<Eigen::Vector2d>> agentLocations(n, std::vector<Eigen::Vector2d>(problem.numAgents(), Eigen::Vector2d(0, 0)));
    for(int i = 0; i < problem.numAgents(); i++){
        for(int j = 0; j < n; j++){
            agentLocations[j][i] = Eigen::Vector2d(problem.agent_properties[i].q_init[0], problem.agent_properties[i].q_init[1]);
        }
    }

    // loop through each robot, finding a path for each.
    for(int i = 0; i < problem.numAgents(); i++){

        std::vector<Eigen::VectorXd> nodes;
        amp::Graph<double> graph;
        amp::LookupSearchHeuristic heuristic;
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
        Eigen::VectorXd nodeStart(dofSystem);
        Eigen::VectorXd nodeGoal(dofSystem);
        nodeStart[0] = problem.agent_properties[i].q_init[0];
        nodeStart[1] = problem.agent_properties[i].q_init[1];
        nodeGoal[0] = problem.agent_properties[i].q_goal[0];
        nodeGoal[1] = problem.agent_properties[i].q_goal[1];
        nodes.push_back(nodeStart);
        heuristic.heuristic_values[0] = Helper().NDDistance(nodeStart, nodeGoal);

        // loop through each timeStep
        for(int j = 0; j < n; j++){
            // generate random sample in the decentralized cspace
            generateRandomSample(cSpace, problem, nodes, p, r, padding);

            // find nearest node in the centralized cspace and update graph.
            findNearestNode(cSpace, problem, nodes, graph, heuristic, stepSize, numberOfChecks, padding, agentLocations[j], i);

            // check if the goal is reached
            if(Helper().NDDistance(nodes[nodes.size() - 1], nodeGoal) < epsilon) {
                nodes.push_back(nodeGoal);
                graph.connect(nodes.size() - 2, nodes.size() - 1, Helper().NDDistance(nodes[nodes.size() - 2], nodes[nodes.size() - 1]));
                heuristic.heuristic_values[nodes.size() - 1] = 0;
                std::cout<<"goal reached for robot: "<< i <<std::endl;
                break;
            }
        }

        // find the path from the graph
        MyAStarAlgorithm aStar;
        amp::ShortestPathProblem prob_astar = {std::make_shared<amp::Graph<double>>(graph), 0, static_cast<amp::Node>(nodes.size() - 1)};
        AStar::GraphSearchResult result = aStar.search(prob_astar, heuristic);

        int j = 0;
        for(const auto& node : result.node_path) {
            Eigen::VectorXd currentNode = nodes[node];
            Eigen::Vector2d waypoint(currentNode[0], currentNode[1]);
            path.agent_paths[i].waypoints.push_back(waypoint);
            agentLocations[j][i] = waypoint;
            j++;
        }
        for(int k = j; k < n; k++){
            agentLocations[k][i] = agentLocations[j][i];
        }

        graph.clear();
        heuristic.heuristic_values.clear();
        nodes.clear();
    }
    return path;
}

void MyDecentralizedMultiAgentRRT::findNearestNode(const NDConfigurationSpace& cSpace, const MultiAgentProblem2D& problem, std::vector<Eigen::VectorXd>& nodes, amp::Graph<double>& graph, amp::LookupSearchHeuristic& heuristic, const double& stepSize, const int& numberOfChecks, const double& padding, const  std::vector<Eigen::Vector2d>& agentLocations, const int& agentId){
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
            if(cSpace.inCollision(stepNode, problem, padding, agentLocations, agentId)) {
                nodes.pop_back();
                return;
            } 
        }
        if(!cSpace.inCollision(stepNode, problem, padding, agentLocations, agentId)){
            nodes.pop_back();
            nodes.push_back(stepNode);
            graph.connect(nearest_node_idx, nodes.size() - 1, min_distance);
            heuristic.heuristic_values[nodes.size() - 1] = Helper().NDDistance(nodes[nodes.size() - 1], nodes[0]);
        }
    }
}

 void MyDecentralizedMultiAgentRRT::generateRandomSample(const NDConfigurationSpace& cSpace, const MultiAgentProblem2D& problem, std::vector<Eigen::VectorXd>& nodes, const double p, const double r, const double& padding){
    std::random_device rd;
    std::default_random_engine generator(rd());
    std::uniform_real_distribution<double> distributionX(problem.x_min, problem.x_max);
    std::uniform_real_distribution<double> distributionY(problem.y_min, problem.y_max);
    std::uniform_real_distribution<double> distributionGoal(0, 1);

    int dofRobot = 2;
    Eigen::VectorXd sample(dofRobot);

    if(distributionGoal(generator) < p) {
        sample[0] = problem.agent_properties[0].q_goal[0];
        sample[1] = problem.agent_properties[1].q_goal[1];
    } else {
        sample[0] = distributionX(generator);
        sample[1] = distributionY(generator);
    }
    nodes.push_back(sample);
 }