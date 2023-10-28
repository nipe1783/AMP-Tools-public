#include "AMPCore.h"
#include "hw/HW7.h"
#include "hw/HW6.h"
#include "MyGoalBiasedPRM2D.h"
#include "EnvironmentHelper.h"
#include "Helper.h"
#include "MyAstarAlgorithm.h"
#include <random>
#include <chrono>
#include <tuple>


namespace amp{
    
    Path2D MyGoalBiasedPRM2D::plan(const Problem2D& prob){

        // start timer:
        auto start_time = std::chrono::high_resolution_clock::now();

        // constants:
        const double r = 2;
        const int n = 500;

        Path2D path;

        // step 1: construct cspace and create graph
        // std::unique_ptr<amp::GridCSpace2D> grid_cspace  = EnvironmentHelper().constructCSpacePRB(prob, 0.25, 0.15);
        amp::Graph<double> graph;
        amp::LookupSearchHeuristic heuristic;
        std::vector<Eigen::Vector2d> nodes;
        nodes.push_back(prob.q_init);
        nodes.push_back(prob.q_goal);

        // step 2: sample cspace
        sampleWorkSpace(prob, n, nodes);

        // step 3: connect nodes and remove invalid nodes
        for(int i = 0; i < nodes.size(); i++){
            linkNearestNodes(i, nodes, r, graph, heuristic, prob);
        }

        // step 4: find shortest from graph
        MyAStarAlgorithm aStar;
        amp::ShortestPathProblem prob_astar = {std::make_shared<amp::Graph<double>>(graph), 0, 1};
        AStar::GraphSearchResult result = aStar.search(prob_astar, heuristic);
        for(const auto& node : result.node_path) {
            path.waypoints.push_back(nodes[node]);
        }

        // end timer:
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        std::cout << "Plan function took: " << duration.count() << " milliseconds" << std::endl;

        
        return path;
    }

    std::tuple<Path2D, double, double> MyGoalBiasedPRM2D::plan(const Problem2D& prob, const double r, const int n, bool smooth){

        // start timer:
        auto start_time = std::chrono::high_resolution_clock::now();

        Path2D path;

        // step 1: construct cspace and create graph
        // std::unique_ptr<amp::GridCSpace2D> grid_cspace  = EnvironmentHelper().constructCSpacePRB(prob, 0.25, 0.15);
        amp::Graph<double> graph;
        amp::LookupSearchHeuristic heuristic;
        std::vector<Eigen::Vector2d> nodes;
        nodes.push_back(prob.q_init);
        nodes.push_back(prob.q_goal);

        // step 2: sample cspace
        sampleWorkSpace(prob, n, nodes);
        std::cout<<"nodes size: "<<nodes.size()<<std::endl;

        // step 3: connect nodes and remove invalid nodes
        for(int i = 0; i < nodes.size(); i++){
            linkNearestNodes(i, nodes, r, graph, heuristic, prob);
        }

        // step 4: find shortest from graph
        double distance = 0;
        MyAStarAlgorithm aStar;
        amp::ShortestPathProblem prob_astar = {std::make_shared<amp::Graph<double>>(graph), 0, 1};
        std::cout<<heuristic.heuristic_values.size()<<std::endl;
        AStar::GraphSearchResult result = aStar.search(prob_astar, heuristic);
        for(const auto& node : result.node_path) {
            path.waypoints.push_back(nodes[node]);
            if(node > 0){
                distance += (nodes[node - 1] - nodes[node]).norm();
            }
        }
        if(smooth){
            smoothPath(100, path.waypoints, prob);
        }

        // end timer:
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        std::cout << "Plan function took: " << duration.count() << " milliseconds" << std::endl;

        return std::make_tuple(path, distance, duration.count());
    }

    void MyGoalBiasedPRM2D::sampleWorkSpace(const Problem2D& prob, int num_samples, std::vector<Eigen::Vector2d>& nodes){
        double xmin = prob.x_min;
        double xmax = prob.x_max;
        double ymin = prob.y_min;
        double ymax = prob.y_max;
        double x;
        double y;
        bool in_collision = false;
        std::random_device rd;
        std::default_random_engine generator(rd());
        std::uniform_real_distribution<double> distribution_x(xmin, xmax);
        std::uniform_real_distribution<double> distribution_y(ymin, ymax);
        for (int i = 0; i < num_samples; i++){
            x = distribution_x(generator);
            y = distribution_y(generator);
            for(int j = 0; j < prob.obstacles.size(); j++){
                if(EnvironmentHelper().inCollision(Eigen::Vector2d(x, y), prob.obstacles[j])){
                    in_collision = true;
                    break;
                }
            }
            if(!in_collision){
                nodes.push_back(Eigen::Vector2d(x, y));
            }
            in_collision = false;
        }
    }

    void MyGoalBiasedPRM2D::linkNearestNodes(const int node_idx, std::vector<Eigen::Vector2d>& nodes, const double r, amp::Graph<double>& graph, amp::LookupSearchHeuristic& heuristic, const Problem2D& prob){
        double distance;
        heuristic.heuristic_values[node_idx] = calculateHeuristic(prob, nodes[node_idx]);
        for(int i = 0; i < node_idx; i++){
            distance = (nodes[i] - nodes[node_idx]).norm();
            if(distance < r && distance > 0){
                if(!Helper().intersects(prob, nodes[node_idx], nodes[i])){
                    graph.connect(node_idx, i, distance);
                }
            }
        }
        for(int i = node_idx; i < nodes.size(); i++){
            distance = (nodes[i] - nodes[node_idx]).norm();
            if(distance < r && distance > 0){
                if(!Helper().intersects(prob, nodes[node_idx], nodes[i])){
                    graph.connect(node_idx, i, distance);
                }
            }
        }
    }

    double MyGoalBiasedPRM2D::calculateHeuristic(const Problem2D& prob, const Eigen::Vector2d& node){
        double distance = (node - prob.q_goal).norm();
        return distance;
    }

    void MyGoalBiasedPRM2D::smoothPath(int iterations, std::vector<Eigen::Vector2d>& nodes, const Problem2D& prob){
        for(int i = 0; i < iterations; i++){
            // Randomly select two distinct node indices
            int node1_idx = rand() % nodes.size();
            int node2_idx = rand() % nodes.size();
            while(node1_idx == node2_idx) {
                node2_idx = rand() % nodes.size();
            }

            // Sort the indices
            if (node1_idx > node2_idx) {
                std::swap(node1_idx, node2_idx);
            }

            Eigen::Vector2d coord_1 = nodes[node1_idx];
            Eigen::Vector2d coord_2 = nodes[node2_idx];

            // Check if the path between coord_1 and coord_2 intersects with any obstacle
            if(!Helper().intersects(prob, coord_1, coord_2)) {
                // Remove the nodes between coord_1 and coord_2 from the nodes vector
                nodes.erase(nodes.begin() + node1_idx + 1, nodes.begin() + node2_idx);
            }
        }
    }

}