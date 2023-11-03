#include "AMPCore.h"
#include "hw/HW7.h"
#include "hw/HW6.h"
#include "MyGoalBiasRRT2D.h"
#include "EnvironmentHelper.h"
#include "Helper.h"
#include "MyAstarAlgorithm.h"
#include <random>
#include <chrono>
#include <tuple>


namespace amp{

    Path2D MyGoalBiasRRT2D::plan(const Problem2D& prob){
    
        // start timer:
        auto start_time = std::chrono::high_resolution_clock::now();

        // constants:
        int counter = 0;
        int sample_idx;
        int nearest_idx;
        double distance;

        // 0.5, 5000, 0.05, 0.5
        int n = 5000;
        double p = 0.05;
        double r = 0.5;
        double threshold = 0.5;
        int prev_node_idx = 0;
        Path2D path;

        // step 1: construct cspace and create graph
        amp::Graph<double> graph;
        amp::LookupSearchHeuristic heuristic;
        std::vector<Eigen::Vector2d> nodes;
        nodes.push_back(prob.q_init);
        heuristic.heuristic_values[0] = calculateHeuristic(prob, nodes[0]);

        while(counter < n){
            counter++;
            // step 2: sample workspace with random sample
            sample_idx = generateRandomSample(prob, nodes, p, r);

            // step 3: find nearest node
            nearest_idx = findNearestNode(sample_idx, nodes, r, prob);

            // step 4: check if the path between the nearest node and the sample is valid
            if(!Helper().intersects(prob, nodes[nearest_idx], nodes[sample_idx])){
                // step 5: add edge to graph
                distance = (nodes[nearest_idx] - nodes[sample_idx]).norm();
                graph.connect(nearest_idx, sample_idx, distance);
                heuristic.heuristic_values[sample_idx] = calculateHeuristic(prob, nodes[sample_idx]);
                
                // step 6: check if sampled idx close enough to the goal
                if((nodes[sample_idx] - prob.q_goal).norm() < threshold){
                    nodes.push_back(prob.q_goal);
                    graph.connect(sample_idx, nodes.size() - 1, (nodes[sample_idx] - nodes[nodes.size() - 1]).norm());
                    heuristic.heuristic_values[nodes.size() - 1] = calculateHeuristic(prob, nodes[nodes.size() - 1]);
                    MyAStarAlgorithm aStar;
                    amp::ShortestPathProblem prob_astar = {std::make_shared<amp::Graph<double>>(graph), 0, static_cast<amp::Node>(nodes.size() - 1)};
                    AStar::GraphSearchResult result = aStar.search(prob_astar, heuristic);
                    distance = 0;
                    for(const auto& node : result.node_path) {
                        path.waypoints.push_back(nodes[node]);
                        if(node > 0){
                            distance += (nodes[prev_node_idx] - nodes[node]).norm();
                            prev_node_idx = node;
                        }
                    }
                    auto end_time = std::chrono::high_resolution_clock::now();
                    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
                    return path;
                }
            }
            else{
                nodes.pop_back();
            }

        }
        std::cout<<"Path not found"<<std::endl;
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        return path;
    }


    std::tuple<Path2D, double, double> MyGoalBiasRRT2D::plan(const Problem2D& prob, const double r, const int n, const double p, const double threshold, bool smooth){
       
       // start timer:
        auto start_time = std::chrono::high_resolution_clock::now();

        // constants:
        int counter = 0;
        int sample_idx;
        int nearest_idx;
        double distance;
        int prev_node_idx = 0;
        Path2D path;

        // step 1: construct cspace and create graph
        amp::Graph<double> graph;
        amp::LookupSearchHeuristic heuristic;
        std::vector<Eigen::Vector2d> nodes;
        nodes.push_back(prob.q_init);
        heuristic.heuristic_values[0] = calculateHeuristic(prob, nodes[0]);

        while(counter < n){
            counter++;
            // step 2: sample workspace with random sample
            sample_idx = generateRandomSample(prob, nodes, p, r);

            // step 3: find nearest node
            nearest_idx = findNearestNode(sample_idx, nodes, r, prob);

            // step 4: check if the path between the nearest node and the sample is valid
            if(!Helper().intersects(prob, nodes[nearest_idx], nodes[sample_idx])){
                // step 5: add edge to graph
                distance = (nodes[nearest_idx] - nodes[sample_idx]).norm();
                graph.connect(nearest_idx, sample_idx, distance);
                heuristic.heuristic_values[sample_idx] = calculateHeuristic(prob, nodes[sample_idx]);
                
                // step 6: check if sampled idx close enough to the goal
                if((nodes[sample_idx] - prob.q_goal).norm() < threshold){
                    nodes.push_back(prob.q_goal);
                    graph.connect(sample_idx, nodes.size() - 1, (nodes[sample_idx] - nodes[nodes.size() - 1]).norm());
                    heuristic.heuristic_values[nodes.size() - 1] = calculateHeuristic(prob, nodes[nodes.size() - 1]);
                    MyAStarAlgorithm aStar;
                    amp::ShortestPathProblem prob_astar = {std::make_shared<amp::Graph<double>>(graph), 0, static_cast<amp::Node>(nodes.size() - 1)};
                    AStar::GraphSearchResult result = aStar.search(prob_astar, heuristic);
                    distance = 0;
                    for(const auto& node : result.node_path) {
                        path.waypoints.push_back(nodes[node]);
                        if(node > 0){
                            distance += (nodes[prev_node_idx] - nodes[node]).norm();
                            prev_node_idx = node;
                        }
                    }
                    
                    auto end_time = std::chrono::high_resolution_clock::now();
                    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
                    return std::make_tuple(path, distance, duration.count());
                }
            }
            else{
                nodes.pop_back();
            }

        }
        std::cout<<"Path not found"<<std::endl;
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        return std::make_tuple(path, 0, duration.count());
    }

    int MyGoalBiasRRT2D::generateRandomSample(const Problem2D& prob, std::vector<Eigen::Vector2d>& nodes, const double p, const double r) {
        std::random_device rd;
        std::default_random_engine generator(rd());
        std::uniform_real_distribution<double> distribution_x(prob.x_min, prob.x_max);
        std::uniform_real_distribution<double> distribution_y(prob.y_min, prob.y_max);
        std::uniform_real_distribution<double> distribution_0_1(0, 1);
        std::uniform_real_distribution<double> distribution_angle(0, 2*M_PI);

        double x, y;
        if(distribution_0_1(generator) < p) {
            double theta = distribution_angle(generator);
            double dist = sqrt(distribution_0_1(generator)) * r;
            x = prob.q_goal[0] + dist * cos(theta);
            y = prob.q_goal[1] + dist * sin(theta);
        } else {
            x = distribution_x(generator);
            y = distribution_y(generator);
        }

        for(const auto& obstacle : prob.obstacles) {
            if(EnvironmentHelper().inCollision(Eigen::Vector2d(x, y), obstacle)) {
                return generateRandomSample(prob, nodes, p, r);
            }
        }

        nodes.push_back(Eigen::Vector2d(x, y));
        return nodes.size() - 1;
    }


    int MyGoalBiasRRT2D::findNearestNode(const int node_idx, std::vector<Eigen::Vector2d>& nodes, const double r, const Problem2D& prob){
        double min_distance = std::numeric_limits<double>::max();
        int nearest_node_idx = -1;

        for(int i = 0; i < nodes.size() - 1; i++) {
            if(i == node_idx) continue;
            double distance = (nodes[i] - nodes[node_idx]).norm();
            if(distance < min_distance) {
                min_distance = distance;
                nearest_node_idx = i;
            }
        }
        return nearest_node_idx;
    }


    double MyGoalBiasRRT2D::calculateHeuristic(const Problem2D& prob, const Eigen::Vector2d& node){
        double distance = (node - prob.q_goal).norm();
        return distance;
    }

    void MyGoalBiasRRT2D::smoothPath(int iterations, std::vector<Eigen::Vector2d>& nodes, const Problem2D& prob){
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