#include "AMPCore.h"
#include "hw/HW7.h"
#include "MyPRM2D.h"
#include "EnvironmentHelper.h"
#include <random>

namespace amp{
    
    Path2D MyPRM2D::plan(const Problem2D& prob){

        // constants:
        const double r = 1;
        const int n = 100;

        Path2D path;

        // step 1: construct cspace and create graph
        // std::unique_ptr<amp::GridCSpace2D> grid_cspace  = EnvironmentHelper().constructCSpacePRB(prob, 0.25, 0.15);
        amp::Graph<double, Eigen::Vector2d> graph;
        std::vector<Eigen::Vector2d> nodes;

        // step 2: sample cspace
        sampleWorkSpace(prob, n, nodes);

        // step 3: connect nodes and remove invalid nodes
        connectNodes(prob, nodes, graph, r);

        // step 4: find shortest path

        path.waypoints.push_back(prob.q_init);
        path.waypoints.push_back(prob.q_goal);
        return path;
    }

    void MyPRM2D::sampleWorkSpace(const Problem2D& prob, int num_samples, std::vector<Eigen::Vector2d>& nodes){
        double xmin = prob.x_min;
        double xmax = prob.x_max;
        double ymin = prob.y_min;
        double ymax = prob.y_max;
        double x;
        double y;
        std::random_device rd;
        std::default_random_engine generator(rd());
        std::uniform_real_distribution<double> distribution_x(xmin, xmax);
        std::uniform_real_distribution<double> distribution_y(ymin, ymax);
        for (int i = 0; i < num_samples; i++){
            x = distribution_x(generator);
            y = distribution_y(generator);
            for(int j = 0; j < prob.obstacles.size(); j++){
                if(!EnvironmentHelper().inCollision(Eigen::Vector2d(x, y), prob.obstacles[j])){
                    nodes.push_back(Eigen::Vector2d(x, y));
                }
            }
        }
    }

    void MyPRM2D::connectNodes(const Problem2D& prob, std::vector<Eigen::Vector2d>& nodes, amp::Graph<double, Eigen::Vector2d>& graph, const double r){
        for(int i = 0; i < nodes.size(); i++){
            linkNearestNodes(nodes[i], nodes, r, graph);
        }
    }

    void MyPRM2D::linkNearestNodes( const Eigen::Vector2d& node, std::vector<Eigen::Vector2d>& nodes, const double r, amp::Graph<double, Eigen::Vector2d>& graph){
        double distance;
        for(int i = 0; i < nodes.size(); i++){
            distance = (nodes[i] - node).norm();
            if(distance < r && distance > 0){
                graph.connect(node, nodes[i], distance);
            }
        }
    }


}