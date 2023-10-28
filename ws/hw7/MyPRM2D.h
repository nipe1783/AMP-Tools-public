#pragma once
#include "AMPCore.h"
#include "hw/HW7.h"

namespace amp {
    class MyPRM2D : public amp::PRM2D {
        public:
            /**
             * @brief Plan a path from the start to the goal.
             * 
             * @param prob Problem to plan
             * @return Path2D Path from start to goal
             **/
            Path2D plan(const Problem2D& prob) override;

            /**
             * @brief Creates nodes samples in the cspace.
             * 
             * @param cpsace cspace to sample
             * @param num_samples number of samples to create
             * @param graph graph to add samples to
             **/
            void sampleWorkSpace(const Problem2D& prob, const int num_samples, std::vector<Eigen::Vector2d>& nodes);


            /**
             * @brief Connects nodes in the cspace and removes invalid nodes.
             * 
             * @param cpsace cspace to sample
             * @param nodes nodes to connect
             * @param graph graph to add samples to
             * @param r radius of nodes to connect
             **/
            void connectNodes(const Problem2D& prob, std::vector<Eigen::Vector2d>& nodes, amp::Graph<double, Eigen::Vector2d>& graph, const double r);

            /**
             * @brief connects current node to all nodes within radius r.
             * 
             * @param node current node
             * @param nodes all nodes
             * @param r radius of max distance
             * @param graph graph to add edges to
             **/
            void MyPRM2D::linkNearestNodes( const Eigen::Vector2d& node, std::vector<Eigen::Vector2d>& nodes, const double r, amp::Graph<double, Eigen::Vector2d>& graph);
    };
}
