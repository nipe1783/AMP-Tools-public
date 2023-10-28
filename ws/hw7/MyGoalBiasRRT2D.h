#pragma once
#include "AMPCore.h"
#include "hw/HW7.h"
#include "hw/HW6.h"

namespace amp {
    class MyGoalBiasRRT2D : public amp::GoalBiasRRT2D {
        public:
            /**
             * @brief Plan a path from the start to the goal.
             * 
             * @param prob Problem to plan
             * @return Path2D Path from start to goal
             **/
            Path2D plan(const Problem2D& prob) override;
            std::tuple<Path2D, double, double> plan(const Problem2D& prob, const double r, const int n, bool smooth = false);

            /**
             * @brief Creates nodes samples in the cspace.
             * 
             * @param cpsace cspace to sample
             * @param num_samples number of samples to create
             * @param graph graph to add samples to
             **/
            void sampleWorkSpace(const Problem2D& prob, const int num_samples, std::vector<Eigen::Vector2d>& nodes);


            /**
             * @brief connects current node to all nodes within radius r.
             * 
             * @param node current node
             * @param nodes all nodes
             * @param r radius of max distance
             * @param graph graph to add edges to
             **/
            void linkNearestNodes(const int node_idx, std::vector<Eigen::Vector2d>& nodes, const double r, amp::Graph<double>& graph, amp::LookupSearchHeuristic& heuristic, const Problem2D& prob);

            double calculateHeuristic(const Problem2D& prob, const Eigen::Vector2d& node);

            void smoothPath(int iterations, std::vector<Eigen::Vector2d>& nodes, const Problem2D& prob);
    };
}
