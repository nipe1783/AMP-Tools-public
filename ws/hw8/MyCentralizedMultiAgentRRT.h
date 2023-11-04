#pragma once
#include "AMPCore.h"
#include "hw/HW7.h"
#include "hw/HW6.h"
#include "hw/HW8.h"
#include "NDConfigurationSpace.h"

namespace amp {
    class MyCentralizedMultiAgentRRT : public amp::CentralizedMultiAgentRRT {
        public:
        
        amp::MultiAgentPath2D plan(const amp::MultiAgentProblem2D& problem) override;
        void generateRandomSample(const NDConfigurationSpace& cSpace, const MultiAgentProblem2D& multiAgentprob, std::vector<Eigen::VectorXd>& nodes, const double p, const double r);

        /**
         * @brief Given a set of nodes where the last in the vector is the new node, find the nearest node in the graph and an edge between them.
         * 
         * @param problem 2D multi robot problem.
         * @param nodes Vector of nodes.
         * @param graph Graph of nodes.
         * @return void. updates graph, heurisitc (if there is a valid node to connect) and nodes (if there is no connection available). 
         **/
        void findNearestNode(const NDConfigurationSpace& cSpace, const MultiAgentProblem2D& problem, std::vector<Eigen::VectorXd>& nodes, amp::Graph<double>& graph, amp::LookupSearchHeuristic& heuristic, double& stepSize,  const int& numberOfChecks);
    };
}
