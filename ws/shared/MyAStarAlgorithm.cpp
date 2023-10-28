#include "MyAstarAlgorithm.h"
#include "AMPCore.h"
#include "hw/HW6.h"
#include <unordered_map>
#include <queue>
#include <set>
#include <vector>
#include <utility>


namespace amp {
    AStar::GraphSearchResult MyAStarAlgorithm::search(const ShortestPathProblem& problem, const SearchHeuristic& heuristic) {
        GraphSearchResult result;
        Node currNode, neighborNode;
        double currCost, neighborCost;
        std::pair<Node, double> currPair;
        std::unordered_map<Node, Node> parentMap;
        
        auto compare = [](const std::pair<Node, double>& lhs, const std::pair<Node, double>& rhs) {
            return lhs.second > rhs.second;
        };
        
        std::priority_queue<std::pair<Node, double>, std::vector<std::pair<Node, double>>, decltype(compare)> priorityQueue(compare);
        std::unordered_map<Node, double> nodeCosts; 
        
        priorityQueue.push(std::make_pair(problem.init_node, heuristic(problem.init_node)));
        nodeCosts[problem.init_node] = 0;

        while(!priorityQueue.empty()) {
            currPair = priorityQueue.top();
            priorityQueue.pop();
            currNode = currPair.first;
            currCost = nodeCosts[currNode];

            if(currNode == problem.goal_node) {
                result.success = true;
                while(currNode != problem.init_node) {
                    result.node_path.push_back(currNode);
                    currNode = parentMap[currNode];
                }
                result.node_path.push_back(problem.init_node);
                std::reverse(result.node_path.begin(), result.node_path.end());
                result.path_cost = currCost;
                return result;
            }

            // Process children
            auto childNodes = problem.graph->children(currNode);
            auto childEdges = problem.graph->outgoingEdges(currNode);
            for(int i = 0; i < childNodes.size(); i++) {
                neighborNode = childNodes[i];
                neighborCost = currCost + childEdges[i];
                if(nodeCosts.find(neighborNode) == nodeCosts.end() || neighborCost < nodeCosts[neighborNode]) {
                    nodeCosts[neighborNode] = neighborCost;
                    priorityQueue.push(std::make_pair(neighborNode, nodeCosts[neighborNode] + heuristic(neighborNode)));
                    parentMap[neighborNode] = currNode;
                }
            }

            // Process parents
            auto parentNodes = problem.graph->parents(currNode);
            auto parentEdges = problem.graph->incomingEdges(currNode);
            for(int i = 0; i < parentNodes.size(); i++) {
                neighborNode = parentNodes[i];
                neighborCost = currCost + parentEdges[i];
                if(nodeCosts.find(neighborNode) == nodeCosts.end() || neighborCost < nodeCosts[neighborNode]) {
                    nodeCosts[neighborNode] = neighborCost;
                    priorityQueue.push(std::make_pair(neighborNode, nodeCosts[neighborNode] + heuristic(neighborNode)));
                    parentMap[neighborNode] = currNode;
                }
            }
        }
        result.success = false;
        return result;
    }
}


