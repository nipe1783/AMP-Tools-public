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
        Node currNode, childNode;
        double currCost, childCost;
        std::pair<Node, double> currPair;
        std::unordered_map<Node, Node> parentMap;  // For tracking path
        
        auto compare = [](const std::pair<Node, double>& lhs, const std::pair<Node, double>& rhs) {
            return lhs.second > rhs.second;
        };
        
        std::priority_queue<std::pair<Node, double>, std::vector<std::pair<Node, double>>, decltype(compare)> priorityQueue(compare);
        std::unordered_map<Node, double> nodeCosts; 
        
        priorityQueue.push(std::make_pair(problem.init_node, heuristic(problem.init_node))); // Corrected this line
        nodeCosts[problem.init_node] = 0;

        while(!priorityQueue.empty()) {
            currPair = priorityQueue.top();
            priorityQueue.pop();
            currNode = currPair.first;
            currCost = nodeCosts[currNode];  // Using actual cost from map

            if(currNode == problem.goal_node) {
                result.success = true;
                // Construct path
                while(currNode != problem.init_node) {
                    result.node_path.push_back(currNode);
                    currNode = parentMap[currNode];
                }
                result.node_path.push_back(problem.init_node);
                std::reverse(result.node_path.begin(), result.node_path.end());
                result.path_cost = currCost;
                return result;
            }

            for(int i = 0; i < problem.graph->children(currNode).size(); i++) {
                childNode = problem.graph->children(currNode)[i];
                childCost = currCost + problem.graph->outgoingEdges(currNode)[i];
                if(nodeCosts.find(childNode) == nodeCosts.end() || childCost < nodeCosts[childNode]) {
                    nodeCosts[childNode] = childCost;
                    priorityQueue.push(std::make_pair(childNode, nodeCosts[childNode] + heuristic(childNode)));
                    parentMap[childNode] = currNode;
                }
            }
        }
        result.success = false;
        return result;
    }
}


