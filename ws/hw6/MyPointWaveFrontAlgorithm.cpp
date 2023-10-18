#include "MyPointWaveFrontAlgorithm.h"
#include "AMPCore.h"
#include "hw/HW4.h"
#include "MyConfigurationSpace.h"
#include "Helper.h"
#include "Tree.h"
#include "TreeNode.h"
#include <queue>
#include <unordered_set>


namespace amp{

    struct pair_hash {
        template <class T1, class T2>
        std::size_t operator () (const std::pair<T1, T2>& p) const {
            auto h1 = std::hash<T1>{}(p.first);
            auto h2 = std::hash<T2>{}(p.second);
            return h1 ^ h2;
        }
    };


    MyPointWaveFrontAlgorithm::MyPointWaveFrontAlgorithm(){};

    MyPointWaveFrontAlgorithm::~MyPointWaveFrontAlgorithm(){};

    std::unique_ptr<amp::GridCSpace2D> MyPointWaveFrontAlgorithm::constructDiscretizedWorkspace(const amp::Environment2D& environment){
        
        double grid_size = 2;
        double density_x0 = (environment.x_max - environment.x_min) / grid_size;
        double density_x1 = (environment.y_max - environment.y_min) / grid_size;

        std::unique_ptr<amp::GridCSpace2D> cSpace = std::make_unique<amp::MyConfigurationSpace>(density_x0, density_x1, environment.x_min, environment.x_max, environment.y_min, environment.y_max);
        double x0;
        double x1;
        amp::Polygon robotPos;
        Eigen::Vector2d bottomLeft;
        Eigen::Vector2d bottomRight;
        Eigen::Vector2d topLeft;
        Eigen::Vector2d topRight;
        for (int i = 0; i < density_x0; i++) {
            for (int j = 0; j < density_x1; j++) {
                bottomLeft = {environment.x_min + i * grid_size, environment.y_min + j * grid_size};
                bottomRight = {environment.x_min + (i + 1) * grid_size, environment.y_min + j * grid_size};
                topRight = {environment.x_min + (i + 1) * grid_size, environment.y_min + (j + 1) * grid_size};
                topLeft = {environment.x_min + i * grid_size, environment.y_min + (j + 1) * grid_size};
                // std::cout<<"bottomLeft: "<< bottomLeft[0] << ","<< bottomLeft[1] << "bottomRight: " << bottomRight[0] << ","<< bottomRight[1] << "topLeft: " << topLeft[0] << ","<< topLeft[1] << "topRight: " << topRight[0] << ","<< topRight[1] << std::endl; 
                robotPos.verticesCCW().clear();
                robotPos.verticesCCW().push_back(bottomLeft);
                robotPos.verticesCCW().push_back(bottomRight);
                robotPos.verticesCCW().push_back(topRight);
                robotPos.verticesCCW().push_back(topLeft);
                
                for (auto& obstacle : environment.obstacles) {
                    if(Helper().polygonIntersect(robotPos, obstacle)){
                        cSpace->operator()(i, j) = true;
                    }
                }

            }
        }
        return cSpace;
    }

    amp::Path2D MyPointWaveFrontAlgorithm::planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace){
        amp::Path2D path;
        constructTree(q_init, q_goal, grid_cspace);
        return path;
    }

    void MyPointWaveFrontAlgorithm::constructTree(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace){
        int x0_grid = grid_cspace.size().first;
        int x1_grid = grid_cspace.size().second;
        std::pair<std::size_t, std::size_t> startCell = grid_cspace.getCellFromPoint(q_init[0], q_init[1]);
        std::pair<std::size_t, std::size_t> goalCell = grid_cspace.getCellFromPoint(q_goal[0], q_goal[1]);
        std::pair<std::size_t, std::size_t> currentCell = goalCell;
        std::pair<std::size_t, std::size_t> childCell;
        std::queue<TreeNode> treeQueue;
        std::unordered_set<std::pair<std::size_t, std::size_t>, pair_hash> visited;
        TreeNode currentNode;
        currentNode.cell = currentCell;
        Tree tree;
        tree.root = &currentNode;
        treeQueue.push(currentNode);
        while(!treeQueue.empty()){
            
            currentNode = treeQueue.front();
            currentCell = currentNode.cell;
            treeQueue.pop();
            // check currentCell reachable neighbors.
            // check right:
            childCell = std::make_pair(currentCell.first + 1, currentCell.second);
            if(childCell.first < x0_grid && !visited.count(childCell)){
                if(!grid_cspace(childCell.first, childCell.second)){
                    TreeNode childNode = TreeNode();
                    childNode.cell = childCell;
                    childNode.parent = &currentNode;
                    currentNode.children.push_back(&childNode);
                    treeQueue.push(childNode);
                    visited.insert(childNode.cell);
                }
            }
            // check down:
            childCell = std::make_pair(currentCell.first, currentCell.second - 1);
            if(childCell.second >= 0 && !visited.count(childCell)){
                if(!grid_cspace(childCell.first, childCell.second)){
                    TreeNode childNode = TreeNode();
                    childNode.cell = childCell;
                    childNode.parent = &currentNode;
                    currentNode.children.push_back(&childNode);
                    treeQueue.push(childNode);
                    visited.insert(childNode.cell);
                }
            }
            // check left:
            childCell = std::make_pair(currentCell.first - 1, currentCell.second);
            if(childCell.first >= 0 && !visited.count(childCell)){
                if(!grid_cspace(childCell.first, childCell.second)){
                    TreeNode childNode = TreeNode();
                    childNode.cell = childCell;
                    childNode.parent = &currentNode;
                    currentNode.children.push_back(&childNode);
                    treeQueue.push(childNode);
                    visited.insert(childNode.cell);
                }
            }
            // check up:
            childCell = std::make_pair(currentCell.first, currentCell.second + 1);
            if(childCell.second < x1_grid && !visited.count(childCell)){
                if(!grid_cspace(childCell.first, childCell.second)){
                    TreeNode childNode = TreeNode();
                    childNode.cell = childCell;
                    childNode.parent = &currentNode;
                    currentNode.children.push_back(&childNode);
                    treeQueue.push(childNode);
                    visited.insert(childNode.cell);
                }
            }

        }

    }
    
}