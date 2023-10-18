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
        
        double grid_size = 0.25;
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
        Eigen::Vector2d step;
        double x0;
        double x1;
        double resolution_x0 = 0.25;
        double resolution_x1 = 0.25;
        path.waypoints.push_back(q_init);
        std::vector<std::pair<std::size_t, std::size_t>> gridPath = constructTree(q_init, q_goal, grid_cspace);
        for(auto& cell : gridPath){
            x0 = (cell.first + 1) * resolution_x0 + grid_cspace.x0Bounds().first;
            x1 = (cell.second + 1) * resolution_x1 + grid_cspace.x1Bounds().first;
            path.waypoints.push_back(Eigen::Vector2d(x0, x1));
        }
        path.waypoints.push_back(q_goal);
        return path;
    }

    std::vector<std::pair<std::size_t, std::size_t>> MyPointWaveFrontAlgorithm::constructTree(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace) {
        
        int x0_grid = grid_cspace.size().first;
        int x1_grid = grid_cspace.size().second;

        std::pair<std::size_t, std::size_t> startCell = grid_cspace.getCellFromPoint(q_init[0], q_init[1]);
        std::pair<std::size_t, std::size_t> goalCell = grid_cspace.getCellFromPoint(q_goal[0], q_goal[1]);
        std::pair<std::size_t, std::size_t> currentCell = startCell;

        std::vector<std::pair<std::size_t, std::size_t>> gridPath;
        std::queue<std::pair<std::size_t, std::size_t>> cellQueue;  // Instead of TreeNode, just use the cell itself

        std::vector<std::vector<bool>> visited(x0_grid, std::vector<bool>(x1_grid, false));

        cellQueue.push(startCell);
        visited[startCell.first][startCell.second] = true;

        std::map<std::pair<std::size_t, std::size_t>, std::pair<std::size_t, std::size_t>> parents;

            while (!cellQueue.empty()) {
                currentCell = cellQueue.front();
                cellQueue.pop();

                if (currentCell == goalCell) {
                    while (currentCell != startCell) {
                        gridPath.push_back(currentCell);
                        currentCell = parents[currentCell];
                    }
                    gridPath.push_back(startCell);
                    std::reverse(gridPath.begin(), gridPath.end());
                    return gridPath;
                }

                std::array<std::pair<int, int>, 4> directions = {{{1, 0}, {0, -1}, {-1, 0}, {0, 1}}};

                for (const auto& dir : directions) {
                    int newX = currentCell.first + dir.first;
                    int newY = currentCell.second + dir.second;
                    
                    if (newX >= 0 && newX < x0_grid && newY >= 0 && newY < x1_grid && !visited[newX][newY] && !grid_cspace(newX, newY)) {
                        visited[newX][newY] = true;
                        cellQueue.push({newX, newY});
                        parents[{newX, newY}] = currentCell;
                    }
                }
            }
            std::cout << "No path found!" << std::endl;
            return gridPath;
    }

    
}