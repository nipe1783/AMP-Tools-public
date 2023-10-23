#include "MyManipulatorWaveFrontAlgorithm.h"
#include "AMPCore.h"
#include "hw/HW4.h"
#include "MyConfigurationSpace.h"
#include "Helper.h"
#include "EnvironmentHelper.h"
#include "Tree.h"
#include "TreeNode.h"
#include <queue>
#include <unordered_set>



namespace amp{

    constexpr std::array<std::pair<int, int>, 4> directions = {{{1, 0}, {0, -1}, {-1, 0}, {0, 1}}};

    struct pair_hash {
        template <class T1, class T2>
        std::size_t operator () (const std::pair<T1, T2>& p) const {
            return std::hash<T1>{}(p.first) ^ (std::hash<T2>{}(p.second) << 1);
        }
    };

    bool compareX(const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
        return a.x() < b.x();
    }
    bool compareY(const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
        return a.y() < b.y();
    }

    double normalizeAngle(double angle) {
        return fmod((fmod(angle, 2.0 * M_PI) + 2.0 * M_PI), 2.0 * M_PI);
    }

    Eigen::Vector2d normalizeAngles(const Eigen::Vector2d& angles) {
        Eigen::Vector2d normalized;
        normalized.x() = normalizeAngle(angles.x());
        normalized.y() = normalizeAngle(angles.y());
        return normalized;
    }



    amp::Path2D MyManipulatorWaveFrontAlgorithm::planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace) {
        amp::Path2D path;
        Eigen::Vector2d step;
        double x0;
        double x1;
        Eigen::Vector2d q_init_new;
        Eigen::Vector2d q_goal_new;
        q_init_new = normalizeAngles(q_init);
        q_goal_new = normalizeAngles(q_goal);
        path.waypoints.push_back(q_init_new);
        std::vector<std::pair<std::size_t, std::size_t>> gridPath = constructTree(q_init_new, q_goal_new, grid_cspace);
        for(auto& cell : gridPath){
            x0 = double(cell.first + 1) / double(grid_cspace.size().first) * grid_cspace.x0Bounds().second;
            x1 = double(cell.second + 1) / double(grid_cspace.size().second) * grid_cspace.x1Bounds().second;
            path.waypoints.push_back(Eigen::Vector2d(x0, x1));
        }
        path.waypoints.push_back(q_goal_new);
        auto minElementItX = std::min_element(path.waypoints.begin(), path.waypoints.end(), compareX);
        unwrapPath(path, Eigen::Vector2d(0,0), Eigen::Vector2d(2 * M_PI, 2 * M_PI));
        return path;
    }


    std::vector<std::pair<std::size_t, std::size_t>> MyManipulatorWaveFrontAlgorithm::constructTree(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace) {
        
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

                for (const auto& dir : directions) {
                    int newX = (currentCell.first + dir.first + x0_grid) % x0_grid;
                    int newY = (currentCell.second + dir.second + x1_grid) % x1_grid;
                    if(newX < 0){
                        newX = x0_grid - 1;
                    }
                    if(newX >= x0_grid){
                        newX = 0;
                    }
                    if(newY < 0){
                        newY = x1_grid - 1;
                    }
                    if(newY >= x1_grid){
                        newY = 0;
                    }
                    if(newX == goalCell.first && newY == goalCell.second){
                        visited[newX][newY] = true;
                        cellQueue.push({newX, newY});
                        parents[{newX, newY}] = currentCell;
                    }
                    else if (newX >= 0 && newX < x0_grid && newY >= 0 && newY < x1_grid && !visited[newX][newY] && !grid_cspace(newX, newY)) {
                        visited[newX][newY] = true;
                        if(isCellAndNeighborsFree(newX, newY, grid_cspace, x0_grid, x1_grid, 1)){
                            cellQueue.push({newX, newY});
                            parents[{newX, newY}] = currentCell;
                        }
                    }
                }
            }
            std::cout << "No path found!" << std::endl;
            return gridPath;
    }

    bool MyManipulatorWaveFrontAlgorithm::isCellAndNeighborsFree(int x, int y, const amp::GridCSpace2D& grid_cspace, int x0_grid, int x1_grid, int n) {
        if (n < 0) return true; // Base case for recursion
        if (grid_cspace(x, y)) return false; // Check the current cell

        for (const auto& dir : directions) {
            int newX = x + dir.first;
            int newY = y + dir.second;
            
            // Handle wrap-around logic
            if(newX < 0) newX = x0_grid - 1;
            if(newX >= x0_grid) newX = 0;
            if(newY < 0) newY = x1_grid - 1;
            if(newY >= x1_grid) newY = 0;
            // Check this neighbor and its neighbors up to n-levels deep
            if (!isCellAndNeighborsFree(newX, newY, grid_cspace, x0_grid, x1_grid, n-1)) {
                return false;
            }
        }
        return true;
    }


}