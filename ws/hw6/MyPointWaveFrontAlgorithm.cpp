#include "MyPointWaveFrontAlgorithm.h"
#include "AMPCore.h"
#include "hw/HW4.h"
#include "MyConfigurationSpace.h"
#include "Helper.h"

namespace amp{

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
        std::pair<std::size_t, std::size_t> startCell = grid_cspace.getCellFromPoint(q_init[0], q_init[1]);
        std::pair<std::size_t, std::size_t> goalCell = grid_cspace.getCellFromPoint(q_goal[0], q_goal[1]);
        std::pair<std::size_t, std::size_t> currentCell = goalCell;
        amp::Graph<int> graph;
        amp::Node node = 2;
        for(int i = 0; i < 10; i++){

        }
        return path;
    }

    amp::Graph<int> MyPointWaveFrontAlgorithm::constructGraph(const amp::GridCSpace2D& grid_cspace, const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal){
        amp::Graph<int> graph;
        return graph;
    }
    
}