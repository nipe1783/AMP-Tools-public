#include "NDConfigurationSpace.h"
#include "Helper.h"
#include "EnvironmentHelper.h"

namespace amp{

    NDConfigurationSpace::NDConfigurationSpace(const Eigen::VectorXd& lower_bounds, const Eigen::VectorXd& upper_bounds)
        : ConfigurationSpace(lower_bounds, upper_bounds) {}

    void NDConfigurationSpace::construct(const amp::MultiAgentProblem2D prob){
        std::cout<<"Constructing CSpace"<<std::endl;
    }

    bool NDConfigurationSpace::inCollision(const Eigen::VectorXd& cspace_state) const {
        std::cout<<"HELLO WORLD"<<std::endl;
        return false;
    }

    bool NDConfigurationSpace::inCollision(const Eigen::VectorXd& cspace_state, const amp::MultiAgentProblem2D& problem, const double& padding) const {
        // check if any robot collides with obstacle
        int dofRobot = 2;
        for(int i = 0; i < problem.numAgents(); i++){
            double x = cspace_state[i*dofRobot];
            double y = cspace_state[i*dofRobot+1];
            for(amp::Obstacle2D obstacle : problem.obstacles){
                double radius = problem.agent_properties[i].radius;
                Eigen::Vector2d obstacleRefPoint = EnvironmentHelper().obstacleRefPoint(obstacle, Eigen::Vector2d(x, y));
                if(Helper().distance(obstacleRefPoint, Eigen::Vector2d(x, y)) <= radius + padding){
                    return true;
                }
                // Obstacle2D expandedObstacle = Helper().expandObstacle(obstacle, problem.agent_properties[i].radius + .05);
                // if(EnvironmentHelper().inCollision(Eigen::Vector2d(x, y), expandedObstacle)) {
                //     return true;
                // }
            }
        }
        // check if any robot collides with another robot
        for(int i = 0; i < problem.numAgents(); i++){
            double x1 = cspace_state[i*dofRobot];
            double y1 = cspace_state[i*dofRobot+1];
            Eigen::Vector2d pos1(x1, y1);
            for(int j = 0; j < problem.numAgents(); j++){
                if(i != j){
                    Eigen::Vector2d pos2(cspace_state[j*dofRobot], cspace_state[j*dofRobot+1]);
                    if(Helper().distance(pos1, pos2) <= problem.agent_properties[i/dofRobot].radius + problem.agent_properties[j/dofRobot].radius + padding){
                        return true;
                    }
                }
            }
        }
        return false;
    }

    bool NDConfigurationSpace::inCollision(const Eigen::VectorXd& cspace_state, const amp::MultiAgentProblem2D& problem, const double& padding, const  std::vector<Eigen::Vector2d>& agentLocations, const int& agentId) const {
        // check if any robot collides with obstacle
        int dofRobot = 2;
        double x = cspace_state[0];
        double y = cspace_state[1];
        for(amp::Obstacle2D obstacle : problem.obstacles){
            double radius = problem.agent_properties[agentId].radius;
            Eigen::Vector2d obstacleRefPoint = EnvironmentHelper().obstacleRefPoint(obstacle, Eigen::Vector2d(x, y));
            if(Helper().distance(obstacleRefPoint, Eigen::Vector2d(x, y)) <= radius + padding){
                return true;
            }
            // double radius = problem.agent_properties[agentId].radius;
            // Obstacle2D expandedObstacle = Helper().expandObstacle(obstacle, .2 + problem.agent_properties[agentId].radius);
            // if(EnvironmentHelper().inCollision(Eigen::Vector2d(x, y), expandedObstacle)) {
            //     return true;
            // }
        }
        
        // check if any robot collides with another robot
        Eigen::Vector2d pos1(x, y);
        for(int i = 0; i < agentLocations.size(); i++){
            if(i != agentId){
                Eigen::Vector2d pos2(agentLocations[i][0], agentLocations[i][1]);
                if(Helper().distance(pos1, pos2) <= problem.agent_properties[agentId].radius + problem.agent_properties[i].radius + padding){
                    return true;
                }
            }
        }
        return false;
    }
}