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
                Obstacle2D expandedObstacle = Helper().expandObstacle(obstacle, problem.agent_properties[i].radius + .05);
                if(EnvironmentHelper().inCollision(Eigen::Vector2d(x, y), expandedObstacle)) {
                    return true;
                }
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
            Obstacle2D expandedObstacle = Helper().expandObstacle(obstacle, problem.agent_properties[agentId].radius + .05);
            if(EnvironmentHelper().inCollision(Eigen::Vector2d(x, y), expandedObstacle)) {
                return true;
            }
        }
        
        // check if any robot collides with another robot
        Eigen::Vector2d pos1(x, y);
        for(int i = 0; i < agentLocations.size(); i++){
            if(i != agentId){
                Eigen::Vector2d pos2(agentLocations[i][0], agentLocations[i][1]);
                std::cout<<"robot 1 pos: "<<pos1[0]<< " " << pos1[1]<<std::endl;
                std::cout<<"robot 2 pos: "<<pos2<<std::endl;
                std::cout<<"distance: "<<Helper().distance(pos1, pos2)<<std::endl;
                if(Helper().distance(pos1, pos2) <= problem.agent_properties[agentId].radius + problem.agent_properties[i].radius + padding){
                    std::cout<<"collision detected"<<std::endl;
                    return true;
                }
            }
        }
        return false;
    }
}