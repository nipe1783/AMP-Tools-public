#include "AMPCore.h"
#include "hw/HW2.h"
#include "hw/HW5.h"
#include "hw/HW6.h"
#include "hw/HW7.h"
#include "hw/HW8.h"
#include "Helper.h"
#include <iostream>
#include <numeric>
#include <vector>
#include "MyCentralizedMultiAgentRRT.h"

using namespace amp;

int main(int argc, char** argv) {
    // MyCentralizedMultiAgentRRT rrt;

    // // Vectors to store all times and nodes for each agent configuration
    // std::vector<double> times_1_2m, times_1_3m, times_1_4m, times_1_5m, times_1_6m;
    // std::vector<double> nodes_1_2m, nodes_1_3m, nodes_1_4m, nodes_1_5m, nodes_1_6m;

    // MultiAgentProblem2D prob_1_2m = HW8::getWorkspace1(2);
    // MultiAgentPath2D path_1_2m = rrt.plan(prob_1_2m);
    // Visualizer::makeFigure(prob_1_2m, path_1_2m);

    // MultiAgentProblem2D prob_1_3m = HW8::getWorkspace1(3);
    // MultiAgentPath2D path_1_3m = rrt.plan(prob_1_3m);
    // Visualizer::makeFigure(prob_1_3m, path_1_3m);

    // MultiAgentProblem2D prob_1_4m = HW8::getWorkspace1(4);
    // MultiAgentPath2D path_1_4m = rrt.plan(prob_1_4m);
    // Visualizer::makeFigure(prob_1_4m, path_1_4m);

    // MultiAgentProblem2D prob_1_5m = HW8::getWorkspace1(5);
    // MultiAgentPath2D path_1_5m = rrt.plan(prob_1_5m);
    // Visualizer::makeFigure(prob_1_5m, path_1_5m);

    // MultiAgentProblem2D prob_1_6m = HW8::getWorkspace1(6);
    // prob_1_6m.agent_properties[5].q_init = Eigen::Vector2d(12.0, 14.0);
    // MultiAgentPath2D path_1_6m = rrt.plan(prob_1_6m);
    // Visualizer::makeFigure(prob_1_6m, path_1_6m);

    // Visualizer::showFigures();

    // // Loop for each agent configuration
    // for (int i = 0; i < 100; i++) {
    //     // Configuration for 2 agents
    //     MultiAgentProblem2D prob_1_2m = HW8::getWorkspace1(2);
    //     auto answer_1_2m = rrt.plan(prob_1_2m, true);
    //     times_1_2m.push_back(std::get<2>(answer_1_2m));
    //     nodes_1_2m.push_back(std::get<1>(answer_1_2m));

    //     // Configuration for 3 agents
    //     MultiAgentProblem2D prob_1_3m = HW8::getWorkspace1(3);
    //     auto answer_1_3m = rrt.plan(prob_1_3m, true);
    //     times_1_3m.push_back(std::get<2>(answer_1_3m));
    //     nodes_1_3m.push_back(std::get<1>(answer_1_3m));

    //     // Configuration for 4 agents
    //     MultiAgentProblem2D prob_1_4m = HW8::getWorkspace1(4);
    //     auto answer_1_4m = rrt.plan(prob_1_4m, true);
    //     times_1_4m.push_back(std::get<2>(answer_1_4m));
    //     nodes_1_4m.push_back(std::get<1>(answer_1_4m));

    //     // Configuration for 5 agents
    //     MultiAgentProblem2D prob_1_5m = HW8::getWorkspace1(5);
    //     auto answer_1_5m = rrt.plan(prob_1_5m, true);
    //     times_1_5m.push_back(std::get<2>(answer_1_5m));
    //     nodes_1_5m.push_back(std::get<1>(answer_1_5m));

    //     // Configuration for 6 agents
    //     MultiAgentProblem2D prob_1_6m = HW8::getWorkspace1(6);
    //     prob_1_6m.agent_properties[5].q_init = Eigen::Vector2d(12.0, 14.0);
    //     auto answer_1_6m = rrt.plan(prob_1_6m, true);
    //     times_1_6m.push_back(std::get<2>(answer_1_6m));
    //     nodes_1_6m.push_back(std::get<1>(answer_1_6m));
    // }

    // // Function to sum elements of a vector
    // auto sum_vector = [](const std::vector<double>& v) {
    //     return std::accumulate(v.begin(), v.end(), 0.0);
    // };

    // std::string title = "Time vs. Number of Agents";
    // std::string xlabel = "Number of Agents";
    // std::string ylabel = "Time (ms)";
    // std::vector<std::string> labels = {"2 agents", "3 agents", "4 agents", "5 agents", "6 agents"};
    // std::vector<std::vector<double>> dataTime = {times_1_2m, times_1_3m, times_1_4m, times_1_5m, times_1_6m};
    // std::vector<std::vector<double>> dataNodes = {nodes_1_2m, nodes_1_3m, nodes_1_4m, nodes_1_5m, nodes_1_6m};
    // std::list<std::vector<double>> dataTimeList(dataTime.begin(), dataTime.end());
    // std::list<std::vector<double>> dataNodesList(dataNodes.begin(), dataNodes.end());
    // Visualizer::makeBoxPlot(dataTimeList, labels, title, xlabel, ylabel);
    // Visualizer::makeBoxPlot(dataNodesList, labels, title, xlabel, "Size of Tree");
    // Visualizer::showFigures();

    return 0;
}