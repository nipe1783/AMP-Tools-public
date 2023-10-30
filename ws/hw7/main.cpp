#include "AMPCore.h"
#include "hw/HW2.h"
#include "hw/HW5.h"
#include "hw/HW6.h"
#include "hw/HW7.h"
#include "MyPRM2D.h"
#include "MyGoalBiasRRT2D.h"
#include "Helper.h"

using namespace amp;


    // struct PlanResult {
    //     std::vector<double> times;
    //     std::vector<double> pathLengths;
    //     double successfulPlans;
    // };

    // // Function to compute times based on given parameters
    // PlanResult computeStatsPRM(MyPRM2D& prm, const Problem2D& problem, double r, int n, int iterations = 100) {
    //     PlanResult result;
    //     double successfulPlans = 0;

    //     for(int i = 0; i < iterations; i++) {
    //         auto answer = prm.plan(problem, r, n, true);
    //         result.times.push_back(std::get<2>(answer));

    //         if(std::get<0>(answer).waypoints.size() == 0) {
    //             std::cout << "No path found" << std::endl;
    //         } else {
    //             result.pathLengths.push_back(std::get<1>(answer));
    //             successfulPlans++;
    //         }
    //     }

    //     result.successfulPlans = successfulPlans;
    //     return result;
    // }

    // PlanResult computeStatsRRT(MyGoalBiasRRT2D& rrt, const Problem2D& problem, double r, int n, double p, double threshold, int iterations = 100) {
    //     PlanResult result;
    //     double successfulPlans = 0;

    //     for(int i = 0; i < iterations; i++) {
    //         auto answer = rrt.plan(problem, r, n, p, threshold, false);
    //         result.times.push_back(std::get<2>(answer));

    //         if(std::get<0>(answer).waypoints.size() == 0) {
    //             std::cout << "No path found" << std::endl;
    //         } else {
    //             result.pathLengths.push_back(std::get<1>(answer));
    //             successfulPlans++;
    //         }
    //     }

    //     result.successfulPlans = successfulPlans;
    //     return result;
    // }

int main(int argc, char** argv) {

    //     // Problems:
    //     Problem2D problem1 = HW5::getWorkspace1();
    //     Problem2D problem2 = HW2::getWorkspace1();
    //     Problem2D problem3 = HW2::getWorkspace2();

    //     // Excercise 1. a
    //         MyPRM2D prm;
    //         auto answer1 = prm.plan(problem3, 2, 400, false);
    //         Visualizer::makeFigure(problem3, std::get<0>(answer1));
    //         std::cout<<"Path length: "<<std::get<1>(answer1)<<std::endl;

    //         struct PlanConfig {
    //             int n;
    //             double r;
    //             std::string label;
    //         };

        //     std::vector<PlanConfig> configs = {
        //         {200, 0.5, "n = 200, r = 0.5"},
        //         {200, 1,   "n = 200, r = 1"},
        //         {200, 1.5, "n = 200, r = 1.5"},
        //         {200, 2, "n = 200, r = 2"},
        //         {500, 0.5,   "n = 500, r = 0.5"},
        //         {500, 1,   "n = 500, r = 1"},
        //         {500, 1.5, "n = 500, r = 1.5"},
        //         {500, 2, "n = 500, r = 2"},
        //     };

        //     std::vector<std::vector<double>> allTimesPRM;
        //     std::vector<std::vector<double>> allPathLengthsPRM;
        //     std::vector<std::vector<double>> allSuccessRatesPRM;
        //     std::vector<std::string> labels;

        //     for (const auto& config : configs) {
        //         PlanResult currentResult = computeStatsPRM(prm, problem1, config.r, config.n);
        //         allTimesPRM.push_back(currentResult.times);
        //         allPathLengthsPRM.push_back(currentResult.pathLengths);
        //         allSuccessRatesPRM.push_back({currentResult.successfulPlans});
        //         labels.push_back(config.label);
        //     }

        //     std::list<std::vector<double>> allTimesListPRM(allTimesPRM.begin(), allTimesPRM.end());
        //     std::list<std::vector<double>> allPathLengthsListPRM(allPathLengthsPRM.begin(), allPathLengthsPRM.end());
        //     std::list<std::vector<double>> allSuccessRatesListPRM(allSuccessRatesPRM.begin(), allSuccessRatesPRM.end());

            // std::string xlabel = "n, r";
            // std::string title;

        //     // Plot for Times
        //     title = "Time vs. n, r for problem 1";
        //     std::string ylabelTime = "time (ms)";
        //     Visualizer::makeBoxPlot(allTimesListPRM, labels, title, xlabel, ylabelTime);

        //     // // Plot for Path Lengths
        //     title = "Path Length vs. n, r for problem 1";
        //     std::string ylabelLength = "Path Length";
        //     Visualizer::makeBoxPlot(allPathLengthsListPRM, labels, title, xlabel, ylabelLength);

        //     // // Plot for Success Rates
        //     title = "Success Rate vs. n, r for problem 1";
        //     std::string ylabelRate = "Successful attempts out of 100";
        //     Visualizer::makeBoxPlot(allSuccessRatesListPRM, labels, title, xlabel, ylabelRate);

        // // Exercise 1. b
        //     allTimesPRM.clear();
        //     allPathLengthsPRM.clear();
        //     allSuccessRatesPRM.clear();
        //     allTimesListPRM.clear();
        //     allPathLengthsListPRM.clear();
        //     allSuccessRatesListPRM.clear();
        //     labels.clear();

        //     for (const auto& config : configs) {
        //         PlanResult currentResult = computeStatsPRM(prm, problem2, config.r, config.n);
        //         allTimesPRM.push_back(currentResult.times);
        //         allPathLengthsPRM.push_back(currentResult.pathLengths);
        //         allSuccessRatesPRM.push_back({currentResult.successfulPlans});
        //         labels.push_back(config.label);
        //     }

        //     allTimesListPRM.assign(allTimesPRM.begin(), allTimesPRM.end());
        //     allPathLengthsListPRM.assign(allPathLengthsPRM.begin(), allPathLengthsPRM.end());
        //     allSuccessRatesListPRM.assign(allSuccessRatesPRM.begin(), allSuccessRatesPRM.end());

        //     // Plot for Times
        //     title = "Time vs. n, r for problem 2";
        //     Visualizer::makeBoxPlot(allTimesListPRM, labels, title, xlabel, ylabelTime);

        //     // Plot for Path Lengths
        //     title = "Path Length vs. n, r for problem 2";
        //     Visualizer::makeBoxPlot(allPathLengthsListPRM, labels, title, xlabel, ylabelLength);

        //     // Plot for Success Rates
        //     title = "Success Rate vs. n, r for problem 2";
        //     Visualizer::makeBoxPlot(allSuccessRatesListPRM, labels, title, xlabel, ylabelRate);

        // // Exercise 1. c
        //     allTimesPRM.clear();
        //     allPathLengthsPRM.clear();
        //     allSuccessRatesPRM.clear();
        //     allTimesListPRM.clear();
        //     allPathLengthsListPRM.clear();
        //     allSuccessRatesListPRM.clear();
        //     labels.clear();

        //     for (const auto& config : configs) {
        //         PlanResult currentResult = computeStatsPRM(prm, problem3, config.r, config.n);
        //         allTimesPRM.push_back(currentResult.times);
        //         allPathLengthsPRM.push_back(currentResult.pathLengths);
        //         allSuccessRatesPRM.push_back({currentResult.successfulPlans});
        //         labels.push_back(config.label);
        //     }

        //     allTimesListPRM.assign(allTimesPRM.begin(), allTimesPRM.end());
        //     allPathLengthsListPRM.assign(allPathLengthsPRM.begin(), allPathLengthsPRM.end());
        //     allSuccessRatesListPRM.assign(allSuccessRatesPRM.begin(), allSuccessRatesPRM.end());

        //     // Plot for Times
        //     title = "Time vs. n, r for problem 3";
        //     Visualizer::makeBoxPlot(allTimesListPRM, labels, title, xlabel, ylabelTime);

        //     // Plot for Path Lengths
        //     title = "Path Length vs. n, r for problem 3";
        //     Visualizer::makeBoxPlot(allPathLengthsListPRM, labels, title, xlabel, ylabelLength);

        //     // Plot for Success Rates
        //     title = "Success Rate vs. n, r for problem 3";
        //     Visualizer::makeBoxPlot(allSuccessRatesListPRM, labels, title, xlabel, ylabelRate);
            
        

        // Excercise 2

            // MyGoalBiasRRT2D rrt;

            // auto answer2_1 = rrt.plan(problem1, 0.5, 5000, 0.05, 0.25, false);
            // Visualizer::makeFigure(problem1, std::get<0>(answer2_1));
            // std::cout<<"Path length: "<<std::get<1>(answer2_1)<<std::endl;

            // problem2:
            // auto answer2_2 = rrt.plan(problem2, 0.5, 5000, 0.05, 0.5, false);
            // Visualizer::makeFigure(problem2, std::get<0>(answer2_2));
            // std::cout<<"Path length: "<<std::get<1>(answer2_2)<<std::endl;
            // HW2::check(std::get<0>(answer2_2), problem2);

            // problem3:
            // auto answer2_3 = rrt.plan(problem3, 0.5, 5000, 0.05, 0.5, false);
            // Visualizer::makeFigure(problem3, std::get<0>(answer2_3));
            // std::cout<<"Path length: "<<std::get<1>(answer2_3)<<std::endl;
            // HW2::check(std::get<0>(answer2_3), problem3);

        //     // running trials
        //         PlanResult rrt_prob1_results = computeStatsRRT(rrt, problem1, 0.5, 5000, 0.05, 0.25);
        //         PlanResult rrt_prob2_results = computeStatsRRT(rrt, problem2, 0.5, 5000, 0.05, 0.5);
        //         PlanResult rrt_prob3_results = computeStatsRRT(rrt, problem3, 0.5, 5000, 0.05, 0.5);
        //         std::vector<std::string> labelsRRT;
        //         labelsRRT.push_back("Problem 1");
        //         labelsRRT.push_back("Problem 2");
        //         labelsRRT.push_back("Problem 3");
        //         xlabel = "Problem";
        //         std::string ylabelLength = "Path Length";

        //         // path lengths
        //             std::vector<std::vector<double>> allPathLengthsRRT;
        //             allPathLengthsRRT.push_back(rrt_prob1_results.pathLengths);
        //             allPathLengthsRRT.push_back(rrt_prob2_results.pathLengths);
        //             allPathLengthsRRT.push_back(rrt_prob3_results.pathLengths);
        //             std::list<std::vector<double>> allPathLengthsListRRT(allPathLengthsRRT.begin(), allPathLengthsRRT.end());
        //             title = "Path Length for RRT for all problems";
        //             Visualizer::makeBoxPlot(allPathLengthsListRRT, labelsRRT, title, xlabel, ylabelLength);
                    
                
        //         // computation times:
        //             std::vector<std::vector<double>> allTimesRRT;
        //             allTimesRRT.push_back(rrt_prob1_results.times);
        //             allTimesRRT.push_back(rrt_prob2_results.times);
        //             allTimesRRT.push_back(rrt_prob3_results.times);
        //             std::list<std::vector<double>> allTimesListRRT(allTimesRRT.begin(), allTimesRRT.end());
        //             std::string ylabelTime = "time (ms)";
        //             title = "Computation Time for RRT for all problems";
        //             Visualizer::makeBoxPlot(allTimesListRRT, labelsRRT, title, xlabel, ylabelTime);

        //         // success rates:
        //             std::vector<std::vector<double>> allSuccessRatesRRT;
        //             allSuccessRatesRRT.push_back({rrt_prob1_results.successfulPlans});
        //             allSuccessRatesRRT.push_back({rrt_prob2_results.successfulPlans});
        //             allSuccessRatesRRT.push_back({rrt_prob3_results.successfulPlans});
        //             std::list<std::vector<double>> allSuccessRatesListRRT(allSuccessRatesRRT.begin(), allSuccessRatesRRT.end());
        //             std::string ylabelRate = "Success Rate";
        //             title = "Number of Successful Plans for RRT for all problems with 100 runs";
        //             Visualizer::makeBoxPlot(allSuccessRatesListRRT, labelsRRT, title, xlabel, ylabelRate);
            
                 
        // Visualizer::showFigures();
        // amp::HW7::hint();
        amp::HW7::grade<MyPRM2D, MyGoalBiasRRT2D>("nipe1783@colorado.edu", argc, argv);

    return 0;
}







