#include "AMPCore.h"
#include "hw/HW2.h"
#include "hw/HW5.h"
#include "hw/HW6.h"
#include "hw/HW7.h"
#include "MyPRM2D.h"
#include "MyGoalBiasRRT2D.h"
#include "Helper.h"

using namespace amp;


    struct PlanResult {
        std::vector<double> times;
        std::vector<double> pathLengths;
        double successfulPlans;
    };

    // Function to compute times based on given parameters
    PlanResult computeStats(MyPRM2D& prm, const Problem2D& problem, double r, int n, int iterations = 20) {
        PlanResult result;
        double successfulPlans = 0;

        for(int i = 0; i < iterations; i++) {
            auto answer = prm.plan(problem, r, n);
            result.times.push_back(std::get<2>(answer));

            if(std::get<0>(answer).waypoints.size() == 0) {
                std::cout << "No path found" << std::endl;
            } else {
                std::cout << "Path found" << std::endl;
                result.pathLengths.push_back(std::get<1>(answer));
                successfulPlans++;
            }
        }

        result.successfulPlans = successfulPlans;
        return result;
    }

    int main(int argc, char** argv) {

        // Problems:
        Problem2D problem1 = HW5::getWorkspace1();
        Problem2D problem2 = HW2::getWorkspace1();
        Problem2D problem3 = HW2::getWorkspace2();

        // Excercise 1
            // MyPRM2D prm;
            // auto answer1 = prm.plan(problem1, 2, 200, true);
            // Visualizer::makeFigure(problem1, std::get<0>(answer1));

            // struct PlanConfig {
            //     int n;
            //     double r;
            //     std::string label;
            // };

            // std::vector<PlanConfig> configs = {
            //     {200, 0.5, "n = 200, r = 0.5"},
            //     {200, 1,   "n = 200, r = 1"},
            //     {200, 1.5, "n = 200, r = 1.5"},
            //     {500, 0.5, "n = 500, r = 0.5"},
            //     {500, 1,   "n = 500, r = 1"},
            //     {500, 1.5, "n = 500, r = 1.5"},
            //     {500, 2,   "n = 500, r = 2"}
            // };

            // std::vector<std::vector<double>> allTimes;
            // std::vector<std::vector<double>> allPathLengths;
            // std::vector<std::vector<double>> allSuccessRates;
            // std::vector<std::string> labels;

            // for (const auto& config : configs) {
            //     PlanResult currentResult = computeStats(prm, problem1, config.r, config.n);
            //     allTimes.push_back(currentResult.times);
            //     allPathLengths.push_back(currentResult.pathLengths);
            //     allSuccessRates.push_back({currentResult.successfulPlans});
            //     labels.push_back(config.label);
            // }

            // std::list<std::vector<double>> allTimesList(allTimes.begin(), allTimes.end());
            // std::list<std::vector<double>> allPathLengthsList(allPathLengths.begin(), allPathLengths.end());
            // std::list<std::vector<double>> allSuccessRatesList(allSuccessRates.begin(), allSuccessRates.end());

            // std::string xlabel = "n, r";
            // std::string title;

            // // Plot for Times
            // title = "Time vs. n, r for problem 1";
            // std::string ylabelTime = "time (ms)";
            // Visualizer::makeBoxPlot(allTimesList, labels, title, xlabel, ylabelTime);

            // // Plot for Path Lengths
            // title = "Path Length vs. n, r for problem 1";
            // std::string ylabelLength = "Path Length";
            // Visualizer::makeBoxPlot(allPathLengthsList, labels, title, xlabel, ylabelLength);

            // // Plot for Success Rates
            // title = "Success Rate vs. n, r for problem 1";
            // std::string ylabelRate = "Success Rate";
            // Visualizer::makeBoxPlot(allSuccessRatesList, labels, title, xlabel, ylabelRate);

        // Excercise 2

            // MyGoalBiasRRT2D rrt;
            // auto answer2_1 = rrt.plan(problem1, 0.5, 5000, 0.05, 0.25, false);
            // Path2D path1 = rrt.plan(problem1);
            // Visualizer::makeFigure(problem1, path1);
            // std::cout<<"Path length: "<<std::get<1>(answer2_1)<<std::endl;

            // problem2:
            // auto answer2_2 = rrt.plan(problem2, 0.5, 5000, 0.05, 0.25, false);
            // Path2D path2 = rrt.plan(problem1);
            // Visualizer::makeFigure(problem1, path2);
            // Visualizer::makeFigure(problem2, std::get<0>(answer2_2));
            // std::cout<<"Path length: "<<std::get<1>(answer2_2)<<std::endl;

            // problem3:
            // auto answer2_3 = rrt.plan(problem3, 0.5, 5000, 0.05, 0.25, false);
            // Visualizer::makeFigure(problem3, std::get<0>(answer2_3));
            // std::cout<<"Path length: "<<std::get<1>(answer2_3)<<std::endl;



        amp::HW7::grade<MyPRM2D, MyGoalBiasRRT2D>("nipe1783@colorado.edu", argc, argv);

        Visualizer::showFigures();
    return 0;
}






