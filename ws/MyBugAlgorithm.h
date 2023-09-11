#pragma once

#include "AMPCore.h"
#include "hw/HW2.h"

/// @brief Declare your bug algorithm class here. Note this class derives the bug algorithm class declared in HW2.h
class MyBugAlgorithm : public amp::BugAlgorithm {
    public:
        // Override and implement the bug algorithm in the plan method. The methods are declared here in the `.h` file
        virtual amp::Path2D plan(const amp::Problem2D& problem) const override;

        // Add any other methods here...
        Eigen::Vector2d stepToGoal(const amp::Problem2D& problem, Eigen::Vector2d location, float delta) const;
        bool occupied(Eigen::Vector2d location, const amp::Problem2D& problem) const;
        bool isPointOnLine(Eigen::Vector2d vert1, Eigen::Vector2d  vert2, Eigen::Vector2d  location) const;

    private:
};