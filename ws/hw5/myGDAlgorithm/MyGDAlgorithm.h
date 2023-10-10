#pragma once
#include "AMPCore.h"
#include "hw/HW5.h"

namespace amp{
    class MyGDAlgorithm : public GDAlgorithm{
        public:
            // constructor and destructor
            MyGDAlgorithm();
            ~MyGDAlgorithm();

            Path2D plan(const Problem2D& prob) override;
            /**
             * @brief Computes the attractive forces from the robot to goal
             * 
             * @param prob robot environment.
             * @return vector representing the attractive vector.
             **/
            Eigen::Vector2d uAtt(const Problem2D& prob);

            /**
             * @brief Computes the repelling forces from the robot to obstacles
             * 
             * @param prob robot environment.
             * @return vector representing the repelling vector.
             **/
            Eigen::Vector2d uRep(const Problem2D& prob);
    };
};