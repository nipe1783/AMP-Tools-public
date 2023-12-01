# pragma once

#include "AMPCore.h"
#include <boost/geometry/io/io.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <ompl/tools/config/SelfConfig.h>
#include <yaml-cpp/yaml.h>

typedef boost::geometry::model::d2::point_xy<double> point;
typedef boost::geometry::model::polygon<point> polygon;

class ObstacleOmpl
{

    public:

        // constructor:
        ObstacleOmpl(const polygon& poly);
        ObstacleOmpl(const amp::Obstacle2D& obstacle);

        // fields:
        polygon poly_;

};