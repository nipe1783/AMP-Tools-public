#include "ObstacleOmpl.h"
#include "AMPCore.h"
#include <boost/geometry/io/io.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <ompl/tools/config/SelfConfig.h>
#include <yaml-cpp/yaml.h>

typedef boost::geometry::model::d2::point_xy<double> point;
typedef boost::geometry::model::polygon<point> polygon;

ObstacleOmpl::ObstacleOmpl(const polygon& poly) : poly_(poly) {}

ObstacleOmpl::ObstacleOmpl(const amp::Obstacle2D& obstacle){
    std::vector<boost::geometry::model::d2::point_xy<double>> points;
    for(int i = 0; i < obstacle.verticesCCW().size(); i++){
        points.push_back(boost::geometry::model::d2::point_xy<double>(obstacle.verticesCCW()[i][0], obstacle.verticesCCW()[i][1]));
    }
    boost::geometry::assign_points(poly_, points);
}