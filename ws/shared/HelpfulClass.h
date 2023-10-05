#include "AMPCore.h"

class MyClass {
    public:
        void hereIsAMethod();
};

class Helper {
    public:
        static amp::Polygon minkowskiSum(amp::Polygon& obstacle, amp::Polygon& robot);
        static amp::Polygon minkowskiDiff(amp::Polygon& obstacle, amp::Polygon& robot);
        static amp::Polygon rotatePolygon(amp::Polygon& p, Eigen::Vector2d rotationVertex, float theta);
        bool intersects(const amp::Environment2D& environment, Eigen::Vector2d& start, Eigen::Vector2d& end);
        float angle(Eigen::Vector2d& v1, Eigen::Vector2d& v2);
        amp::Polygon orderVertices(amp::Polygon& polygon);
        bool isPointOnSegment(const Eigen::Vector2d &start, const Eigen::Vector2d &end, const Eigen::Vector2d &point);
};