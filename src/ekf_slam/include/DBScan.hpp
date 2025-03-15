#ifndef DBSCAN_HPP
#define DBSCAN_HPP

#include <vector>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <eigen3/Eigen/Dense>

struct Point {
    double x, y;
    bool visited = false;  
    int clusterID = -1;    
};

struct Circle {
    double x, y, r;
};

class DBSCAN {
public:
    DBSCAN(double eps, int minPts);
    
    void fit(std::vector<Point>& points);
    std::vector<std::vector<Point>> getClusters(const std::vector<Point>& points);
    Circle fitCircle(const std::vector<Point>& cluster);

private:
    double eps;
    int minPts;
    
    double distance(const Point& p1, const Point& p2) const; 
    std::vector<Point*> regionQuery(std::vector<Point>& points, const Point& target);
    void expandCluster(std::vector<Point>& points, Point& point, std::vector<Point*>& neighbors, int clusterID);
};

#endif
