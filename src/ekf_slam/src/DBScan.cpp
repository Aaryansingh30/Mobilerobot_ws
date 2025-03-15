#include "DBScan.hpp"

DBSCAN::DBSCAN(double eps, int minPts) : eps(eps), minPts(minPts) {}

void DBSCAN::fit(std::vector<Point>& points) {
    int clusterID = 1;
    for (auto& point : points) {
        if (point.visited) continue;

        point.visited = true;
        std::vector<Point*> neighbors = regionQuery(points, point);
        if (neighbors.size() < static_cast<size_t>(minPts)) {
            point.clusterID = 0; // Mark as noise
        } else {
            expandCluster(points, point, neighbors, clusterID);
            clusterID++;
        }
    }
}

std::vector<std::vector<Point>> DBSCAN::getClusters(const std::vector<Point>& points) {
    std::unordered_map<int, std::vector<Point>> cluster_map;
    std::vector<std::vector<Point>> clusters;

    for (const auto& point : points) {
        if (point.clusterID > 0) {
            cluster_map[point.clusterID].push_back(point);
        }
    }

    for (auto& entry : cluster_map) {
        clusters.push_back(std::move(entry.second));
    }

    return clusters;
}

Circle DBSCAN::fitCircle(const std::vector<Point>& cluster) {
    int n = cluster.size();
    if (n < 3) return {0.0, 0.0, 0.0}; // Need at least 3 points to fit a circle

    double sum_x = 0, sum_y = 0, sum_xx = 0, sum_yy = 0, sum_xy = 0, sum_xr = 0, sum_yr = 0;

    for (const auto& p : cluster) {
        double x = p.x, y = p.y;
        double r2 = x * x + y * y;
        sum_x += x;
        sum_y += y;
        sum_xx += x * x;
        sum_yy += y * y;
        sum_xy += x * y;
        sum_xr += x * r2;
        sum_yr += y * r2;
    }

    Eigen::Matrix3d A;
    Eigen::Vector3d B;

    A << sum_xx, sum_xy, sum_x,
         sum_xy, sum_yy, sum_y,
         sum_x,  sum_y,  n;

    if (A.determinant() < 1e-6) {  // Check for singular matrix (collinear points)
        return {0.0, 0.0, 0.0};
    }

    B << -sum_xr, -sum_yr, -sum_xx - sum_yy;

    Eigen::Vector3d X = A.colPivHouseholderQr().solve(B);

    double a = -0.5 * X(0);
    double b = -0.5 * X(1);
    double r = std::sqrt(a * a + b * b - X(2));

    return {a, b, r};
}

double DBSCAN::distance(const Point& p1, const Point& p2)const {
    return std::hypot(p1.x - p2.x, p1.y - p2.y);
}

std::vector<Point*> DBSCAN::regionQuery(std::vector<Point>& points, const Point& target) {
    std::vector<Point*> neighbors;
    for (auto& point : points) {
        if (&point != &target && distance(target, point) <= eps) {  // Avoid self-inclusion
            neighbors.push_back(&point);
        }
    }
    return neighbors;
}

void DBSCAN::expandCluster(std::vector<Point>& points, Point& point, std::vector<Point*>& neighbors, int clusterID) {
    point.clusterID = clusterID;
    std::queue<Point*> queue;
    for (auto* p : neighbors) queue.push(p);

    while (!queue.empty()) {
        Point* current = queue.front();
        queue.pop();

        if (!current->visited) {
            current->visited = true;
            std::vector<Point*> newNeighbors = regionQuery(points, *current);
            if (newNeighbors.size() >= static_cast<size_t>(minPts)) {
                for (auto* p : newNeighbors) queue.push(p);
            }
        }

        if (current->clusterID == -1 || current->clusterID == 0) {
            current->clusterID = clusterID;
        }
    }
}
