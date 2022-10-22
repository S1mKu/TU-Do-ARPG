#include "data_association.hpp"

#include "util.hpp"

#include <cmath>
#include <limits>
#include <Eigen/Dense>

#include <ros/ros.h>


class DistanceMetric { public: virtual double compute(PointList source_segment, PointList target_segment) = 0; };

class MeanDistanceMetric : public DistanceMetric
{
public:
    double compute(PointList source_segment, PointList target_segment)
    {
        Point p1 = mean(source_segment);
        Point p2 = mean(target_segment);
        return std::pow(std::pow(p1.first - p2.first, 2.0) + std::pow(p1.second - p2.second, 2.0), 0.5);
    }
};

class EuclideanDistanceMetric : public DistanceMetric
{
public:
    double compute(PointList source_segment, PointList target_segment)
    {
        double dist = 0.0;

        for (Point p1 : target_segment)
        {
            double min_dist = 999999.0;

            for (Point p2 : source_segment)
            {
                double d = std::pow(std::pow(p1.first - p2.first, 2.0) + std::pow(p1.second - p2.second, 2.0), 0.5);

                if (d < min_dist)
                {
                    min_dist = d;
                }

                dist += d;
            }

            // dist += min_dist;
        }

        return dist / ((double) source_segment.size() * target_segment.size());
    }
};

class Point2PointDistanceMetric : public DistanceMetric
{
public:
    double compute(PointList source_segment, PointList target_segment)
    {
        double min_dist = 9999999999.9; // just a big enough number
        int min_i = 0;
        int min_j = 0;

        for (int i = 0; i < source_segment.size(); i++)
        {
            Point p1 = source_segment.at(i);

            for (int j = 0; j < target_segment.size(); j++)
            {
                Point p2 = target_segment.at(j);

                double dist = std::pow(std::pow(p1.first - p2.first, 2.0) + std::pow(p1.second - p2.second, 2.0), 0.5);

                if (dist < min_dist)
                {
                    min_dist = dist;
                    min_i = i;
                    min_j = j;
                }
            }
        }

        double min_sf = 9999999999.9; // just a big enough number
        Point p1 = source_segment.front();

        for (int j = 0; j < target_segment.size(); j++)
        {
            Point p2 = target_segment.at(j);

            double dist = std::pow(std::pow(p1.first - p2.first, 2.0) + std::pow(p1.second - p2.second, 2.0), 0.5);

            if (dist < min_sf)
            {
                min_sf = dist;
            }
        }

        double min_sl = 9999999999.9; // just a big enough number
        p1 = source_segment.back();

        for (int j = 0; j < target_segment.size(); j++)
        {
            Point p2 = target_segment.at(j);

            double dist = std::pow(std::pow(p1.first - p2.first, 2.0) + std::pow(p1.second - p2.second, 2.0), 0.5);

            if (dist < min_sl)
            {
                min_sl = dist;
            }
        }

        double min_tf = 9999999999.9; // just a big enough number
        p1 = target_segment.front();

        for (int j = 0; j < source_segment.size(); j++)
        {
            Point p2 = source_segment.at(j);

            double dist = std::pow(std::pow(p1.first - p2.first, 2.0) + std::pow(p1.second - p2.second, 2.0), 0.5);

            if (dist < min_sf)
            {
                min_sf = dist;
            }
        }

        double min_tl = 9999999999.9; // just a big enough number
        p1 = target_segment.back();

        for (int j = 0; j < source_segment.size(); j++)
        {
            Point p2 = source_segment.at(j);

            double dist = std::pow(std::pow(p1.first - p2.first, 2.0) + std::pow(p1.second - p2.second, 2.0), 0.5);

            if (dist < min_tl)
            {
                min_tl = dist;
            }
        }

        double threshold = 0.1;

        if ((min_sl < min_sf && min_tf < min_tl && min_sl < threshold && min_tf < threshold)
         || (min_sf < min_sl && min_tl < min_tf && min_sf < threshold && min_tl < threshold))
        {
            return min_dist;
        }

        return 9999999999.9; // just a big enough number
    }
};


std::vector<association_t> associate(std::vector<Obstacle> source, std::vector<segment_t> target, DistanceMetric &dist_metric)
{
    std::vector<association_t> associations;
    int obst_ind = 0;
    int n = source.size();
    int m = target.size();

    if (n == 0)
    {
        return associations;
    }

    Eigen::MatrixXd dist_matrix = Eigen::MatrixXd::Zero(n, m);

    for (unsigned int s = 0; s < n; s++)
    {
        segment_t source_seg = source.at(s).segment_backlog.back();

        for (unsigned int t = 0; t < m; t++)
        {
            segment_t target_seg = target.at(t);
            
            if (target_seg.invalidated)
            {
                dist_matrix(s, t) = 9999999.0 - 1.0; // just a big enough number to exceed any threshold
            }
            else
            {
                double dist = dist_metric.compute(source_seg.points, target_seg.points);
                dist_matrix(s, t) = dist;
            }
        }
    }

    int upper_bound = n;
    int lower_bound = std::min(n, m);

    for (unsigned int i = 0; i < upper_bound; i++)
    {    
        // find smallest element
        int s_ind = 0;
        int t_ind = 0;
        double min_dist = std::numeric_limits<double>::infinity();

        for (unsigned int s = 0; s < n; s++)
        {
            for (unsigned int t = 0; t < m; t++)
            {
                double dist = dist_matrix(s, t);
                
                if (dist < min_dist)
                {
                    s_ind = s;
                    t_ind = t;
                    min_dist = dist;
                }
            }
        }

        if (i < lower_bound && !target.at(t_ind).invalidated)
        {
            association_t association;
            association.obstacle_index = s_ind;
            association.segment_index = t_ind;
            association.distance = min_dist;
            associations.push_back(association);

            dist_matrix.col(t_ind).array() = 9999999.0 - 1.0;
            dist_matrix.row(s_ind).array() = 9999999.0; // just a big enough number to exceed any threshold
        }
        else
        {
            association_t association;
            association.obstacle_index = s_ind;
            association.segment_index = -1;
            association.distance = std::numeric_limits<double>::infinity();
            association.obstacle_unassociated = true;
            associations.push_back(association);

            dist_matrix.row(s_ind).array() = 9999999.0; // just a big enough number to exceed any threshold
        }
    }

    return associations;
}

/**
 * @brief 
 * 
 * NOTE: Associaties obstacle -> segment. Thus, multiple obstacles can be associated to the same segment. Be aware of that!!!
 * 
 * @param obstacles 
 * @param segments 
 * @param dist_metric 
 * @return std::vector<association_t> 
 */
std::vector<association_t> associate_static_obstacles(std::vector<Obstacle> source, std::vector<segment_t> target)
{
    Point2PointDistanceMetric dist_metric = Point2PointDistanceMetric();
    return associate(source, target, dist_metric);
}

std::vector<association_t> associate_dynamic_obstacles(std::vector<Obstacle> source, std::vector<segment_t> target)
{
    MeanDistanceMetric dist_metric = MeanDistanceMetric();
    return associate(source, target, dist_metric);
}
