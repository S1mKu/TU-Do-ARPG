#include "scan_matching.hpp"

#include <limits.h>
#include <cmath>

#include <chrono>
#include <thread>
#include <limits>
#include <math.h>
#include <cmath>

#include "geometry_msgs/Point32.h"

#include "visualization_msgs/MarkerArray.h"


ICP::ICP(ros::NodeHandle &nh): node_handle_(nh)
{
    rviz_pub = node_handle_.advertise<visualization_msgs::Marker>("scan_matching", 1);
    rviz_pub_arr = node_handle_.advertise<visualization_msgs::MarkerArray>("scan_matching_association", 1);
}

void ICP::visualize(Eigen::Matrix2Xd target, Eigen::Matrix2Xd source)
{
    visualization_msgs::Marker markers;

    markers.header.frame_id = "map";
    markers.header.stamp = ros::Time::now();
    markers.ns = "icp_target";
    markers.action = visualization_msgs::Marker::ADD;
    markers.pose.orientation.w = 1.0;

    markers.id = 0;

    markers.type = visualization_msgs::Marker::POINTS;

    // points are green
    markers.color.r = 0.0;
    markers.color.g = 1.0;
    markers.color.b = 0.2;
    markers.color.a = 1.0;

    markers.scale.x = 0.05;

    for (int i = 0; i < target.cols(); i++)
    {
        if (source(0, i) == target(0, i) && source(1, i) == target(1, i))
        {
            continue;
        }

        geometry_msgs::Point p;
        p.x = target(0, i);
        p.y = target(1, i);

        markers.points.push_back(p);
    }

    rviz_pub.publish(markers);


    markers.points.clear();

    markers.header.stamp = ros::Time::now();
    markers.ns = "icp_source";

    // points are red
    markers.color.r = 1.0;
    markers.color.g = 0.0;
    markers.color.b = 0.2;
    markers.color.a = 1.0;

    for (int i = 0; i < source.cols(); i++)
    {
        if (source(0, i) == target(0, i) && source(1, i) == target(1, i))
        {
            continue;
        }

        geometry_msgs::Point p;
        p.x = source(0, i);
        p.y = source(1, i);

        markers.points.push_back(p);
    }

    rviz_pub.publish(markers);
}

std::vector<std::pair<int, int>> ICP::get_correspondences(Eigen::Matrix2Xd target, Eigen::Matrix2Xd source)
{
    std::vector<std::pair<int, int>> correspondences;

    for (unsigned int i = 0; i < source.cols(); i++)
    {
        double min_dist = std::numeric_limits<double>::max();
        int min_dist_index = 0;

        for (unsigned j = 0; j < target.cols(); j++)
        {
            double dist = std::pow(std::pow(target(0, j) - source(0, i), 2.0) + std::pow(target(1, j) - source(1, i), 2.0), 0.5);

            if (dist < min_dist)
            {
                min_dist = dist;
                min_dist_index = j;
            }
        }

        std::pair<int, int> corr_pair(i, min_dist_index);
        correspondences.push_back(corr_pair);
    }

    return correspondences;
}

Eigen::Vector2d ICP::get_center(Eigen::Matrix2Xd data, std::vector<int> exclude_indices)
{
    Eigen::Matrix2Xd cleaned_data = Eigen::Matrix2Xd::Zero(2, data.cols() - exclude_indices.size());

    unsigned int j = 0;

    for (unsigned int i = 0; i < data.cols(); i++)
    {
        if (!exclude_indices.empty() && exclude_indices.front() == i)
        {
            exclude_indices.erase(exclude_indices.begin());
            j++;
        }
        else
        {
            cleaned_data(0, i - j) = data(0, i);
            cleaned_data(1, i - j) = data(1, i);
        }
    }

    Eigen::Vector2d center(cleaned_data.row(0).mean(), cleaned_data.row(1).mean());
    return center;
}

double ICP::compute_error(Eigen::Matrix2Xd source, Eigen::Matrix2Xd target)
{
    Eigen::VectorXd local_err = (source - target).colwise().squaredNorm();
    double err = 0.0;
    double n = 0;
    double min_err = local_err.minCoeff();
    double max_err = local_err.maxCoeff();

    for (int i = 0; i < local_err.size(); i++)
    {
        // if (local_err(i) > 2.0)
        // {
        //     double weight = 0.5 + 0.5 * (local_err(i) - min_err) / (max_err - min_err);
        //     err += local_err(i); // * weight;
        //     n += 1;
        // }

        err += local_err(i);
    }

    return err / local_err.size();
}

Eigen::Matrix2d ICP::get_cross_covariance(Eigen::Matrix2Xd target, Eigen::Matrix2Xd source, std::vector<std::pair<int, int>> correspondences)
{
    Eigen::Matrix2d cov;

    for (unsigned int i = 0; i < correspondences.size(); i++)
    {
        std::pair<int, int> association = correspondences.at(i);

        Eigen::Vector2d p = source.col(association.first);
        Eigen::Vector2d q = target.col(association.second);

        // weight = kernel(p_point - q_point)
        double weight = 1.0;

        cov += weight * q * p.transpose();
    }

    return cov;
}

bool ICP::is_inner_beam(beam_t beam, geometry_msgs::Polygon *centerline)
{
    // compute nearest poitn of centerline to beam
    double min_dist = std::numeric_limits<double>::max();
    int min_ind = 0;

    int i = 0;
    for (geometry_msgs::Point32 p : centerline->points)
    {
        double dist = std::sqrt(std::pow(p.x - beam.endpoint_x, 2.0) + std::pow(p.y - beam.endpoint_y, 2.0));

        if (dist < min_dist)
        {
            min_dist = dist;
            min_ind = i;
        }

        i++;
    }

    int i1 = min_ind;
    int i2 = (min_ind + 1) % (int) centerline->points.size();

    if (i1 > i2)
    {
        int tmp = i1;
        i1 = i2;
        i2 = tmp;
    }

    geometry_msgs::Point32 p1 = centerline->points[i1]; // point on centerline
    geometry_msgs::Point32 p2 = centerline->points[i2]; // point on centerline

    // cross product
    return ((p2.x - p1.x) * (beam.endpoint_y - p1.y) - (p2.y - p1.y) * (beam.endpoint_x - p1.x)) > 0;
}

std::vector<beam_t> ICP::compute_svd_icp(map_t *map, std::vector<beam_t> beams_orig, unsigned int iterations, geometry_msgs::Polygon *centerline)
{
    std::vector<beam_t> beams;
    std::vector<int> exclude_indices;
    Eigen::Vector2d translation;
    int N = beams_orig.size();

    // copy over beams
    for (int i = 0; i < N; i++)
    {
        beam_t beam = beams_orig.at(i);
        beams.push_back(beam);
    }

    Eigen::Matrix2Xd best_fit = Eigen::Matrix2Xd::Zero(2, N);
    double min_error = std::numeric_limits<double>::max();

    // retrieve source and target data points
    Eigen::Matrix2Xd source = Eigen::Matrix2Xd::Zero(2, N);
    Eigen::Matrix2Xd target = Eigen::Matrix2Xd::Zero(2, N);

    std::vector<bool> is_inner_beam_list;

    for (beam_t b : beams)
    {
        is_inner_beam_list.push_back(is_inner_beam(b, centerline));
    }

    visualization_msgs::Marker m1;

    m1.header.frame_id = "map";
    m1.header.stamp = ros::Time::now();
    m1.ns = "inner_beams";
    m1.action = visualization_msgs::Marker::ADD;
    m1.pose.orientation.w = 1.0;

    m1.id = 0;

    m1.type = visualization_msgs::Marker::POINTS;

    // points are green
    m1.color.r = 0.0;
    m1.color.g = 1.0;
    m1.color.b = 1.0;
    m1.color.a = 1.0;

    m1.scale.x = 0.05;

    visualization_msgs::Marker m2;

    m2.header.frame_id = "map";
    m2.header.stamp = ros::Time::now();
    m2.ns = "outer_bams";
    m2.action = visualization_msgs::Marker::ADD;
    m2.pose.orientation.w = 1.0;

    m2.id = 0;

    m2.type = visualization_msgs::Marker::POINTS;

    // points are red
    m2.color.r = 1.0;
    m2.color.g = 0.0;
    m2.color.b = 1.0;
    m2.color.a = 1.0;

    m2.scale.x = 0.05;

    for (int i = 0; i < target.cols(); i++)
    {
        beam_t beam = beams.at(i);

        if (is_inner_beam_list.at(i))
        {
            geometry_msgs::Point p;
            p.x = beam.endpoint_x;
            p.y = beam.endpoint_y;

            m1.points.push_back(p);
        }
        else
        {
            geometry_msgs::Point p;
            p.x = beam.endpoint_x;
            p.y = beam.endpoint_y;

            m2.points.push_back(p);
        }
    }

    rviz_pub.publish(m1);
    rviz_pub.publish(m2);

    visualization_msgs::MarkerArray marker_array;

    for (unsigned int iteration = 0; iteration < iterations; iteration++)
    {
        for (int i = 0; i < N; i++)
        {
            map_cell_t *beam_endpoint_cell = NULL;

            if (iteration == 0)
            {
                beam_t beam = beams.at(i);

                source(0, i) = beam.endpoint_x;
                source(1, i) = beam.endpoint_y;





                double angle = 10.0 / 180.0 * M_PI;
                Eigen::Vector2d offset;
                offset << 0.5, 0.0;
                Eigen::Matrix2d rotation_matrix;
                rotation_matrix << std::cos(angle), -std::sin(angle),
                                std::sin(angle),  std::cos(angle);

                source.col(i) = (rotation_matrix * source.col(i)).colwise() + offset;





                beam_endpoint_cell = map_get_cell(map, beam.endpoint_x, beam.endpoint_y);
            }
            else
            {
                beam_endpoint_cell = map_get_cell(map, source(0, i), source(1, i));
            }

            if (beam_endpoint_cell != NULL)
            {
                if (is_inner_beam_list.at(i))
                {
                    target(0, i) = MAP_WXGX(map, beam_endpoint_cell->ind_nearest_inner_wall % map->size_x);
                    target(1, i) = MAP_WYGY(map, floor(beam_endpoint_cell->ind_nearest_inner_wall / map->size_x));
                }
                else // outer beam
                {
                    target(0, i) = MAP_WXGX(map, beam_endpoint_cell->ind_nearest_outer_wall % map->size_x);
                    target(1, i) = MAP_WYGY(map, floor(beam_endpoint_cell->ind_nearest_outer_wall / map->size_x));
                }
            }
            else // source coordinated not valid
            {
                target(0, i) = source(0, i);
                target(1, i) = source(1, i);
            }
        }

        if (iteration == 0)
        {
            best_fit = source;
            min_error = compute_error(source, target);
        }

        visualize(target, source);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        //TODO compute some metric for determining the fit

        // compute center of source and target
        Eigen::Vector2d source_center = get_center(source, exclude_indices);
        Eigen::Vector2d target_center = get_center(target, exclude_indices);

        // centralize source and target
        Eigen::Matrix2Xd source_centered = source.colwise() - source_center;
        Eigen::Matrix2Xd target_centered = target.colwise() - target_center;

        // compute norm of centered source
        Eigen::VectorXd source_norm = source_centered.colwise().squaredNorm();
        double norm_min = source_norm.minCoeff();
        double norm_max = source_norm.maxCoeff();

        // compute cross-covariance matrix
        Eigen::Matrix2d cov;

        for (int i = 0; i < N; i++)
        {
            if (source(0, i) == target(0, i) && source(1, i) == target(1, i))
            {
                continue;
            }

            Eigen::Vector2d p = source_centered.col(i);
            Eigen::Vector2d q = target_centered.col(i);

            // weight = kernel(p_point - q_point)
            double weight = 1.0;

            weight = 0.5;
            weight += 0.5 * (source_norm(i) - norm_min) / (norm_max - norm_min);

            cov += weight * q * p.transpose();
        }

        // compute SVD of cross-covariance matrix
        Eigen::JacobiSVD<Eigen::Matrix2d> svd(cov, Eigen::ComputeFullU | Eigen::ComputeFullV);

        // retrieve transformation
        //TODO do we need to transpose V?
        // Eigen::Matrix2d R = svd.matrixV() * svd.matrixU().transpose();
        Eigen::Matrix2d R = svd.matrixU() * svd.matrixV().transpose();

        if (R.determinant() < 0)
        {
            R.col(1) *= -1.;
        }

        translation = target_center - R * source_center;
        source = (R * source).colwise() + translation;

        double err = compute_error(source, target);

        if (err < min_error)
        {
            min_error = err;
            Eigen::Matrix2Xd tmp = source;
            best_fit = tmp;
        }
    }

    // store results / transformed beam endpoints
    for (int i = 0; i < N; i++)
    {
        beam_t *beam = &beams.at(i);
        beam->endpoint_x = best_fit(0, i);
        beam->endpoint_y = best_fit(1, i);
    }

    return beams;
}

std::vector<beam_t> ICP::compute_ls_icp(map_t *map, std::vector<beam_t> beams_orig, unsigned int iterations, geometry_msgs::Polygon *centerline)
{
    int N = beams_orig.size();
    std::vector<beam_t> beams = beams_orig; // deep copy?
    std::vector<int> exclude_indices;
    Eigen::Matrix2d rot = Eigen::Matrix2d::Zero();
    Eigen::Vector2d t = Eigen::Vector2d::Zero();
    Eigen::Vector3d x = Eigen::Vector3d::Zero();

    Eigen::Matrix2d best_rot = Eigen::Matrix2d::Identity();
    Eigen::Vector2d best_t = Eigen::Vector2d::Zero();

    Eigen::Matrix2Xd best_fit = Eigen::Matrix2Xd::Zero(2, N);
    double min_error = std::numeric_limits<double>::max();

    // retrieve source and target data points
    Eigen::Matrix2Xd source = Eigen::Matrix2Xd::Zero(2, N);
    Eigen::Matrix2Xd target = Eigen::Matrix2Xd::Zero(2, N);

    std::vector<bool> is_inner_beam_list;

    int max_inner = 0;
    int max_outer = 0;

    for (int i = 0; i < N; i++)
    {
        beam_t beam = beams.at(i);

        source(0, i) = beam.endpoint_x;
        source(1, i) = beam.endpoint_y;
    }

    // /**
    //  * Inroduce an error into the lidar scan for testing purposes
    //  * 
    //  */
    // Eigen::Vector2d source_center = get_center(source, exclude_indices);

    // for (int i = 0; i < N; i++)
    // {
    //     double angle = 5.0 / 180.0 * M_PI;
    //     Eigen::Vector2d offset;
    //     offset << 0.0, 0.7;
    //     Eigen::Matrix2d rotation_matrix;
    //     rotation_matrix << std::cos(angle), -std::sin(angle),
    //                     std::sin(angle),  std::cos(angle);

    //     source.col(i) = ((rotation_matrix * (source.col(i).colwise() - source_center)).colwise() + offset).colwise() + source_center;
    // }

    for (unsigned int iteration = 0; iteration < iterations; iteration++)
    {
        for (int i = 0; i < N; i++)
        {
            map_cell_t *endpoint_cell = map_get_cell(map, source(0, i), source(1, i));

            if (endpoint_cell == NULL)
            {
                if (iteration == 0)
                {
                    is_inner_beam_list.push_back(true);
                }
                else
                {
                    is_inner_beam_list.at(i) = true;
                }

                continue;
            }

            if (iteration == 0)
            {
                is_inner_beam_list.push_back(endpoint_cell->occ_dist_inner < endpoint_cell->occ_dist_outer);
            }
            else
            {
                is_inner_beam_list.at(i) = endpoint_cell->occ_dist_inner < endpoint_cell->occ_dist_outer;
            }
        }

        double diff = 0.0;
        int diff_count = 0;

        for (int i = 0; i < N; i++)
        {
            map_cell_t *beam_endpoint_cell = map_get_cell(map, source(0, i), source(1, i));

            if (beam_endpoint_cell != NULL)
            {
                if (is_inner_beam_list.at(i))
                {
                    target(0, i) = MAP_WXGX(map, beam_endpoint_cell->ind_nearest_inner_wall % map->size_x);
                    target(1, i) = MAP_WYGY(map, floor(beam_endpoint_cell->ind_nearest_inner_wall / map->size_x));
                }
                else // outer beam
                {
                    target(0, i) = MAP_WXGX(map, beam_endpoint_cell->ind_nearest_outer_wall % map->size_x);
                    target(1, i) = MAP_WYGY(map, floor(beam_endpoint_cell->ind_nearest_outer_wall / map->size_x));
                }

                // accumulate diff
                diff += (source.col(i) - target.col(i)).squaredNorm();
                diff_count++;
            }
            else // source coordinated not valid
            {
                target(0, i) = source(0, i);
                target(1, i) = source(1, i);
            }
        }

        double mean_diff = diff / (double) diff_count;

        if (iteration == 0) // now that the target is initialized we can also initialize the best fit
        {
            best_fit = source;
            min_error = compute_error(source, target);
        }

        // visualize(target, source);
        // std::this_thread::sleep_for(std::chrono::milliseconds(500));

        Eigen::Matrix<double, 3, 3> H = Eigen::Matrix3d::Zero();
        Eigen::Vector3d g = Eigen::Vector3d::Zero();

        double chi = 0.0;

        for (int i = 0; i < N; i++)
        {
            /**
             * compute error metric
             *
             */
            double theta = x(2);
            Eigen::Matrix2d rotation;
            rotation << std::cos(theta), -std::sin(theta),
                        std::sin(theta),  std::cos(theta);

            Eigen::Vector2d translation;
            translation << x(0), x(1);

            Eigen::Vector2d prediction = rotation * source.col(i) + translation;
            Eigen::Vector2d error = prediction - target.col(i);

            /**
             * compute weighing factor 1
             *
             */
            Eigen::Vector2d s1 = source.col(i);

            map_cell_t *cell_1 = map_get_cell(map, s1(0), s1(1));
            
            double weight = 1.0;
            double threshold = 0.4; // 40 cm

            // is the current point outside the racetrack?
            bool outside_racetrack = false;

            if ((is_inner_beam_list.at(i) && (map->cells + cell_1->ind_nearest_inner_wall)->occ_dist_outer <= cell_1->occ_dist_outer)
            || (!is_inner_beam_list.at(i) && (map->cells + cell_1->ind_nearest_outer_wall)->occ_dist_inner <= cell_1->occ_dist_inner))
            {
                outside_racetrack = true;
            }

            int dist = 0;

            if (is_inner_beam_list.at(i))
            {
                dist = cell_1->occ_dist_inner;
            }
            else
            {
                dist = cell_1->occ_dist_outer;
            }

            if (outside_racetrack && dist > 10)
            {
                weight = std::max(0.1, 1.0 - ((double) dist / 40.0));
            }
            else if (!outside_racetrack && dist > 5)
            {
                weight = std::max(0.1, 1.0 - ((double) dist / 20.0));
            }

            Eigen::Matrix2d dR0;
            dR0 << -std::sin(theta), -std::cos(theta),
                    std::cos(theta), -std::sin(theta);

            Eigen::Matrix<double, 2, 3> J = Eigen::Matrix<double, 2, 3>::Zero();
            J.block(0, 0, 2, 2) = Eigen::Matrix<double, 2, 2>::Identity();
            J.col(2) = dR0 * source.col(i);

            H += (J.transpose() * J) * weight;
            g += (J.transpose() * error) * weight;
            chi += error.transpose() * error;
        }

        Eigen::Vector3d dx = H.colPivHouseholderQr().solve(-g);

        x += dx;
        rot << std::cos(x(2)), -std::sin(x(2)),
                std::sin(x(2)),  std::cos(x(2));
        t << x(0), x(1);
        x(2) = std::atan2(std::sin(x(2)), std::cos(x(2)));
        source = (rot * source).colwise() + t;

        // double err = chi;
        double err = compute_error(source, target);

        if (err < min_error)
        {
            min_error = err;
            best_rot = rot;
            best_t = t;
        }
    }

    // store results / transformed beam endpoints
    for (int i = 0; i < N; i++)
    {
        beam_t *beam = &beams.at(i);

        //TODO transform - store best transform not best fit -> any runtime improvements?
        double epx = beam->endpoint_x;
        double epy = beam->endpoint_y;

        //TODO check why this does not work!!!
        double x = best_rot(0, 0) * epx + best_rot(0, 1) * epy + best_t(0);
        double y = best_rot(1, 0) * epx + best_rot(1, 1) * epy + best_t(1);

        // double x = rot(0, 0) * epx + rot(0, 1) * epy + t(0);
        // double y = rot(1, 0) * epx + rot(1, 1) * epy + t(1);
        beam->endpoint_x = x;
        beam->endpoint_y = y;
    }

    return beams;
}
