#include "state_estimation.hpp"

#include <Eigen/Dense>

#include "data_t.hpp"
#include "util.hpp"
#include "spline/spline.hpp"

StateEstimation::StateEstimation(ros::NodeHandle nh) : node_handle_(nh), icp(nh) { }

void StateEstimation::updateObstacle(Obstacle& obstacle, segment_t& segment, double dt)
{
    obstacle.segment_backlog.push_back(segment);
    int num_pos = obstacle.position_backlog.size();

    if (num_pos < 2)
    {
        /**
         * Update position
         * 
         */
        Point new_pos = mean(segment.points);
        obstacle.position_backlog.push_back(new_pos);

        if (num_pos == 0)
        {
            // this should not happen, there should always be a theta for each position and segment since initialiting the obstacle
            double theta = 0;
            obstacle.theta_backlog.push_back(theta);
        }
        else // num_pos == 1
        {
            assert(obstacle.theta_backlog.size() == 1);
            assert(obstacle.position_backlog.size() == 2);
            assert(obstacle.segment_backlog.size() == 2);

            Point p1 = obstacle.position_backlog.at(0);
            Point p2 = obstacle.position_backlog.at(1);
            double norm = std::sqrt(std::pow(p1.first - p2.first, 2.0) + std::pow(p1.second - p2.second, 2.0));
            double theta = std::atan2((p2.second - p1.second) / norm, (p2.first - p1.first) / norm);

            obstacle.theta_backlog.at(0) = theta;
            obstacle.theta_backlog.push_back(theta);
        }
    }
    else
    {
        // /**
        //  * Update position from icp translation
        //  * 
        //  */
        // segment_t seg_source = obstacle.segment_backlog.at(num_pos - 1);
        // segment_t seg_target = segment;

        // Eigen::Matrix2Xd staticData = Eigen::Matrix2Xd::Zero(2, seg_target.points.size());
        // Eigen::Matrix2Xd fitData = Eigen::Matrix2Xd::Zero(2, seg_source.points.size());
        
        // for (int i = 0; i < seg_target.points.size(); i++)
        // {
        //     Point p = seg_target.points.at(i);
        //     staticData(0, i) = p.first;
        //     staticData(1, i) = p.second;
        // }

        // for (int i = 0; i < seg_source.points.size(); i++)
        // {
        //     Point p = seg_source.points.at(i);
        //     fitData(0, i) = p.first;
        //     fitData(1, i) = p.second;
        // }

        // ROS_WARN_STREAM("computeICPTranslation: " << seg_target.points.size() << " " << staticData.cols() << " - " << seg_source.points.size() << " " << fitData.cols());

        // Eigen::Vector2d v = icp.compute_svd_icp(staticData, fitData, 20);

        // Point old_pos = obstacle.position_backlog.back();
        // Point predicted_pos;
        // predicted_pos.first = old_pos.first + v(0);
        // predicted_pos.second = old_pos.second + v(1);

        //TODO include predicted pose

        Point mean_pos = mean(segment.points);
        Point final_pos;
        final_pos.first = (mean_pos.first); // + predicted_pos.first) / 2.0;
        final_pos.second = (mean_pos.second); // + predicted_pos.second) / 2.0;

        obstacle.position_backlog.push_back(final_pos);

        /**
         * estimate theta/orientation
         * 
         * spline.h requires at least three points
         * 
         */
        int n = obstacle.position_backlog.size();

        std::vector<double> X;
        std::vector<double> Y;

        for (int i = 0; i < n; i++)
        {
            Point p = obstacle.position_backlog[i];
            X.push_back(p.first);
            Y.push_back(p.second);
        }

        tk::spline::spline_type type = tk::spline::cspline;
        bool make_monotonic = false;
        bool is_closed_curve = false;

        double tmin = 0.0, tmax = 0.0;
        std::vector<double> T;

        int idx_first = -1, idx_last = -1;

        // setup a "time variable" so that we can interpolate x and y
        // coordinates as a function of time: (X(t), Y(t))
        T.resize(X.size());
        T[0] = 0.0;

        for (size_t i = 1; i < T.size(); i++)
        {
            // time is proportional to the distance, i.e. we go at a const speed
            double dist = std::sqrt(std::pow(X.at(i) - X.at(i-1), 2.0) + std::pow(Y.at(i) - Y.at(i-1), 2.0));
            T.at(i) = T.at(i-1) + dist;

            // ROS_WARN_STREAM("T" << i << " " << T.at(i) << " " << std::pow(X.at(i) - X.at(i-1), 2.0) << " " << std::pow(Y.at(i) - Y.at(i-1), 2.0) << " " << std::pow(Y.at(i) - Y.at(i-1), 2.0) << " " << dist);
        }

        if (idx_first < 0 || idx_last < 0)
        {
            tmin = T[0] - 0.0;
            tmax = T.back() + 0.0;
        } else {
            tmin = T[idx_first];
            tmax = T[idx_last];
        }

        // define a spline for each coordinate x, y
        tk::spline sx, sy;
        sx.set_boundary(tk::spline::second_deriv, 0.0,
                        tk::spline::second_deriv, 0.0);
        sy.set_boundary(tk::spline::second_deriv, 0.0,
                        tk::spline::second_deriv, 0.0);
        sx.set_points(T, X, type);
        sy.set_points(T, Y, type);

        double t = tmax; // tmin + (double) (tmax - tmin);
        double dx = sx.deriv(1, t);
        double dy = sy.deriv(1, t);
        double norm = std::pow(std::pow(dx, 2.0) + std::pow(dy, 2.0), 0.5);
        dx = dx / norm;
        dy = dy / norm;

        int num_positions = obstacle.position_backlog.size();
        Point p1 = obstacle.position_backlog.back();
        Point p2 = obstacle.position_backlog.at(num_positions - 2);
        norm = std::sqrt(std::pow(p1.first - p2.first, 2.0) + std::pow(p1.second - p2.second, 2.0));
        double theta = std::atan2((p1.second - p2.second) / norm, (p1.first - p2.first) / norm);

        obstacle.theta_backlog.push_back(theta);
    }

    /**
     * Update velocity
     * 
     */
    int num_positions = obstacle.position_backlog.size();
    Point last_pos = obstacle.position_backlog.back();
    Point current_pos = obstacle.position_backlog.at(num_positions - 2);
    Point diff;
    diff.first = last_pos.first - current_pos.first;
    diff.second = last_pos.second - current_pos.second;

    double velocity = std::pow(std::pow(diff.first, 2.0) + std::pow(diff.second, 2.0), 0.5) / dt;
    obstacle.velocity = velocity;
}

