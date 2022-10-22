/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
 *                      gerkey@usc.edu    kaspers@robotics.usc.edu
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/**************************************************************************
 * Desc: Global map (grid-based)
 * Author: Andrew Howard
 * Date: 6 Feb 2003
 * CVS: $Id: map.c 1713 2003-08-23 04:03:43Z inspectorg $
**************************************************************************/

#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <limits>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <ros/console.h>

#include "grid_map.hpp"

#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"

// using namespace SCAN_OBSTACLE_FILTER;

// Create a new map
map_t *map_alloc(void)
{
    map_t *map;

    map = (map_t*) malloc(sizeof(map_t));

    // Assume we start at (0, 0)
    map->origin_x = 0;
    map->origin_y = 0;

    // Make the size odd
    map->size_x = 0;
    map->size_y = 0;
    map->scale = 0;

    // Allocate storage for main map
    map->cells = (map_cell_t*) NULL;

    return map;
}

// Destroy a map
void map_free(map_t *map)
{
    free(map->cells);
    free(map);
}

void map_init(map_t *map, const nav_msgs::OccupancyGrid::ConstPtr& map_msg, geometry_msgs::Polygon *centerline, ros::Publisher rviz_pub)
{
    map->size_x = map_msg->info.width;
    map->size_y = map_msg->info.height;
    map->scale = map_msg->info.resolution;

    // store origin in global coordinates
    map->origin_x = map_msg->info.origin.position.x + (map->size_x / 2) * map->scale;
    map->origin_y = map_msg->info.origin.position.y + (map->size_y / 2) * map->scale;

    map->cells = (map_cell_t *) malloc(sizeof(map_cell_t) * map->size_x * map->size_y);

    // orig matrix - initialized to zero
    cv::Mat wall_mat = cv::Mat::zeros(map->size_x, map->size_y, CV_8U);

    for (int x = 0; x < map->size_x; x++)
    {
        for (int y = 0; y < map->size_y; y++)
        {
            int i = MAP_INDEX(map, x, y);

            // just a big enough number
            // do NOT use max int due to problems with overflow
            map->cells[i].occ_dist_inner = 999999999; // std::numeric_limits<int>::max();
            map->cells[i].occ_dist_outer = 999999999; // std::numeric_limits<int>::max();

            if (map_msg->data[i] == 0)
            {
                map->cells[i].occ_state = -1; // free 
            }
            else if (map_msg->data[i] == 100)
            {
                map->cells[i].occ_state = +1; // occupied 

                // check neighborhood
                int num_free_neighbors = 0;
                for (int m = -1; m <= 1; m++)
                {
                    for (int n = -1; n <= 1; n++)
                    {
                        if (MAP_VALID(map, x + m, y + n))
                        {
                            int ind = MAP_INDEX(map, x + m, y + n);

                            if (map_msg->data[ind] == 0)
                            {
                                num_free_neighbors++;
                                break; // since we check for num_free_neighbors > 0
                            }
                        }
                    }
                }

                if (num_free_neighbors > 0) // part of wall
                {
                    wall_mat.at<unsigned char>(x, y) = 255;
                }
            }
            else
            {
                map->cells[i].occ_state = 0; // unknown 
            }
        }
    }

    // compute polygon from centerline
    std::vector<cv::Point> centerline_poly;

    for (geometry_msgs::Point32 p : centerline->points)
    {
        centerline_poly.push_back(cv::Point(MAP_GXWX(map, p.x), MAP_GYWY(map, p.y)));
    }

    std::vector<std::vector<cv::Point>> all_polys;
    all_polys.push_back(centerline_poly);

    // matrix for inner racetrack boundary
    cv::Mat inner_wall_mat = wall_mat.clone();

    // matrix for outer racetrack boundary
    cv::Mat outer_wall_mat = wall_mat.clone();

    // compute inner and outer wall
    cv::Mat stencil = cv::Mat::zeros(map->size_x, map->size_y, CV_8U);

    fillPoly(stencil, all_polys, cv::Scalar(255), 16, 0);

    for (int x = map->size_x - 1; x >= 0 ; x--)
    {
        for (int y = 0; y < map->size_y; y++)
        {
            // apply stencil like a mask for outer and inner wall
            if (stencil.at<unsigned char>(y, x) > 0) //TODO check this
            {
                outer_wall_mat.at<unsigned char>(x, y) = 0;
            }
            else
            {
                inner_wall_mat.at<unsigned char>(x, y) = 0;
            }

            if (outer_wall_mat.at<unsigned char>(x, y) > 0)
            {
                int i = MAP_INDEX(map, x, y);
                map->cells[i].occ_dist_outer = 0;
                map->cells[i].ind_nearest_outer_wall = i;
                continue;
            }

            if (inner_wall_mat.at<unsigned char>(x, y) > 0)
            {
                int i = MAP_INDEX(map, x, y);
                map->cells[i].occ_dist_inner = 0;
                map->cells[i].ind_nearest_inner_wall = i;
                continue;
            }
        }
    }

    // forward scan - start at the top left corner and finish at the bottom right corner
    for (int y = map->size_y - 1; y >= 0; y--)
    {
        for (int x = 0; x < map->size_x; x++)
        {
            int i = MAP_INDEX(map, x, y);
            int i_left = MAP_INDEX(map, x - 1, y);
            int i_upper = MAP_INDEX(map, x, y + 1);

            // inner wall 
            if (x > 0 && map->cells[i].occ_dist_inner > map->cells[i_left].occ_dist_inner + 1)
            {
                map->cells[i].occ_dist_inner = map->cells[i_left].occ_dist_inner + 1;
                map->cells[i].ind_nearest_inner_wall = map->cells[i_left].ind_nearest_inner_wall;
            }

            if (y < map->size_y - 1 && map->cells[i].occ_dist_inner > map->cells[i_upper].occ_dist_inner + 1)
            {
                map->cells[i].occ_dist_inner = map->cells[i_upper].occ_dist_inner + 1;
                map->cells[i].ind_nearest_inner_wall = map->cells[i_upper].ind_nearest_inner_wall;
            }

            // outer wall
            if (x > 0 && map->cells[i].occ_dist_outer > map->cells[i_left].occ_dist_outer + 1)
            {
                map->cells[i].occ_dist_outer = map->cells[i_left].occ_dist_outer + 1;
                map->cells[i].ind_nearest_outer_wall = map->cells[i_left].ind_nearest_outer_wall;
            }

            if (y < map->size_y - 1 && map->cells[i].occ_dist_outer > map->cells[i_upper].occ_dist_outer + 1)
            {
                map->cells[i].occ_dist_outer = map->cells[i_upper].occ_dist_outer + 1;
                map->cells[i].ind_nearest_outer_wall = map->cells[i_upper].ind_nearest_outer_wall;
            }
        }
    }

    double max_occ_dist = 0.0;

    // backward scan - start at the bottom right corner and finish at the top left corner
    for (int x = map->size_x - 1; x >= 0 ; x--)
    {
        for (int y = 0; y < map->size_y; y++)
        {
            int i = MAP_INDEX(map, x, y);
            int i_right = MAP_INDEX(map, x + 1, y);
            int i_lower = MAP_INDEX(map, x, y - 1);

            // inner wall 
            if (x < map->size_x - 1 && map->cells[i].occ_dist_inner > map->cells[i_right].occ_dist_inner + 1)
            {
                map->cells[i].occ_dist_inner = map->cells[i_right].occ_dist_inner + 1;
                map->cells[i].ind_nearest_inner_wall = map->cells[i_right].ind_nearest_inner_wall;
            }

            if (y > 0 && map->cells[i].occ_dist_inner > map->cells[i_lower].occ_dist_inner + 1)
            {
                map->cells[i].occ_dist_inner = map->cells[i_lower].occ_dist_inner + 1;
                map->cells[i].ind_nearest_inner_wall = map->cells[i_lower].ind_nearest_inner_wall;
            }

            if (map->cells[i].occ_dist_inner > max_occ_dist)
            {
                max_occ_dist = map->cells[i].occ_dist_inner;
            }
            
            // outer wall
            if (x < map->size_x - 1 && map->cells[i].occ_dist_outer > map->cells[i_right].occ_dist_outer + 1)
            {
                map->cells[i].occ_dist_outer = map->cells[i_right].occ_dist_outer + 1;
                map->cells[i].ind_nearest_outer_wall = map->cells[i_right].ind_nearest_outer_wall;
            }

            if (y > 0 && map->cells[i].occ_dist_outer > map->cells[i_lower].occ_dist_outer + 1)
            {
                map->cells[i].occ_dist_outer = map->cells[i_lower].occ_dist_outer + 1;
                map->cells[i].ind_nearest_outer_wall = map->cells[i_lower].ind_nearest_outer_wall;
            }

            if (map->cells[i].occ_dist_outer > max_occ_dist)
            {
                max_occ_dist = map->cells[i].occ_dist_outer;
            }
        }
    }

    // cv::Mat m_inner = cv::Mat::zeros(map->size_x, map->size_y, CV_64FC1);
    // cv::Mat m_outer = cv::Mat::zeros(map->size_x, map->size_y, CV_64FC1);

    // for (int x = map->size_x - 1; x >= 0 ; x--)
    // {
    //     for (int y = 0; y < map->size_y; y++)
    //     {
    //         int i = MAP_INDEX(map, x, y);

    //         m_inner.at<double>(x, y) = map->cells[i].occ_dist_inner;
    //         m_outer.at<double>(x, y) = map->cells[i].occ_dist_outer;
    //     }
    // }

    // cv::normalize(m_inner, m_inner, 0, 1.0, cv::NORM_MINMAX);
    // cv::normalize(m_outer, m_outer, 0, 1.0, cv::NORM_MINMAX);

    // cv::resize(m_inner, m_inner, cv::Size(m_inner.cols/2, m_inner.rows/2));
    // cv::resize(m_outer, m_outer, cv::Size(m_outer.cols/2, m_outer.rows/2));

    // cv::rotate(m_inner, m_inner, cv::ROTATE_90_COUNTERCLOCKWISE);
    // cv::rotate(m_outer, m_outer, cv::ROTATE_90_COUNTERCLOCKWISE);

    // cv::namedWindow("Inner", cv::WINDOW_NORMAL);
    // cv::imshow("Inner", m_inner);

    // cv::namedWindow("Outer", cv::WINDOW_NORMAL);
    // cv::imshow("Outer", m_outer);

    // cv::waitKey();

    map->max_occ_dist = max_occ_dist;
}

// Get the cell at the given point
map_cell_t *map_get_cell(map_t *map, double ox, double oy)
{
    int i, j;
    map_cell_t *cell;

    i = MAP_GXWX(map, ox);
    j = MAP_GYWY(map, oy);

    if (!MAP_VALID(map, i, j))
        return NULL;

    cell = map->cells + MAP_INDEX(map, i, j);
    return cell;
}
