#pragma once

#include <stdint.h>

#include <ros/ros.h>

#include "stddef.h"

#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Polygon.h"

// namespace SCAN_OBSTACLE_FILTER 
// {

// Description for a single map cell.
typedef struct
{
  // Occupancy state (-1 = free, 0 = unknown, +1 = occ)
  char occ_state;

  // Distance to the nearest occupied cell
  // unsigned char occ_dist;
  int occ_dist_inner;
  int occ_dist_outer;

  int ind_nearest_inner_wall;
  int ind_nearest_outer_wall;

  // Wifi levels
  //int wifi_levels[MAP_WIFI_MAX_LEVELS];

} map_cell_t;


// Description for a map
typedef struct
{
  // Map origin; the map is a viewport onto a conceptual larger map.
  double origin_x, origin_y;
  
  // Map scale (m/cell)
  double scale;

  // Map dimensions (number of cells)
  int size_x, size_y;
  
  // The map data, stored as a grid
  map_cell_t *cells;

  // Max distance at which we care about obstacles, for constructing
  // likelihood field
  double max_occ_dist;
  
} map_t;


/**************************************************************************
 * Basic map functions
 **************************************************************************/

// Create a new (empty) map
map_t *map_alloc(void);

// initialize a map with the contents of the OccupancyGrid msg
void map_init(map_t *map, const nav_msgs::OccupancyGrid::ConstPtr& map_msg, geometry_msgs::Polygon *centerline, ros::Publisher rviz_pub);

// Destroy a map
void map_free(map_t *map);

// Get the cell at the given point
map_cell_t *map_get_cell(map_t *map, double ox, double oy);

/**************************************************************************
 * Map manipulation macros
 **************************************************************************/

// map_msg->info.origin.position.x + (map->size_x / 2) * map->scale;

// Convert from map index to world coords
#define MAP_WXGX(map, i) (map->origin_x + ((i) - map->size_x / 2) * map->scale)
#define MAP_WYGY(map, j) (map->origin_y + ((j) - map->size_y / 2) * map->scale)

// Convert from world coords to map coords
#define MAP_GXWX(map, x) (floor((x - map->origin_x) / map->scale + 0.5) + map->size_x / 2)
#define MAP_GYWY(map, y) (floor((y - map->origin_y) / map->scale + 0.5) + map->size_y / 2)

// Test to see if the given map coords lie within the absolute map bounds.
#define MAP_VALID(map, i, j) ((i >= 0) && (i < map->size_x) && (j >= 0) && (j < map->size_y))

// Compute the cell index for the given map coords.
#define MAP_INDEX(map, i, j) ((i) + (j) * map->size_x)

// }
