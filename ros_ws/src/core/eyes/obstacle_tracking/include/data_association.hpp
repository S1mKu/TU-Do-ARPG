#pragma once

#include <vector>

#include "data_t.hpp"

std::vector<association_t> associate_static_obstacles(std::vector<Obstacle> source, std::vector<segment_t> target);

std::vector<association_t> associate_dynamic_obstacles(std::vector<Obstacle> source, std::vector<segment_t> target);
