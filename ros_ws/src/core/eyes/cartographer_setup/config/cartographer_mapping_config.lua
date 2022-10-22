-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

-- Modified by the F1/10 Autonomous Racing Project Group

include "cartographer_config.lua"

-- might be able to optimize these parameters
-- see: http://google-cartographer-ros.readthedocs.io/en/latest/tuning.html

-- configure correct size of submaps
--  - the accumulated odometry error in each submap should be small, otherwise the submaps get deformed
--  - the number of subnmaps should be small, otherwise the backend might deform them 
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 14 -- 100 
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 100

-- Tuning the CeresScanMatcher
-- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 1e3
-- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 1e3

-- Tuning the global SLAM with odometry
POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e2
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1e2
POSE_GRAPH.optimization_problem.odometry_translation_weight = 4e7
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 4e7

-- 0 := turn off global SLAM to not mess with our tuning
POSE_GRAPH.optimize_every_n_nodes = 3 -- 20

return options