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

-- TRAJECTORY_BUILDER.pure_localization_trimmer = { 
--   max_submaps_to_keep = 3,
-- }

TRAJECTORY_BUILDER_2D.max_range = 25.0
TRAJECTORY_BUILDER_2D.min_range = 0.5 

-- might be able to optimize these parameters
-- see: http://google-cartographer-ros.readthedocs.io/en/latest/tuning.html

-- configure correct size of submaps
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 35 -- 100 
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 3

-- Tuning the CeresScanMatcher
--TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 1e2
--TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 1e2

-- 0 := turn off global SLAM to not mess with our tuning
-- POSE_GRAPH.optimize_every_n_nodes = 5 -- 20

-- TRAJECTORY_BUILDER.pure_localization_trimmer.max_submaps_to_keep = 3
POSE_GRAPH.optimize_every_n_nodes = 3

return options