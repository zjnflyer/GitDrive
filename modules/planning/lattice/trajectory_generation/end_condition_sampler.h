/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file
 **/

#ifndef MODULES_PLANNING_LATTICE_TRAJECTORY_GENERATION_END_CONDITION_SAMPLER_H_
#define MODULES_PLANNING_LATTICE_TRAJECTORY_GENERATION_END_CONDITION_SAMPLER_H_

#include <array>
#include <utility>
#include <vector>
#include <memory>
#include <string>

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/planning/common/path_decision.h"
#include "modules/planning/lattice/behavior/feasible_region.h"
#include "modules/planning/lattice/behavior/path_time_graph.h"
#include "modules/planning/lattice/behavior/prediction_querier.h"
#include "modules/planning/proto/lattice_structure.pb.h"

namespace apollo {
namespace planning {

// Input: planning objective, vehicle kinematic/dynamic constraints,
// Output: sampled ending 1 dimensional states with corresponding time duration.
class EndConditionSampler {
 public:
  EndConditionSampler(
      const std::array<double, 3>& init_s,
      const std::array<double, 3>& init_d,
      PathDecision* path_decision,
      const ReferenceLineInfo* ptr_reference_line_info,
      std::shared_ptr<PathTimeGraph> ptr_path_time_graph,
      std::shared_ptr<PredictionQuerier> ptr_prediction_querier,
      const PlanningConfig& config);

  virtual ~EndConditionSampler() = default;

  std::vector<std::pair<std::array<double, 3>, double>>
  SampleLatEndConditions() const;

  std::vector<std::pair<std::array<double, 3>, double>>
  SampleLonEndConditionsForCruising(const double ref_cruise_speed) const;

  std::vector<std::pair<std::array<double, 3>, double>>
  SampleLonEndConditionsForStopping(const double ref_stop_point) const;

  std::vector<std::pair<std::array<double, 3>, double>>
  SampleLonEndConditionsForPathTimePoints() const;
  
  void Init(const PlanningConfig& config);

 private:
  std::vector<SamplePoint> QueryPathTimeObstacleSamplePoints() const;

  void QueryFollowPathTimePoints(
      const apollo::common::VehicleConfig& vehicle_config,
      const std::string& obstacle_id,
      std::vector<SamplePoint>* sample_points) const;

  void QueryOvertakePathTimePoints(
      const apollo::common::VehicleConfig& vehicle_config,
      const std::string& obstacle_id,
      std::vector<SamplePoint>* sample_points) const;

 private:
  std::array<double, 3> init_s_;
  std::array<double, 3> init_d_;
  FeasibleRegion feasible_region_;
  PathDecision* path_decision_;
  const ReferenceLineInfo* ptr_reference_line_info_;
  std::shared_ptr<PathTimeGraph> ptr_path_time_graph_;
  std::shared_ptr<PredictionQuerier> ptr_prediction_querier_;

  //calibration parameters
  double LatEnd_d1 = 0.0;
  double LatEnd_d2 = 0.0;
  double LatEnd_d3 = 0.0;
  double LatEnd_d4 = 0.0;
  double LatEnd_d5 = 0.0;
  double LatEnd_d6 = 0.0;
  double LatEnd_d7 = 0.0;

  double LatEnd_s1 = 0.0;
  double LatEnd_s2 = 0.0;
  double LatEnd_s3 = 0.0;
  double LatEnd_s4 = 0.0;
  double LatEnd_s5 = 0.0;
  double LatEnd_s6 = 0.0;

};

}  // namespace planning
}  // namespace apollo

#endif
// MODULES_PLANNING_LATTICE_TRAJECTORY_GENERATION_END_CONDITION_SAMPLER_H_
