/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "modules/planning/toolkits/deciders/front_vehicle.h"

#include <algorithm>
#include <string>
#include <vector>
#include <limits>

#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/planning/proto/planning_status.pb.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/time/time.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/planning/common/ego_info.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::Status;
using apollo::common::VehicleConfigHelper;
using apollo::common::time::Clock;
using apollo::hdmap::HDMapUtil;
using apollo::perception::PerceptionObstacle;

FrontVehicle::FrontVehicle(const TrafficRuleConfig& config)
    : TrafficRule(config) {}

Status FrontVehicle::ApplyRule(Frame* const frame,
                               ReferenceLineInfo* const reference_line_info) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  AINFO << "............ Traffic Rule (front_vehicle) Started ............";

  MakeDecisions(frame, reference_line_info);

  AINFO << "............ Traffic Rule (front_vehicle) Ended ............";

  return Status::OK();
}

/**
 * @brief: make decision
 */
void FrontVehicle::MakeDecisions(Frame* frame,
                                 ReferenceLineInfo* const reference_line_info) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  // JZ Added, if ADC is not on this lane, skip front vehicle rule on this reference line
  const auto& adc_sl_boundary = reference_line_info->AdcSlBoundary();
  const auto& reference_line = reference_line_info->reference_line();
  double k_rule_disable_l_threshold = config_.front_vehicle().rule_disable_l_threshold();
  if (adc_sl_boundary.start_l() > k_rule_disable_l_threshold ||
      adc_sl_boundary.end_l() < -k_rule_disable_l_threshold) {
    AINFO << "ATTENTION: ADC is leaving this reference line, skip front_vehicle rule";
    return;
  }

  MakeSidePassDecision(reference_line_info);

  MakeStopDecision(reference_line_info);
}

/**
 * @brief: make SIDEPASS decision
 */
bool FrontVehicle::MakeSidePassDecision(
    ReferenceLineInfo* const reference_line_info) {
  CHECK_NOTNULL(reference_line_info);

  if (!config_.front_vehicle().enable_side_pass()) {
    AINFO << "MakeSidePassDecision interrupted, sidepass disabled in traffic rule";
    return true;
  }

  if (!reference_line_info->Lanes().IsOnSegment()) {
    // The lane keeping reference line
    AINFO << "MakeSidePassDecision interrupted, IsOnSegment returned false";
    return true;
  }

  if (!ProcessSidePass(reference_line_info)) {
    AINFO << "MakeSidePassDecision interrupted, ProcessSidePass returned false";
    return false;
  }

  auto* sidepass_status = GetPlanningStatus()->mutable_side_pass();
  if (sidepass_status->has_status() &&
      sidepass_status->status() == SidePassStatus::SIDEPASS) {
    ADEBUG << "SIDEPASS: obstacle[" << sidepass_status->pass_obstacle_id()
           << "]";
    ObjectDecisionType sidepass;
    auto sidepass_decision = sidepass.mutable_sidepass();
    sidepass_decision->set_type(sidepass_status->pass_side());

    auto* path_decision = reference_line_info->path_decision();
    path_decision->AddLateralDecision(
        "front_vehicle", sidepass_status->pass_obstacle_id(), sidepass);
    AINFO << "Lateral Decision 'SIDEPASS' added for: obstacle[" << sidepass_status->pass_obstacle_id()
           << "]";
  }

  return true;
}

bool FrontVehicle::ProcessSidePass(
    ReferenceLineInfo* const reference_line_info) {
  CHECK_NOTNULL(reference_line_info);

  // find obstacle being blocked, to process SIDEPASS
  std::string passable_obstacle_id = FindPassableObstacle(reference_line_info);
  auto* sidepass_status = GetPlanningStatus()->mutable_side_pass();

  // JZ Added - check if there is passable obstacle, if false then return directly
  if (passable_obstacle_id.empty()) {
    AINFO << "no passable obstacle found on this reference line";
    sidepass_status->set_status(SidePassStatus::UNKNOWN);
    return false;
  }

  if (!sidepass_status->has_status()) {
    sidepass_status->set_status(SidePassStatus::UNKNOWN);
  }

  // JZ Added - check if the stored pass_obstacle_id is different from the current passable obstacle id
  // if YES then set status to UNKNOWN once
  if (sidepass_status->pass_obstacle_id()!=passable_obstacle_id) {
    sidepass_status->set_status(SidePassStatus::UNKNOWN);
    sidepass_status->set_pass_obstacle_id(passable_obstacle_id);
  }

  auto status = sidepass_status->status();
  ADEBUG << "side_pass status: " << SidePassStatus_Status_Name(status);
  AINFO << "side_pass status: " << SidePassStatus_Status_Name(status);

  switch (status) {
    case SidePassStatus::UNKNOWN: {
      sidepass_status->set_status(SidePassStatus::DRIVE);
      break;
    }
    case SidePassStatus::DRIVE: {
      const double kAdcStopSpeedThreshold = config_.front_vehicle().adc_stop_speed_threshold();  // JZ_Added unit: m/s
      const auto& adc_planning_point = EgoInfo::instance()->start_point();
      if (!passable_obstacle_id.empty() &&
          adc_planning_point.v() < kAdcStopSpeedThreshold) {
        sidepass_status->set_status(SidePassStatus::WAIT);
        sidepass_status->set_wait_start_time(Clock::NowInSeconds());
      }
      break;
    }
    case SidePassStatus::WAIT: {
      const auto& reference_line = reference_line_info->reference_line();
      const auto& adc_sl_boundary = reference_line_info->AdcSlBoundary();

      if (passable_obstacle_id.empty()) {
        sidepass_status->set_status(SidePassStatus::DRIVE);
        sidepass_status->clear_wait_start_time();
      } else {
        const double wait_start_time = sidepass_status->wait_start_time();
        const double wait_time = Clock::NowInSeconds() - wait_start_time;
        ADEBUG << "wait_start_time[" << wait_start_time << "] wait_time["
               << wait_time << "]";

        if (wait_time > config_.front_vehicle().side_pass_wait_time()) {
          // calculate if the left/right lane exist
          std::vector<hdmap::LaneInfoConstPtr> lanes;
          const double adc_s =
              (adc_sl_boundary.start_s() + adc_sl_boundary.end_s()) / 2.0;
          reference_line.GetLaneFromS(adc_s, &lanes);
          if (lanes.empty()) {
            AWARN << "No valid lane found at s[" << adc_s << "]";
            AINFO << "No valid lane found at s[" << adc_s << "]";
            return false;
          }

          bool enter_sidepass_mode = false;
          ObjectSidePass::Type side = ObjectSidePass::LEFT;
          if (lanes.size() >= 2) {
            // currently do not sidepass when lanes > 2 (usually at junctions).
          } else {
            sidepass_status->set_status(SidePassStatus::DRIVE);
            sidepass_status->clear_wait_start_time();

            auto& lane = lanes.front()->lane();
            if (lane.left_neighbor_forward_lane_id_size() > 0) {
              enter_sidepass_mode = true;
              side = ObjectSidePass::LEFT;
              AINFO << "sidepass type set to 'FORWARD LEFT'";
            }
            if (!enter_sidepass_mode &&
                lane.right_neighbor_forward_lane_id_size() > 0) {
              bool has_city_driving = false;
              for (auto& id : lane.right_neighbor_forward_lane_id()) {
                if (HDMapUtil::BaseMap().GetLaneById(id)->lane().type() ==
                    hdmap::Lane::CITY_DRIVING) {
                  has_city_driving = true;
                  break;
                }
              }
              if (has_city_driving) {
                enter_sidepass_mode = true;
                side = ObjectSidePass::RIGHT;
                AINFO << "sidepass type set to 'FORWARD RIGHT'";
              }
            }
            if (!enter_sidepass_mode &&
                lane.left_neighbor_reverse_lane_id_size() > 0) {
              enter_sidepass_mode = true;
              side = ObjectSidePass::LEFT;
              AINFO << "sidepass type set to 'REVERSE LEFT'";
            }
            if (!enter_sidepass_mode &&
                lane.right_neighbor_reverse_lane_id_size() > 0) {
              enter_sidepass_mode = true;
              side = ObjectSidePass::RIGHT;
              AINFO << "sidepass type set to 'REVERSE RIGHT'";
            }
          }
          if (enter_sidepass_mode) {
            sidepass_status->set_status(SidePassStatus::SIDEPASS);
            sidepass_status->set_pass_obstacle_id(passable_obstacle_id);
            sidepass_status->clear_wait_start_time();
            sidepass_status->set_pass_side(side);
          } else {
            AINFO << "No left or right lane found to execute side pass";
          }
        }
      }
      break;
    }
    case SidePassStatus::SIDEPASS: {
      if (passable_obstacle_id.empty()) {
        sidepass_status->set_status(SidePassStatus::DRIVE);
      }
      break;
    }
    default:
      break;
  }
  return true;
}

/**
 * @brief: a blocked obstacle is a static obstacle being blocked by
 *         other obstacles or traffic rules => not passable
 */
std::string FrontVehicle::FindPassableObstacle(
    ReferenceLineInfo* const reference_line_info) {
  CHECK_NOTNULL(reference_line_info);

  std::string passable_obstacle_id;
  const auto& adc_sl_boundary = reference_line_info->AdcSlBoundary();
  auto* path_decision = reference_line_info->path_decision();

  double k_sidepass_keep_range = config_.front_vehicle().sidepass_keep_range();

  for (const auto* path_obstacle : path_decision->path_obstacles().Items()) {
    const PerceptionObstacle& perception_obstacle =
        path_obstacle->obstacle()->Perception();
    const std::string& obstacle_id = std::to_string(perception_obstacle.id());
    PerceptionObstacle::Type obstacle_type = perception_obstacle.type();
    std::string obstacle_type_name =
        PerceptionObstacle_Type_Name(obstacle_type);

    if (path_obstacle->obstacle()->IsVirtual() ||
        !path_obstacle->obstacle()->IsStatic()) {
      ADEBUG << "obstacle_id[" << obstacle_id << "] type[" << obstacle_type_name
             << "] VIRTUAL or NOT STATIC. SKIP";
      AINFO << "obstacle_id[" << obstacle_id << "] type[" << obstacle_type_name
             << "] VIRTUAL or NOT STATIC. SKIP";
      continue;
    }

// JZ Added - original "if (obstacle_sl.start_s() <= adc_sl_boundary.end_s()) "
// probably a bug, obstacle behind ADC => obs.end_s < adc.start_s

    const auto& obstacle_sl = path_obstacle->PerceptionSLBoundary();
    if (obstacle_sl.end_s() <= adc_sl_boundary.start_s() - k_sidepass_keep_range) {
      ADEBUG << "obstacle_id[" << obstacle_id << "] type[" << obstacle_type_name
             << "] behind ADC. SKIP";
      AINFO << "obstacle_id[" << obstacle_id << "] type[" << obstacle_type_name
             << "] behind ADC. SKIP";
      continue;
    }

    const double side_pass_s_threshold =
        config_.front_vehicle().side_pass_s_threshold();
    if (obstacle_sl.start_s() - adc_sl_boundary.end_s() >
        side_pass_s_threshold) {
      ADEBUG << "obstacle_id[" << obstacle_id << "] type[" << obstacle_type_name
             << "] outside of s_threshold. SKIP";
      AINFO << "obstacle_id[" << obstacle_id << "] type[" << obstacle_type_name
             << "] outside of s_threshold. SKIP";
      continue;
    }

    const double side_pass_l_threshold =
        config_.front_vehicle().side_pass_l_threshold();
    if (obstacle_sl.start_l() > side_pass_l_threshold ||
        obstacle_sl.end_l() < -side_pass_l_threshold) {
      ADEBUG << "obstacle_id[" << obstacle_id << "] type[" << obstacle_type_name
             << "] outside of l_threshold. SKIP";
      AINFO << "obstacle_id[" << obstacle_id << "] type[" << obstacle_type_name
             << "] outside of l_threshold. SKIP";
      AINFO << "obstacle start_l: " << obstacle_sl.start_l();      
      AINFO << "obstacle end_l: " << obstacle_sl.end_l();
      continue;
    }

    bool is_blocked_by_others = false;
    for (const auto* other_obstacle : path_decision->path_obstacles().Items()) {
      if (other_obstacle->Id() == path_obstacle->Id()) {
        continue;
      }
      if (other_obstacle->PerceptionSLBoundary().start_l() >
              obstacle_sl.end_l() ||
          other_obstacle->PerceptionSLBoundary().end_l() <
              obstacle_sl.start_l()) {
        // not blocking the backside vehicle
        continue;
      }

      const double delta_s = other_obstacle->PerceptionSLBoundary().start_s() -
                             obstacle_sl.end_s();
      if (delta_s < 0.0 || delta_s > side_pass_s_threshold) {
        continue;
      } else {
        // TODO(All): fixed the segmentation bug for large vehicles, otherwise
        // the follow line will be problematic.
        // is_blocked_by_others = true; break;
      }
    }
    if (!is_blocked_by_others) {
      passable_obstacle_id = path_obstacle->Id();
      AINFO << "passable obstacle found: " << passable_obstacle_id;
      AINFO << "distance to EGO car: " << obstacle_sl.start_s() - adc_sl_boundary.end_s();
      break;
    }
  }
  return passable_obstacle_id;
}

void FrontVehicle::MakeStopDecision(ReferenceLineInfo* reference_line_info) {
  const auto& adc_sl = reference_line_info->AdcSlBoundary();
  auto* path_decision = reference_line_info->path_decision();
  const auto& reference_line = reference_line_info->reference_line();
  const auto& vehicle_param = VehicleConfigHelper::GetConfig().vehicle_param();
  const double adc_width = vehicle_param.width();


  // JZ Added - record the sidepass obstacle_sl.start_s as sidepass point
  // any obstacles within sidepass_range from the sidepass point 
  // will be skipped for stop decision
  double sidepass_point_s = -std::numeric_limits<double>::infinity();
  double k_sidepass_range = config_.front_vehicle().side_pass_range();
  double k_sidepass_l_buffer = config_.front_vehicle().sidepass_l_buffer();

  for (const auto* path_obstacle : path_decision->path_obstacles().Items()) {
    const PerceptionObstacle& perception_obstacle =
        path_obstacle->obstacle()->Perception();
    const std::string& obstacle_id = std::to_string(perception_obstacle.id());
    PerceptionObstacle::Type obstacle_type = perception_obstacle.type();
    std::string obstacle_type_name =
        PerceptionObstacle_Type_Name(obstacle_type);

    if (path_obstacle->obstacle()->IsVirtual() ||
        !path_obstacle->obstacle()->IsStatic()) {
      ADEBUG << "obstacle_id[" << obstacle_id << "] type[" << obstacle_type_name
             << "] VIRTUAL or NOT STATIC. SKIP";
      AINFO << "obstacle_id[" << obstacle_id << "] type[" << obstacle_type_name
             << "] VIRTUAL or NOT STATIC. SKIP";

      continue;
    }

    bool is_on_road = reference_line_info->reference_line().HasOverlap(
        path_obstacle->obstacle()->PerceptionBoundingBox());
    double length = reference_line_info->reference_line().Length();
    if (!is_on_road) {
      // skip obstacles not on reference line
      ADEBUG << "obstacle_id[" << obstacle_id << "] type[" << obstacle_type_name
             << "] NOT_ON_ROAD. SKIP";
      AINFO << "obstacle_id[" << obstacle_id << "] type[" << obstacle_type_name
             << "] NOT_ON_ROAD. SKIP";
      continue;
    }

    const auto& vehicle_sl = reference_line_info->VehicleSlBoundary();
    const auto& obstacle_sl = path_obstacle->PerceptionSLBoundary();
    if (obstacle_sl.end_s() <= adc_sl.start_s() ||
        obstacle_sl.end_s() <= vehicle_sl.start_s()) {
      // skip backside vehicles
      ADEBUG << "obstacle_id[" << obstacle_id << "] type[" << obstacle_type_name
             << "] behind ADC. SKIP";
      AINFO << "obstacle_id[" << obstacle_id << "] type[" << obstacle_type_name
             << "] behind ADC. SKIP";

      continue;
    }

    // JZ Added - skip making stop decision if obstacle is not on ADC's current path
    if (obstacle_sl.end_l() < adc_sl.start_l() - k_sidepass_l_buffer ||
        obstacle_sl.start_l() > adc_sl.end_l() + k_sidepass_l_buffer) {
        AINFO << "obstacle_id[" << obstacle_id << "] type[" << obstacle_type_name
              << "] outside of l_buffer, not on ADC path. SKIP";
        continue;
    }

    // check SIDE_PASS decision
    if (path_obstacle->LateralDecision().has_sidepass()) {
      sidepass_point_s = obstacle_sl.start_s();
      ADEBUG << "obstacle_id[" << obstacle_id << "] type[" << obstacle_type_name
             << "] SIDE_PASS. SKIP";
      AINFO << "obstacle_id[" << obstacle_id << "] type[" << obstacle_type_name
             << "] SIDE_PASS. SKIP. sidepass_point_s set to: " << sidepass_point_s;
      continue;
    }

    // JZ Added - skip the obstacles that are within sidepass range 
    // from the current sidepass labeled obstacle
    if (obstacle_sl.start_s() > sidepass_point_s &&
        obstacle_sl.start_s() <= sidepass_point_s + k_sidepass_range) {
      AINFO << "obstacle_id[" << obstacle_id << "] type[" << obstacle_type_name
             << "] Within sidepass range. SKIP";
        continue;
    }

    // use min width to take care splitting-lane scenario
    double start_s_left_width = 0.0;
    double start_s_right_width = 0.0;
    reference_line.GetLaneWidth(obstacle_sl.start_s(), &start_s_left_width,
                                &start_s_right_width);
    double end_s_left_width = 0.0;
    double end_s_right_width = 0.0;
    reference_line.GetLaneWidth(obstacle_sl.end_s(), &end_s_left_width,
                                &end_s_right_width);

    const double left_width = std::min(start_s_left_width, end_s_left_width);
    const double left_driving_width = left_width - obstacle_sl.end_l() -
                                      config_.front_vehicle().nudge_l_buffer();

    const double right_width = std::min(start_s_right_width, end_s_right_width);
    const double right_driving_width = right_width + obstacle_sl.start_l() -
                                       config_.front_vehicle().nudge_l_buffer();

    ADEBUG << "obstacle_id[" << obstacle_id << "] type[" << obstacle_type_name 
           << "] left_driving_width[" << left_driving_width
           << "] right_driving_width[" << right_driving_width << "] adc_width["
           << adc_width << "]";

    // JZ Added - limit nudge speed based on lateral buffer distance
    const auto& adc_planning_point = EgoInfo::instance()->start_point();
    double lateral_dist = std::max(left_driving_width - adc_width, right_driving_width - adc_width);
    double longitudinal_dist = obstacle_sl.start_s() - adc_sl.end_s();
    double k_nudge_speed_coef = config_.front_vehicle().nudge_speed_coef(); // JZ Added
    double adc_nudge_speed_limit = 1.0; //JZ Added, default parameter to be added
    if (lateral_dist > 0) {
      adc_nudge_speed_limit = lateral_dist * k_nudge_speed_coef;
    } else {
      adc_nudge_speed_limit = 0.0;
    }
    const double stop_distance =
        path_obstacle->MinRadiusStopDistance(vehicle_param);
    const double early_stop_dist = config_.front_vehicle().early_stop_buffer(); // JZ Added
    const double total_stop_dist = stop_distance + early_stop_dist;             // JZ Added

    // stop if not able to bypass, or if obstacle crossed reference line
    // or nudge_speed_limit surpassed (until stop point)
    if ((left_driving_width < adc_width && right_driving_width < adc_width) ||
        (obstacle_sl.start_l() <= 0.0 && obstacle_sl.end_l() >= 0.0) ||
        (adc_planning_point.v() > adc_nudge_speed_limit && longitudinal_dist > total_stop_dist)) {
      AINFO << "obstacle_id[" << obstacle_id << "] type[" << obstacle_type_name << "]" << " ON ROAD, CAN NOT NUDGE";
      AINFO << "left_driving_width[" << left_driving_width
            << "] right_driving_width[" << right_driving_width << "] adc_width["
            << adc_width << "]";
      AINFO << "obstacle_start_l[" << obstacle_sl.start_l()
            << "] obstacle_end_l[" << obstacle_sl.end_l();
      AINFO << "adc_speed = " << adc_planning_point.v() << " vs " << "adc_nudge_speed_limit = " << adc_nudge_speed_limit;
      ADEBUG << "STOP: obstacle[" << obstacle_id << "]";
      AINFO << "STOP: obstacle[" << obstacle_id << "]";

      // build stop decision

      const double stop_s = obstacle_sl.start_s() - stop_distance - early_stop_dist;
      auto stop_point = reference_line.GetReferencePoint(stop_s);
      const double stop_heading =
          reference_line.GetReferencePoint(stop_s).heading();

      ObjectDecisionType stop;
      auto stop_decision = stop.mutable_stop();
      if (obstacle_type == PerceptionObstacle::UNKNOWN_MOVABLE ||
          obstacle_type == PerceptionObstacle::BICYCLE ||
          obstacle_type == PerceptionObstacle::VEHICLE) {
        stop_decision->set_reason_code(
            StopReasonCode::STOP_REASON_HEAD_VEHICLE);
      } else {
        stop_decision->set_reason_code(StopReasonCode::STOP_REASON_OBSTACLE);
      }
      stop_decision->set_distance_s(-total_stop_dist);
      stop_decision->set_stop_heading(stop_heading);
      stop_decision->mutable_stop_point()->set_x(stop_point.x());
      stop_decision->mutable_stop_point()->set_y(stop_point.y());
      stop_decision->mutable_stop_point()->set_z(0.0);

      path_decision->AddLongitudinalDecision("front_vehicle",
                                             path_obstacle->Id(), stop);
      AINFO << "Build stop point and set reason code: " << path_obstacle->LongitudinalDecision().stop().reason_code();
      AINFO << "Stop point s = " << stop_s;
    }
    else {
      AINFO << "obstacle_id[" << obstacle_id << "] type[" << obstacle_type_name << "]" << " ON ROAD, NUDGE OK";
      AINFO << "left_driving_width[" << left_driving_width
            << "] right_driving_width[" << right_driving_width << "] adc_width["
            << adc_width << "]";
      
    }
  }
}

}  // namespace planning
}  // namespace apollo
