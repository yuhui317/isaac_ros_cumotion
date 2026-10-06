// MoveIt PlanningContext 薄适配：把详细/简化规划响应连接到 CumotionInterface，供规划插件调用。
// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2024-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// SPDX-License-Identifier: Apache-2.0

#include "isaac_ros_cumotion_moveit/cumotion_planning_context.hpp"

namespace nvidia
{
namespace isaac
{
namespace manipulation
{

// 将已设置的 planning_scene 与 request 原样委托给 cuMotion 接口。
void CumotionPlanningContext::solve(planning_interface::MotionPlanDetailedResponse & res)
{
  cumotion_interface_->solve(planning_scene_, request_, res);
}

// 先获取详细响应，再在成功时提取第一条轨迹与第一项处理时间。
void CumotionPlanningContext::solve(planning_interface::MotionPlanResponse & res)
{
  planning_interface::MotionPlanDetailedResponse res_detailed;
  solve(res_detailed);

  res.error_code = res_detailed.error_code;

  if (res) {
    res.trajectory = res_detailed.trajectory[0];
    res.planning_time = res_detailed.processing_time[0];
  }
}

}  // namespace manipulation
}  // namespace isaac
}  // namespace nvidia
