// cuMotion 规划器 ROS 接口：组合目标、机器人状态、碰撞世界与原生 cumotion_lib 规划组件；此头文件不实现控制器执行。
// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#ifndef ISAAC_ROS_CUMOTION__CUMOTION_PLANNER_HPP_
#define ISAAC_ROS_CUMOTION__CUMOTION_PLANNER_HPP_

#include <cumotion/pose3.h>
#include <cumotion/robot_world_inspector.h>
#include <cumotion/trajectory.h>
#include <cumotion/trajectory_optimizer.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Core>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <isaac_ros_cumotion_interfaces/action/ik_solution.hpp>
#include <isaac_ros_cumotion_interfaces/action/motion_plan.hpp>
#include <isaac_ros_cumotion_interfaces/srv/get_robot_description.hpp>
#include <isaac_ros_cumotion_interfaces/srv/publish_static_planning_scene.hpp>
#include <isaac_ros_cumotion_interfaces/srv/set_robot_description.hpp>
#include <moveit_msgs/action/move_group.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/motion_plan_request.hpp>
#include <moveit_msgs/msg/robot_state.hpp>
#include <nvblox_msgs/srv/esdf_and_gradients.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>


#include "isaac_ros_cumotion/impl/cumotion_logger_bridge.hpp"
#include "isaac_ros_cumotion/impl/ik_solver_impl.hpp"
#include "isaac_ros_cumotion/impl/robot_manager_impl.hpp"
#include "isaac_ros_cumotion/impl/trajectory_optimizer_impl.hpp"
#include "isaac_ros_cumotion/impl/utils.hpp"
#include "isaac_ros_cumotion/impl/world_manager_impl.hpp"

namespace nvidia
{
namespace isaac_ros
{
namespace cumotion
{

/**
 * Combined cuMotion planner node that provides:
 *  - MoveGroup action server
 *  - IK action server
 *  - MotionPlan action server
 */
// 节点持有三类动作服务器、TF、关节状态和 ESDF 客户端，规划求解细节位于对应 .cpp。
class CumotionPlanner final : public rclcpp::Node
{
public:
  using IKAction = isaac_ros_cumotion_interfaces::action::IKSolution;
  using MotionPlan = isaac_ros_cumotion_interfaces::action::MotionPlan;

  using GoalHandleMoveGroup = rclcpp_action::ServerGoalHandle<moveit_msgs::action::MoveGroup>;
  using GoalHandleIk = rclcpp_action::ServerGoalHandle<IKAction>;
  using GoalHandleMotionPlan = rclcpp_action::ServerGoalHandle<MotionPlan>;

  /**
   * Constructs and fully initializes a cuMotion planner node.
   */
  // 构造时初始化节点参数及各管理器，依赖 URDF/XRDF 与外部 cuMotion 库。
  explicit CumotionPlanner(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // Initializes the world manager for collision checking and environment updates.
  // 初始化碰撞世界管理器，后续 MoveIt 几何体与 nvblox ESDF 都由其维护。
  void InitializeWorldManager();

  // Initializes the robot manager for robot description, kinematics, and joint state management.
  // 初始化机器人描述、运动学以及 JointState 按名称重排的状态缓冲。
  void InitializeRobotManager();

  // Initializes the robot-world inspector used for collision sphere queries and validation.
  // 建立机器人/世界检查器，用于碰撞球查询与可视化。
  void InitializeRobotWorldInspector();

  // Initializes the trajectory optimizer for motion planning operations.
  // 建立调用原生 cuMotion 轨迹优化器的 ROS 侧包装。
  void InitializeTrajectoryOptimizer();

  // Initializes the IK solver for inverse kinematics queries.
  // 建立 IK 求解组件，供 IKSolution 动作使用。
  void InitializeIkSolver();

  // Initializes the MoveGroup action server for motion planning requests.
  // 注册兼容 MoveIt 的 MoveGroup 动作服务器，不等于轨迹控制器。
  void InitializeMoveGroupActionServer();

  // Initializes the IK action server for inverse kinematics requests.
  // 注册逆运动学动作服务器。
  void InitializeIkActionServer();

  // Initializes the MotionPlan (goalset) action server.
  // 注册支持目标集合等专用请求的 MotionPlan 动作服务器。
  void InitializeMotionPlanActionServer();

  // Initializes the static planning scene service client.
  // 建立读取静态碰撞场景的服务客户端。
  void InitializeStaticPlanningSceneClient();

  // Initializes robot description Get/Set services.
  // 注册机器人 URDF/XRDF 获取与替换服务。
  void InitializeRobotDescriptionService();

  // Publish collision-sphere markers.
  // 发布机器人碰撞球 Marker，用于观察碰撞模型与实体外形的关系。
  void PublishCollisionSpheres();

  // Callback for joint state messages. Updates the RobotManagerImpl state buffer.
  // 把实时关节消息交给 RobotManager，未知关节不会进入配置空间状态。
  void JointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

  // Gets the start configuration from a MoveIt motion plan request.
  // 提取请求起点或使用缓存当前关节状态；实现中请求位置向量的次序必须与配置空间一致。
  bool GetStartCSpacePosition(
    const moveit_msgs::msg::MotionPlanRequest & request,
    Eigen::VectorXd & start_cspace_position);

  // Gets goal constraints from a MoveIt motion plan request.
  // 解析 MoveIt 第一个约束集合中的关节目标或末端位置/朝向目标。
  bool GetGoal(
    const moveit_msgs::msg::MotionPlanRequest & request,
    cumotion_lib::Pose3 & goal_task_space_pose,
    Eigen::VectorXd & goal_cspace_position,
    bool & has_pose_goal,
    bool & has_joint_goal);

  // Calls the static planning scene service to load collision objects from a scene file.
  // 调用静态场景服务以加载场景文件中的碰撞几何。
  void CallPublishStaticPlanningSceneService();

  // Callback for the static planning scene service response.
  // 接收场景服务响应，后续规划合并静态与请求中的动态物体。
  void StaticSceneServiceCallback(
    rclcpp::Client<isaac_ros_cumotion_interfaces::srv::PublishStaticPlanningScene>::SharedFuture
    future);

  // Robot description service callbacks.
  // 返回缓存的机器人描述文本，供外部客户端核对当前模型。
  void HandleGetRobotDescription(
    const std::shared_ptr<isaac_ros_cumotion_interfaces::srv::GetRobotDescription::Request> request,
    std::shared_ptr<isaac_ros_cumotion_interfaces::srv::GetRobotDescription::Response> response);

  // 处理运行时机器人模型替换；依赖组件需要随机器人描述一起重新建立。
  void HandleSetRobotDescription(
    const std::shared_ptr<isaac_ros_cumotion_interfaces::srv::SetRobotDescription::Request> request,
    std::shared_ptr<isaac_ros_cumotion_interfaces::srv::SetRobotDescription::Response> response);

  // 检查规划器繁忙或机器人模型正在替换时是否拒绝 MoveGroup 请求。
  rclcpp_action::GoalResponse HandleMoveGroupGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const moveit_msgs::action::MoveGroup::Goal> goal);

  // 接收取消请求的协议入口，取消传播能力仍应结合执行实现检查。
  rclcpp_action::CancelResponse HandleMoveGroupCancel(
    const std::shared_ptr<GoalHandleMoveGroup> goal_handle);

  // 接受目标后的入口，.cpp 中另起线程执行规划。
  void HandleMoveGroupAccepted(
    const std::shared_ptr<GoalHandleMoveGroup> goal_handle);

  // 更新世界、读取起点和目标，再调用轨迹优化并返回 MoveIt RobotTrajectory。
  void ExecuteMoveGroupGoal(
    const std::shared_ptr<GoalHandleMoveGroup> goal_handle);

  // IK 目标接受/拒绝的入口，与规划共用机器人和世界状态。
  rclcpp_action::GoalResponse HandleIkGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const IKAction::Goal> goal);

  // IK 取消请求的入口。
  rclcpp_action::CancelResponse HandleIkCancel(
    const std::shared_ptr<GoalHandleIk> goal_handle);

  // 启动已接受的 IK 请求。
  void HandleIkAccepted(
    const std::shared_ptr<GoalHandleIk> goal_handle);

  // 执行 IK 请求并组织动作结果。
  void ExecuteIkGoal(
    const std::shared_ptr<GoalHandleIk> goal_handle);

  // 专用 MotionPlan 请求的接受/拒绝入口。
  rclcpp_action::GoalResponse HandleMotionPlanGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const MotionPlan::Goal> goal);

  // 专用规划动作的取消入口。
  rclcpp_action::CancelResponse HandleMotionPlanCancel(
    const std::shared_ptr<GoalHandleMotionPlan> goal_handle);

  // 启动已接受的专用 MotionPlan 请求。
  void HandleMotionPlanAccepted(const std::shared_ptr<GoalHandleMotionPlan> goal_handle);

  // 分派专用规划请求到抓取、关节空间或姿态目标流程。
  void ExecuteMotionPlan(const std::shared_ptr<GoalHandleMotionPlan> goal_handle);

  // 专用抓取规划分支接口；本轮仅阅读声明，具体路径约束留作下一轮分析。
  void ExecuteGraspPlanning(
    const std::shared_ptr<const MotionPlan::Goal> & request,
    const Eigen::VectorXd & start_state,
    double time_dilation,
    std::shared_ptr<MotionPlan::Result> & result);

  // 专用关节空间规划分支接口。
  void ExecuteCSpacePlanning(
    const std::shared_ptr<const MotionPlan::Goal> & request,
    const Eigen::VectorXd & start_state,
    double time_dilation,
    std::shared_ptr<MotionPlan::Result> & result);

  // 专用姿态目标规划分支接口。
  void ExecutePosePlanning(
    const std::shared_ptr<const MotionPlan::Goal> & request,
    const Eigen::VectorXd & start_state,
    double time_dilation,
    std::shared_ptr<MotionPlan::Result> & result);

  // 把允许的平移/朝向路径与终端偏差表达成优化器约束。
  void CreatePathConstraintsFromParams(
    float translation_path_deviation_limit,
    float translation_terminal_deviation_limit,
    bool enable_orientation_path_axis_constraint,
    float orientation_path_axis_deviation_limit,
    float orientation_terminal_deviation_limit,
    const cumotion_lib::Pose3 & target_pose,
    const std::string & constraint_name,
    cumotion_lib::TrajectoryOptimizer::TranslationConstraint & translation_constraint,
    cumotion_lib::TrajectoryOptimizer::OrientationConstraint & orientation_constraint);

  // 从专用 MotionPlan 请求解析配置空间起点。
  bool GetMotionPlanStartState(
    const std::shared_ptr<const MotionPlan::Goal> & request,
    Eigen::VectorXd & start_state);

  // 从专用请求提取候选目标 Pose 集合。
  bool ExtractGoalPoses(
    const std::shared_ptr<const MotionPlan::Goal> & request,
    std::vector<cumotion_lib::Pose3> & goal_poses);

  // Gets `object_frame` pose in `world_frame`. Returns true on success.
  // 利用 TF 查询物体相对 world_frame 的 Pose，失败通过布尔结果反馈。
  bool GetObjectPose(
    const std::string & world_frame,
    const std::string & object_frame,
    geometry_msgs::msg::Pose & object_pose);

  // 按偏移所属坐标系组合两个 Pose，局部/世界偏移的乘法次序不可互换。
  cumotion_lib::Pose3 ComposeWithOffset(
    const cumotion_lib::Pose3 & base_pose,
    const cumotion_lib::Pose3 & offset,
    bool offset_in_base_frame);

  // Create an ESDF request message (requests all allocated blocks from nvblox).
  // 构造 nvblox ESDF 请求，可附带待清除物体区域。
  std::shared_ptr<nvblox_msgs::srv::EsdfAndGradients::Request> CreateEsdfRequest(
    const EsdfClearingObjects * clearing_objects = nullptr,
    bool update_esdf = true) const;

  /**
   * Update the cuMotion world ESDF grid by calling the nvblox ESDF service.
   * Returns true on success, false on failure.
   */
  // 从 nvblox 拉取距离场并交给世界管理器，失败将影响碰撞可用性。
  bool UpdateEsdfFromNvblox(const EsdfClearingObjects * clearing_objects, bool update_esdf);

  /**
   * Initializes ESDF workspace bounds before planning consumes the ESDF.
   * Returns true if bounds are already cached or were initialized successfully.
   */
  // 在规划使用距离场前确保工作空间几何边界已缓存。
  bool EnsureEsdfWorkspaceBoundsCached();

  // Publish the current cuMotion world occupancy as voxels if enabled and subscribed.
  // 将世界占据体素以可视化消息发布，并不替代碰撞数据输入。
  void PublishWorldVoxels();

  // Robot manager for robot description, kinematics, and joint state management.
  std::unique_ptr<RobotManagerImpl> robot_manager_;

  // Trajectory optimizer implementation for motion planning operations.
  std::unique_ptr<TrajectoryOptimizerImpl> trajectory_optimizer_;

  /**
   * Collision-free IK solver implementation for generating c-space solutions to task-space
   * targets.
   */
  std::unique_ptr<IkSolverImpl> ik_solver_;

  // World manager for collision checking and environment updates.
  std::shared_ptr<WorldManagerImpl> world_manager_;

  // Robot-world inspector used to query robot collision spheres for visualization.
  std::unique_ptr<cumotion_lib::RobotWorldInspector> robot_world_inspector_;

  // Logger bridge for routing cuMotion logs through ROS2.
  std::shared_ptr<CumotionLoggerBridge> logger_;

  // Publisher for visualizing robot collision spheres in RViz.
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr collision_spheres_pub_;

  // Publisher for visualizing planned trajectories in RViz (MoveIt DisplayTrajectory).
  rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_trajectory_pub_;

  // Publisher for world voxel visualization.
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr voxel_pub_;

  // Action server for MoveGroup requests.
  rclcpp_action::Server<moveit_msgs::action::MoveGroup>::SharedPtr move_group_action_server_;

  // Action server for IK solution requests.
  rclcpp_action::Server<IKAction>::SharedPtr ik_action_server_;

  // MotionPlan action server (goalset planner).
  rclcpp_action::Server<MotionPlan>::SharedPtr motion_plan_server_;

  // Callback group for the MotionPlan action server.
  rclcpp::CallbackGroup::SharedPtr action_server_cb_group_;

  // TF2 buffer for coordinate frame transformations.
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  // TF2 listener for receiving transforms.
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Callback group dedicated to JointState subscription.
  rclcpp::CallbackGroup::SharedPtr joint_state_cb_group_;

  // Joint state subscription (feeds RobotManagerImpl::UpdateJointState).
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  // Service client for requesting ESDF and gradient data from nvblox mapper.
  rclcpp::Client<nvblox_msgs::srv::EsdfAndGradients>::SharedPtr esdf_client_;

  // Callback group for ESDF service calls to avoid callback starvation.
  rclcpp::CallbackGroup::SharedPtr esdf_client_cb_group_;

  // Client for the static planning scene service.
  rclcpp::Client<isaac_ros_cumotion_interfaces::srv::PublishStaticPlanningScene>::SharedPtr
    static_planning_scene_client_;

  // Service servers for robot description management.
  rclcpp::CallbackGroup::SharedPtr robot_description_cb_group_;
  rclcpp::Service<isaac_ros_cumotion_interfaces::srv::GetRobotDescription>::SharedPtr
    get_robot_description_service_;
  rclcpp::Service<isaac_ros_cumotion_interfaces::srv::SetRobotDescription>::SharedPtr
    set_robot_description_service_;

  // Topic published whenever robot description is successfully updated.
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr robot_description_updated_pub_;

  // Monotonic sequence number for robot-description update events.
  uint64_t robot_description_update_seq_{0};

  // Robot configuration parameters.
  const std::string urdf_file_path_;
  const std::string xrdf_file_path_;
  const std::string joint_states_topic_;

  // Trajectory interpolation parameter.
  const double interpolation_dt_;

  // World/ESDF parameters.
  const bool read_esdf_world_;
  const bool publish_cumotion_world_as_voxels_;
  const bool add_ground_plane_;
  const double publish_voxel_size_;
  const int max_publish_voxels_;
  const double ground_plane_size_x_;
  const double ground_plane_size_y_;
  const double ground_plane_thickness_;
  const double ground_plane_z_offset_;
  const bool update_esdf_on_request_;
  const std::string esdf_service_name_;

  // Service timeout parameters.
  const int static_scene_service_max_wait_attempts_;

  // Service name.
  const std::string static_planning_scene_service_name_;

  // Motion planning parameters.
  const double time_dilation_factor_;
  const bool override_moveit_scaling_;

  // Debug mode.
  const bool debug_mode_;

  // When true, world-collision spheres are included in the collision sphere visualization.
  const bool publish_world_collision_spheres_;

  // When true, self-collision spheres are included in the collision sphere visualization.
  const bool publish_self_collision_spheres_;

  // Configuration for the world manager.
  WorldManagerImpl::Config world_config_;

  /**
   * Collision objects from the static planning scene in MoveIt format.
   * NOTE: These are MoveIt collision objects, not cuMotion native obstacles. They are converted
   * to cuMotion obstacles by WorldManagerImpl before being added to the cuMotion world.
   */
  std::vector<moveit_msgs::msg::CollisionObject> world_objects_;

  // Flag indicating if the planner is currently busy.
  bool planner_busy_{false};

  // Flag indicating SetRobotDescription is currently updating the robot model.
  bool robot_description_update_in_progress_{false};

  // Mutex to protect planner_busy_ from concurrent access.
  // 保护规划器繁忙状态等跨回调标志；机器人模型替换还使用单独的互斥锁。
  mutable std::mutex state_mutex_;

  // Mutex to serialize robot description swaps and dependent component rebuilds.
  mutable std::mutex robot_description_mutex_;
};

}  // namespace cumotion
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_CUMOTION__CUMOTION_PLANNER_HPP_
