// Copyright 2021 ros2_control development team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// 헤더파일과 네임스페이스 정의
#include "joint_trajectory_command_broadcaster/joint_trajectory_command_broadcaster.hpp"
#include <cstddef>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <atomic>
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/time.hpp"
#include "std_msgs/msg/header.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "std_msgs/msg/bool.hpp"
#include "urdf/model.h"

// rclcpp_lifecycle 네임스페이스에 State 정의해둠
namespace rclcpp_lifecycle
{
class State;
}  // namespace rclcpp_lifecycle

// joint trajectory command broadcaster 네임스페이스에 
namespace joint_trajectory_command_broadcaster
{
// numeric_limits로 값이 지정되지 않거나 초기화안되면 nan으로 설정해둠
const auto kUninitializedValue = std::numeric_limits<double>::quiet_NaN();
using hardware_interface::HW_IF_POSITION; // 위치 제어 인터페이스 이름

// JointTrajectoryCommandBroadcaster 생성자
JointTrajectoryCommandBroadcaster::JointTrajectoryCommandBroadcaster() {} 

// 컨트롤러 초기화 함수
controller_interface::CallbackReturn JointTrajectoryCommandBroadcaster::on_init()
{
  try {
    // ROS2 노드에서 파라미터를 읽는 객체 생성
    // ParamListener는 노드에서 joint, interface 등 설정값을 가져오는 역할
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
  } catch (const std::exception & e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

/*
인터페이스란?
하드웨어(모터, 센서 등)와 소프트웨어(컨트롤러) 사이의 통신 창구

대표적인 인터페이스 이름	역할
position	목표 위치를 읽거나 쓰는 용도 (ex. 모터 90도 회전)
velocity	속도를 읽거나 쓰는 용도 (ex. 모터 10deg/s 회전)
effort	힘/토크를 읽거나 쓰는 용도 (ex. 모터 토크 2Nm 적용)
*/

// command_interface_configuration(): 컨트롤러가 명령을 보낼 인터페이스 정의하는 함수
controller_interface::InterfaceConfiguration
JointTrajectoryCommandBroadcaster::command_interface_configuration() const
// 이 컨트롤러는 직접 하드웨어에 명령을 쓰지 않고,
// trajectory 메시지를 발행만 하기 때문에 명령 인터페이스가 필요 없음
{
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::NONE};
}
//InterfaceConfiguration → 어떤 하드웨어 인터페이스를 쓸지 정의
//interface_configuration_type::NONE → 명령 인터페이스는 사용하지 않음

controller_interface::InterfaceConfiguration JointTrajectoryCommandBroadcaster::
state_interface_configuration()
const
{
  controller_interface::InterfaceConfiguration state_interfaces_config; // 결과 객체 생성

  // 만약 모든 인터페이스를 사용하도록 설정돼 있다면
  if (use_all_available_interfaces()) {
    state_interfaces_config.type = controller_interface::interface_configuration_type::ALL;
  } else {  // 특정 조인트, 특정 인터페이스만 사용
    state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
     // params_.joints: 사용할 조인트 이름 목록
    for (const auto & joint : params_.joints) {
      // 예: "joint1/position", "joint2/velocity" 형태로 names에 추가
      for (const auto & interface : params_.interfaces) {
        state_interfaces_config.names.push_back(joint + "/" + interface);
      }
    }
  }

  return state_interfaces_config; // 컨트롤러에 전달
}

// ROS2 Lifecycle Controller에서 컨트롤러가 활성화되기 전에 설정단계에서 실행
controller_interface::CallbackReturn JointTrajectoryCommandBroadcaster::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Lifecycle 컨트롤러 초기화 단계에서 ParamListener를 만들어 params_를 가져왔는지 확인
  if (!param_listener_) {
    RCLCPP_ERROR(get_node()->get_logger(), "Error encountered during init");
    return controller_interface::CallbackReturn::ERROR;
  }
  // 파라미터 다시 가져오기 (혹시 업데이트 됐을 수 있으므로)
  params_ = param_listener_->get_params();

  // joints나 interfaces가 비어있으면 모든 인터페이스 사용  
  if (use_all_available_interfaces()) {
    RCLCPP_INFO(
      get_node()->get_logger(),
      "'joints' or 'interfaces' parameter is empty. "
      "All available state interfaces will be considered.");
    params_.joints.clear();
    params_.interfaces.clear();
  } else { // 특정 joints/인터페이스만 사용하는 경우
    RCLCPP_INFO(
      get_node()->get_logger(),
      "Publishing trajectory states for the defined 'joints' and 'interfaces' parameters.");
  }

  // Map interface if needed
  // 인터페이스 이름을 joint state 이름과 매핑
  map_interface_to_joint_state_.clear(); 
  map_interface_to_joint_state_[HW_IF_POSITION] = params_.map_interface_to_joint_state.position;

  try {
    // 토픽 이름 설정 (~ = 노드 네임스페이스 안에서 로컬 토픽 사용 여부)
    const std::string topic_name_prefix = params_.use_local_topics ? "~/" : "";
    // Create publisher for JointTrajectory
    // JointTrajectory 메시지 publisher 생성
    joint_trajectory_publisher_ =
      get_node()->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      topic_name_prefix + "joint_trajectory", rclcpp::SystemDefaultsQoS());
    
    // 실시간 퍼블리셔 생성 (RealtimePublisher는 락 없이 안전하게 퍼블리시 가능)
    realtime_joint_trajectory_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<trajectory_msgs::msg::JointTrajectory>>(
      joint_trajectory_publisher_);
  } catch (const std::exception & e) {
    // get_node() may throw, logging raw here
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  // 서브 collision_flag
  // 외부 노드가 /collision_flag 토픽에 Bool 값을 퍼블리시하면 즉, 충돌이 일어났으면 콜백함
  collision_flag_sub_ = get_node()->create_subscription<std_msgs::msg::Bool>(
    "/collision_flag", rclcpp::QoS(10),
    std::bind(&JointTrajectoryCommandBroadcaster::collision_callback, this, std::placeholders::_1));

  // URDF 로드 및 파싱
  const std::string & urdf = get_robot_description();
  is_model_loaded_ = !urdf.empty() && model_.initString(urdf);
  if (!is_model_loaded_) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      // URDF 없으면 필터링 없이 모든 조인트 고려
      "Failed to parse robot description. Will proceed without URDF-based filtering.");
  }

  return CallbackReturn::SUCCESS;
}

// 컨트롤러 활성화
controller_interface::CallbackReturn JointTrajectoryCommandBroadcaster::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // 로봇 관절과 연결된 장비(인터페이스)가 제대로 존재하는지 확인.
  if (!init_joint_data()) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "None of requested interfaces exist. Controller will not run.");
    return CallbackReturn::ERROR;
  }

  // Check offsets and create mapping based on params_.joints order
  // 조인트 보정값(offset)을 저장할 리스트 초기화
  //    모든 관절 보정값을 0으로 시작
  joint_offsets_.clear();
  joint_offsets_.resize(params_.joints.size(), 0.0);

  // 만약 설정 파일(params)에 관절 보정값이 제공되었다면 처리
  if (!params_.offsets.empty()) {
    // 관절 수와 보정값 수가 다르면 잘못된 설정 → 에러
    if (params_.offsets.size() != params_.joints.size()) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "The number of provided offsets (%zu) does not match the number of joints in params (%zu).",
        params_.offsets.size(), params_.joints.size());
      return CallbackReturn::ERROR;
    }

    // Create mapping from joint name to offset based on params_.joints order
    // (관절 이름 → 보정값) 매핑 테이블 만들기
    // ex) { "joint1": 0.1, "joint2": -0.05 }
    std::unordered_map<std::string, double> joint_offset_map;
    for (size_t i = 0; i < params_.joints.size(); ++i) {
      joint_offset_map[params_.joints[i]] = params_.offsets[i];
    }

    // 실제 관절 배열 순서(joint_names_)에 맞추어 보정값 넣기
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      auto it = joint_offset_map.find(joint_names_[i]); // 이름이 같은 관절 찾기
      if (it != joint_offset_map.end()) {
        joint_offsets_[i] = it->second; // 보정값 적용
      }
      // If joint not found in params_.joints, offset remains 0.0
    }
  }
  // No need to init JointState or DynamicJointState messages, only JointTrajectory
  // will be published. We'll construct it on-the-fly in update()
  
   // 요청된 관절 인터페이스 수와 실제 존재하는 인터페이스 수가 다를 경우 경고 출력
  if (
    !use_all_available_interfaces() &&
    state_interfaces_.size() != (params_.joints.size() * params_.interfaces.size()))
  {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "Not all requested interfaces exist. "
      "Check ControllerManager output for more detailed information.");
  }

  //모든 준비가 잘 끝남 → 컨트롤러 활성화 성공
  return CallbackReturn::SUCCESS;
}

// 컨트롤러 비활성화
controller_interface::CallbackReturn JointTrajectoryCommandBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  joint_names_.clear();
  name_if_value_mapping_.clear();

  return CallbackReturn::SUCCESS;
}

/* 맵 안에 특정 키가 있는지 검사하는 도구
{
  "position": 값,
  "velocity": 값,
  "effort": 값
}
이런 맵이 있을 떄 ["position"] 가 있는지 확인하고 싶을때 사용

로봇 joint의 상태(interface) 중에서
position 정보가 있는 joint만 골라내기 위해 쓰임.
*/ 
template<typename T>
bool has_any_key(
  const std::unordered_map<std::string, T> & map, const std::vector<std::string> & keys)
{
  for (const auto & key_item : map) {
    const auto & key = key_item.first;
    if (std::find(keys.cbegin(), keys.cend(), key) != keys.cend()) {
      return true;
    }
  }
  return false;
}

// 이 컨트롤러가 사용할 joint 목록을 결정하는 함수
bool JointTrajectoryCommandBroadcaster::init_joint_data()
{
  // 그 전 명령어에 있던 정보들 초기화
  joint_names_.clear();
  if (state_interfaces_.empty()) {
    return false;
  }

  // Initialize mapping
  /*
  { joint="joint1", interface="position", value=1.57 },
  { joint="joint1", interface="velocity", value=0.2 },
  { joint="joint2", interface="position", value=-0.3 },
  { joint="joint2", interface="velocity", value=0.0 },
  -> 이런값들을 아래처럼 변환
  name_if_value_mapping_ = {
  "joint1": { "position": 값, "velocity": 값 },
  "joint2": { "position": 값, "velocity": 값 },
  "joint3": { "velocity": 값 }
  }
  */
  for (auto si = state_interfaces_.crbegin(); si != state_interfaces_.crend(); si++) {
    if (name_if_value_mapping_.count(si->get_prefix_name()) == 0) {
      name_if_value_mapping_[si->get_prefix_name()] = {};
    }
    std::string interface_name = si->get_interface_name();
    if (map_interface_to_joint_state_.count(interface_name) > 0) {
      interface_name = map_interface_to_joint_state_[interface_name];
    }
    name_if_value_mapping_[si->get_prefix_name()][interface_name] = kUninitializedValue;
  }

  // Filter out joints without position interface (since we want positions)
  // position 인터페이스가 있는 joint만 골라내기
  for (const auto & name_ifv : name_if_value_mapping_) {
    const auto & interfaces_and_values = name_ifv.second;
    if (has_any_key(interfaces_and_values, {HW_IF_POSITION})) {
      if (
        !params_.use_urdf_to_filter || !params_.joints.empty() || !is_model_loaded_ ||
        model_.getJoint(name_ifv.first))
      {
        joint_names_.push_back(name_ifv.first);
      }
    }
  }

  // Add extra joints if needed
  // extra_joints 파라미터가 있으면 추가
  rclcpp::Parameter extra_joints;
  if (get_node()->get_parameter("extra_joints", extra_joints)) {
    const std::vector<std::string> & extra_joints_names = extra_joints.as_string_array();
    for (const auto & extra_joint_name : extra_joints_names) {
      if (name_if_value_mapping_.count(extra_joint_name) == 0) {
        name_if_value_mapping_[extra_joint_name] = {
          {HW_IF_POSITION, 0.0}};
        joint_names_.push_back(extra_joint_name);
      }
    }
  }

  return true;
}

bool JointTrajectoryCommandBroadcaster::use_all_available_interfaces() const
{
  return params_.joints.empty() || params_.interfaces.empty();
}

double get_value(
  const std::unordered_map<std::string, std::unordered_map<std::string, double>> & map,
  const std::string & name, const std::string & interface_name)
{
  const auto & interfaces_and_values = map.at(name);
  const auto interface_and_value = interfaces_and_values.find(interface_name);
  if (interface_and_value != interfaces_and_values.cend()) {
    return interface_and_value->second;
  } else {
    return kUninitializedValue;
  }
}

controller_interface::return_type JointTrajectoryCommandBroadcaster::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (collision_detected_.load()) {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 2000,
      "Collision detected. Skipping joint_trajectory publish.");
    return controller_interface::return_type::OK;
  }
  // Update stored values
  for (const auto & state_interface : state_interfaces_) {
    std::string interface_name = state_interface.get_interface_name();
    if (map_interface_to_joint_state_.count(interface_name) > 0) {
      interface_name = map_interface_to_joint_state_[interface_name];
    }
    auto value = state_interface.get_optional();
    if (value) {
      name_if_value_mapping_[state_interface.get_prefix_name()][interface_name] = *value;
    }
  }

  // Publish JointTrajectory message with current positions
  if (realtime_joint_trajectory_publisher_ && realtime_joint_trajectory_publisher_->trylock()) {
    auto & traj_msg = realtime_joint_trajectory_publisher_->msg_;
    traj_msg.header.stamp = rclcpp::Time(0, 0);
    traj_msg.joint_names = joint_names_;

    const size_t num_joints = joint_names_.size();
    traj_msg.points.clear();
    traj_msg.points.resize(1);
    traj_msg.points[0].positions.resize(num_joints, kUninitializedValue);

    for (size_t i = 0; i < num_joints; ++i) {
      double pos_value =
        get_value(name_if_value_mapping_, joint_names_[i], HW_IF_POSITION);

      // Check if the current joint is in the reverse_joints parameter
      if (
        std::find(
          params_.reverse_joints.begin(),
          params_.reverse_joints.end(),
          joint_names_[i]) != params_.reverse_joints.end())
      {
        pos_value = -pos_value;
      }

      // Apply offset
      pos_value += joint_offsets_[i];

      traj_msg.points[0].positions[i] = pos_value;
    }

    // Optionally set velocities/accelerations/time_from_start if needed
    traj_msg.points[0].time_from_start = rclcpp::Duration(0, 0);  // immediate

    realtime_joint_trajectory_publisher_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

void JointTrajectoryCommandBroadcaster::collision_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  collision_detected_.store(msg->data);
}

}  // namespace joint_trajectory_command_broadcaster

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  joint_trajectory_command_broadcaster::JointTrajectoryCommandBroadcaster,
  controller_interface::ControllerInterface)
