// Copyright 2021 Institute for Robotics and Intelligent Machines,
//                Georgia Institute of Technology
// Copyright 2024 Intelligent Robotics Lab
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
//
// Author: Christian Llanes <christian.llanes@gatech.edu>
// Author: David Vargas Frutos <david.vargas@urjc.es>
// Author: Francisco Martín <fmrico@urjc.es>

#include <string>
#include <vector>
#include <memory>

#include "mocap4r2_msgs/msg/marker.hpp"
#include "mocap4r2_msgs/msg/markers.hpp"

#include "mocap4r2_optitrack_driver/mocap4r2_optitrack_driver.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include <sys/types.h>

namespace mocap4r2_optitrack_driver
{

OptitrackDriverNode::OptitrackDriverNode()
: ControlledLifecycleNode("mocap4r2_optitrack_driver_node")
{
  declare_parameter<std::string>("connection_type", "Unicast");
  declare_parameter<std::string>("server_address", "000.000.000.000");
  declare_parameter<std::string>("local_address", "000.000.000.000");
  declare_parameter<std::string>("multicast_address", "000.000.000.000");
  declare_parameter<uint16_t>("server_command_port", 0);
  declare_parameter<uint16_t>("server_data_port", 0);
  declare_parameter<uint16_t>("rigid_body_count", 3);

  this->get_parameter("rigid_body_count", rigid_body_count_);
  // Declare rigid body name parameters
  for (int i = 0; i < rigid_body_count_; ++i) {  // allow up to 10 rigid bodies
    declare_parameter<std::string>("rigid_body_name_id" + std::to_string(i), "");
  }

  client = new NatNetClient();
  client->SetFrameReceivedCallback(process_frame_callback, this);
}

void OptitrackDriverNode::set_settings_optitrack()
{
  if (connection_type_ == "Multicast") {
    client_params.connectionType = ConnectionType::ConnectionType_Multicast;
    client_params.multicastAddress = multicast_address_.c_str();
  } else if (connection_type_ == "Unicast") {
    client_params.connectionType = ConnectionType::ConnectionType_Unicast;
  } else {
    RCLCPP_FATAL(get_logger(), "Unknown connection type -- options are Multicast, Unicast");
    rclcpp::shutdown();
  }

  client_params.serverAddress = server_address_.c_str();
  client_params.localAddress = local_address_.c_str();
  client_params.serverCommandPort = server_command_port_;
  client_params.serverDataPort = server_data_port_;
}

bool OptitrackDriverNode::stop_optitrack()
{
  RCLCPP_INFO(get_logger(), "Disconnecting from optitrack DataStream SDK");
  return true;
}

void OptitrackDriverNode::control_start(const mocap4r2_control_msgs::msg::Control::SharedPtr msg)
{
  (void)msg;
}

void OptitrackDriverNode::control_stop(const mocap4r2_control_msgs::msg::Control::SharedPtr msg)
{
  (void)msg;
}

void NATNET_CALLCONV process_frame_callback(sFrameOfMocapData * p_data, void * p_user_data)
{
  static_cast<OptitrackDriverNode *>(p_user_data)->process_frame(p_data);
}

std::chrono::nanoseconds OptitrackDriverNode::get_optitrack_system_latency(sFrameOfMocapData * data)
{
  if (data == nullptr || data->CameraMidExposureTimestamp == 0) {
    RCLCPP_WARN_ONCE(get_logger(), "Optitrack's system latency not available");
    return std::chrono::nanoseconds::zero();
  }

  const double client_latency_sec =
    client->SecondsSinceHostTimestamp(data->CameraMidExposureTimestamp);
  const double client_latency_millisec = client_latency_sec * 1000.0;
  const double transit_latency_millisec =
    client->SecondsSinceHostTimestamp(data->TransmitTimestamp) * 1000.0;

  const double large_latency_threshold = 100.0;
  if (client_latency_millisec >= large_latency_threshold) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *this->get_clock(), 500,
      "Optitrack system latency >%.0f ms: [Transmission: %.0fms, Total: %.0fms]",
      large_latency_threshold, transit_latency_millisec, client_latency_millisec);
  }

  return round<std::chrono::nanoseconds>(std::chrono::duration<float>{client_latency_sec});
}

void OptitrackDriverNode::process_frame(sFrameOfMocapData * data)
{
  if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    return;
  }

  frame_number_++;
  rclcpp::Duration frame_delay = rclcpp::Duration(get_optitrack_system_latency(data));

  std::map<int, std::vector<mocap4r2_msgs::msg::Marker>> marker2rb;

  // Markers
  if (mocap4r2_markers_pub_->get_subscription_count() > 0) {
    mocap4r2_msgs::msg::Markers msg;
    msg.header.stamp = now() - frame_delay;
    msg.header.frame_id = "map";
    msg.frame_number = frame_number_;

    for (int i = 0; i < data->nLabeledMarkers; i++) {
      bool unlabeled = ((data->LabeledMarkers[i].params & 0x10) != 0);
      bool active_marker = ((data->LabeledMarkers[i].params & 0x20) != 0);
      sMarker & marker_data = data->LabeledMarkers[i];
      int model_id = 0;
      int marker_id = 0;
      NatNet_DecodeID(marker_data.ID, &model_id, &marker_id);

      mocap4r2_msgs::msg::Marker marker;
      marker.id_type = mocap4r2_msgs::msg::Marker::USE_INDEX;
      marker.marker_index = i;
      marker.translation.x = marker_data.x;
      marker.translation.y = marker_data.y;
      marker.translation.z = marker_data.z;
      if (active_marker || unlabeled) {
        msg.markers.push_back(marker);
      } else {
        marker2rb[model_id].push_back(marker);
      }
    }
    mocap4r2_markers_pub_->publish(msg);
  }

  // Rigid Bodies & Individual Poses
  if (data->nRigidBodies > 0) {
    mocap4r2_msgs::msg::RigidBodies msg_rb;
    msg_rb.header.stamp = now() - frame_delay;
    msg_rb.header.frame_id = "map";
    msg_rb.frame_number = frame_number_;

    for (int i = 0; i < data->nRigidBodies; i++) {
      int rigid_body_id = data->RigidBodies[i].ID;

      // Create publisher for this rigid body if it doesn't exist
      if (pose_publishers_.find(rigid_body_id) == pose_publishers_.end()) {
        std::string param_name = "rigid_body_name_id" + std::to_string(rigid_body_id);
        if (has_parameter(param_name)) {
          std::string body_name = get_parameter(param_name).get_value<std::string>();
          if (!body_name.empty()) {
            pose_publishers_[rigid_body_id] = create_publisher<geometry_msgs::msg::PoseStamped>(
              body_name, rclcpp::QoS(1000));
            pose_publishers_[rigid_body_id]->on_activate();
          }
        }
      }

      mocap4r2_msgs::msg::RigidBody rb;
      rb.rigid_body_name = std::to_string(rigid_body_id);
      rb.pose.position.x = data->RigidBodies[i].x;
      rb.pose.position.y = data->RigidBodies[i].y;
      rb.pose.position.z = data->RigidBodies[i].z;
      rb.pose.orientation.x = data->RigidBodies[i].qx;
      rb.pose.orientation.y = data->RigidBodies[i].qy;
      rb.pose.orientation.z = data->RigidBodies[i].qz;
      rb.pose.orientation.w = data->RigidBodies[i].qw;
      rb.markers = marker2rb[rigid_body_id];

      msg_rb.rigidbodies.push_back(rb);

      // Publish individual pose
      auto pose_pub_it = pose_publishers_.find(rigid_body_id);
      if (pose_pub_it != pose_publishers_.end()) {
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header = msg_rb.header;
        pose_msg.pose = rb.pose;
        pose_pub_it->second->publish(pose_msg);
      }
    }

    if (mocap4r2_rigid_body_pub_->get_subscription_count() > 0) {
      mocap4r2_rigid_body_pub_->publish(msg_rb);
    }
  }
}

using CallbackReturnT =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturnT
OptitrackDriverNode::on_configure(const rclcpp_lifecycle::State & state)
{
  (void)state;
  init_parameters();

  mocap4r2_markers_pub_ = create_publisher<mocap4r2_msgs::msg::Markers>(
    "markers", rclcpp::QoS(1000));
  mocap4r2_rigid_body_pub_ = create_publisher<mocap4r2_msgs::msg::RigidBodies>(
    "rigid_bodies", rclcpp::QoS(1000));

  connect_optitrack();

  RCLCPP_INFO(get_logger(), "Configured!\n");
  return ControlledLifecycleNode::on_configure(state);
}

CallbackReturnT
OptitrackDriverNode::on_activate(const rclcpp_lifecycle::State & state)
{
  (void)state;
  mocap4r2_markers_pub_->on_activate();
  mocap4r2_rigid_body_pub_->on_activate();

  // Activate individual pose publishers
  for (auto & [id, publisher] : pose_publishers_) {
    publisher->on_activate();
  }

  RCLCPP_INFO(get_logger(), "Activated!\n");
  return ControlledLifecycleNode::on_activate(state);
}

CallbackReturnT
OptitrackDriverNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  (void)state;
  mocap4r2_markers_pub_->on_deactivate();
  mocap4r2_rigid_body_pub_->on_deactivate();

  // Deactivate individual pose publishers
  for (auto & [id, publisher] : pose_publishers_) {
    publisher->on_deactivate();
  }

  RCLCPP_INFO(get_logger(), "Deactivated!\n");
  return ControlledLifecycleNode::on_deactivate(state);
}

CallbackReturnT
OptitrackDriverNode::on_cleanup(const rclcpp_lifecycle::State & state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Cleaned up!\n");

  if (disconnect_optitrack()) {
    return ControlledLifecycleNode::on_cleanup(state);
  }
  return CallbackReturnT::FAILURE;
}

CallbackReturnT
OptitrackDriverNode::on_shutdown(const rclcpp_lifecycle::State & state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Shutted down!\n");

  if (disconnect_optitrack()) {
    return ControlledLifecycleNode::on_shutdown(state);
  }
  return CallbackReturnT::FAILURE;
}

CallbackReturnT
OptitrackDriverNode::on_error(const rclcpp_lifecycle::State & state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "State id [%d]", get_current_state().id());
  RCLCPP_INFO(get_logger(), "State label [%s]", get_current_state().label().c_str());

  disconnect_optitrack();
  return ControlledLifecycleNode::on_error(state);
}

bool OptitrackDriverNode::connect_optitrack()
{
  RCLCPP_INFO(
    get_logger(),
    "Trying to connect to Optitrack NatNET SDK at %s ...", server_address_.c_str());

  client->Disconnect();
  set_settings_optitrack();

  if (client->Connect(client_params) != ErrorCode::ErrorCode_OK) {
    RCLCPP_INFO(get_logger(), "... not connected :( ");
    return false;
  }

  RCLCPP_INFO(get_logger(), "... connected!");

  memset(&server_description, 0, sizeof(server_description));
  client->GetServerDescription(&server_description);
  if (!server_description.HostPresent) {
    RCLCPP_DEBUG(get_logger(), "Unable to connect to server. Host not present.");
    return false;
  }

  data_descriptions = nullptr;
  ErrorCode data_desc_result = client->GetDataDescriptionList(&data_descriptions);
  if (data_desc_result != ErrorCode_OK || data_descriptions == nullptr) {
    RCLCPP_DEBUG(get_logger(), "[Client] Unable to retrieve Data Descriptions.\n");
  }

  RCLCPP_INFO(get_logger(), "\n[Client] Server application info:\n");
  RCLCPP_INFO(
    get_logger(), "Application: %s (ver. %d.%d.%d.%d)\n",
    server_description.szHostApp, server_description.HostAppVersion[0],
    server_description.HostAppVersion[1], server_description.HostAppVersion[2],
    server_description.HostAppVersion[3]);
  RCLCPP_INFO(
    get_logger(), "NatNet Version: %d.%d.%d.%d\n", server_description.NatNetVersion[0],
    server_description.NatNetVersion[1],
    server_description.NatNetVersion[2], server_description.NatNetVersion[3]);
  RCLCPP_INFO(get_logger(), "Client IP:%s\n", client_params.localAddress);
  RCLCPP_INFO(get_logger(), "Server IP:%s\n", client_params.serverAddress);
  RCLCPP_INFO(get_logger(), "Server Name:%s\n", server_description.szHostComputerName);

  void * frame_rate_data = nullptr;
  int msg_size = 0;

  if (client->SendMessageAndWait("FrameRate", &frame_rate_data, &msg_size) == ErrorCode_OK) {
    float frame_rate = *(static_cast<float *>(frame_rate_data));
    RCLCPP_INFO(get_logger(), "Mocap Framerate : %3.2f\n", frame_rate);
  } else {
    RCLCPP_DEBUG(get_logger(), "Error getting frame rate.\n");
  }

  return true;
}

bool OptitrackDriverNode::disconnect_optitrack()
{
  void * response_data = nullptr;
  int response_size = 0;
  if (client->SendMessageAndWait("Disconnect", &response_data, &response_size) == ErrorCode_OK) {
    client->Disconnect();
    RCLCPP_INFO(get_logger(), "[Client] Disconnected");
    return true;
  }
  RCLCPP_ERROR(get_logger(), "[Client] Disconnect not successful..");
  return false;
}

void OptitrackDriverNode::init_parameters()
{
  get_parameter<std::string>("connection_type", connection_type_);
  get_parameter<std::string>("server_address", server_address_);
  get_parameter<std::string>("local_address", local_address_);
  get_parameter<std::string>("multicast_address", multicast_address_);
  get_parameter<uint16_t>("server_command_port", server_command_port_);
  get_parameter<uint16_t>("server_data_port", server_data_port_);
}

}  // namespace mocap4r2_optitrack_driver
