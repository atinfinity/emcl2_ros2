// Copyright 2022 Ryuichi Ueda ryuichiueda@gmail.com
// SPDX-FileCopyrightText: 2022 Ryuichi Ueda ryuichiueda@gmail.com
// SPDX-License-Identifier: LGPL-3.0-or-later
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.
// CAUTION: Some lines came from amcl (LGPL).

#ifndef EMCL2__EMCL2_NODE_HPP_
#define EMCL2__EMCL2_NODE_HPP_

#include <message_filters/subscriber.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav2_msgs/msg/particle_cloud.hpp>
#include <nav2_msgs/srv/set_initial_pose.hpp>
#include <nav2_util/lifecycle_node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_srvs/srv/empty.hpp>

#include "emcl2/ExpResetMcl2.hpp"
#include "emcl2/LikelihoodFieldMap.hpp"
#include "emcl2/OdomModel.hpp"

namespace emcl2
{
class EMcl2Node : public nav2_util::LifecycleNode
{
public:
  explicit EMcl2Node(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~EMcl2Node();

  void loop(void);

protected:
  // Managed-node lifecycle transitions (see nav2_util::LifecycleNode).
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  std::shared_ptr<ExpResetMcl2> pf_;

  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseArray>::SharedPtr
    particlecloud_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav2_msgs::msg::ParticleCloud>::SharedPtr
    particle_cloud_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    pose_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float32>::SharedPtr alpha_pub_;
  // The scan is fed through a tf2 MessageFilter so the callback only fires once
  // the odom->base transform at the scan's timestamp is available. Processing the
  // scan directly would race the transform and drop most updates.
  std::shared_ptr<
    message_filters::Subscriber<sensor_msgs::msg::LaserScan, rclcpp_lifecycle::LifecycleNode>>
  laser_scan_sub_;
  std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>> laser_scan_filter_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    initial_pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;

  // ros::ServiceServer global_loc_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr global_loc_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reinit_global_loc_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr nomotion_update_srv_;
  rclcpp::Service<nav2_msgs::srv::SetInitialPose>::SharedPtr set_initial_pose_srv_;
  rclcpp::Time scan_time_stamp_;

  std::string footprint_frame_id_;
  std::string global_frame_id_;
  std::string odom_frame_id_;
  std::string scan_frame_id_;
  std::string base_frame_id_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tfb_;
  std::shared_ptr<tf2_ros::TransformListener> tfl_;
  std::shared_ptr<tf2_ros::Buffer> tf_;

  tf2::Transform latest_tf_;

  bool active_;
  bool init_pf_;
  bool init_request_;
  bool initialpose_receive_;
  bool simple_reset_request_;
  bool scan_receive_;
  bool map_receive_;
  double init_x_, init_y_, init_t_;
  double transform_tolerance_;

  void publishPose(
    double x, double y, double t, double x_dev, double y_dev, double t_dev, double xy_cov,
    double yt_cov, double tx_cov);
  void publishOdomFrame(double x, double y, double t);
  void publishParticles(void);
  bool getOdomPose(double & x, double & y, double & yaw);        // same name is found in amcl
  bool getLidarPose(double & x, double & y, double & yaw, bool & inv);
  void receiveMap(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg);

  void declareParameter();

  void initCommunication(void);
  void initTF();
  void initPF(void);
  std::shared_ptr<LikelihoodFieldMap> initMap(void);
  std::shared_ptr<OdomModel> initOdometry(void);

  nav_msgs::msg::OccupancyGrid map_;

  void cbScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg);
  void cbSimpleReset(
    const std_srvs::srv::Empty::Request::ConstSharedPtr,
    std_srvs::srv::Empty::Response::SharedPtr);
  void cbNomotionUpdate(
    const std_srvs::srv::Empty::Request::ConstSharedPtr,
    std_srvs::srv::Empty::Response::SharedPtr);
  void cbSetInitialPose(
    const nav2_msgs::srv::SetInitialPose::Request::ConstSharedPtr req,
    nav2_msgs::srv::SetInitialPose::Response::SharedPtr);
  void setInitialPose(double x, double y, double t);
  void initialPoseReceived(
    const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr
    msg);                                 // same name is found in amcl
};

}  // namespace emcl2

#endif  // EMCL2__EMCL2_NODE_HPP_
