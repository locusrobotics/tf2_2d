/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Locus Robotics
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef TF2_2D__TF2_2D_HPP_
#define TF2_2D__TF2_2D_HPP_

#include <Eigen/Core>
#include <tf2_ros/buffer_interface.h>

#include <array>
#include <cmath>

#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <tf2/convert.hpp>
#include <tf2/transform_datatypes.hpp>
#include <tf2/utils.hpp>
#include <tf2_2d/rotation.hpp>
#include <tf2_2d/transform.hpp>
#include <tf2_2d/vector2.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <boost/array.hpp>

/**
 * This file contains conversion functions from standard ROS message types to/from the 2D geometry objects. These are
 * designed to work with the tf2 data conversion system: http://wiki.ros.org/tf2/Tutorials/Migration/DataConversions
 */

namespace tf2
{

inline void toMsg(const tf2_2d::Vector2 & in, geometry_msgs::msg::Vector3 & msg)
{
  msg.x = in.x();
  msg.y = in.y();
  msg.z = 0;
}

inline void toMsg(const tf2_2d::Vector2 & in, geometry_msgs::msg::Point & msg)
{
  msg.x = in.x();
  msg.y = in.y();
  msg.z = 0;
}

inline void toMsg(const tf2_2d::Vector2 & in, geometry_msgs::msg::Point32 & msg)
{
  msg.x = in.x();
  msg.y = in.y();
  msg.z = 0;
}

// You can only choose one destination message type with this signature
inline geometry_msgs::msg::Vector3 toMsg(const tf2_2d::Vector2 & in)
{
  geometry_msgs::msg::Vector3 msg;
  toMsg(in, msg);
  return msg;
}

inline void fromMsg(const geometry_msgs::msg::Vector3 & msg, tf2_2d::Vector2 & out)
{
  out.setX(msg.x);
  out.setY(msg.y);
}

inline void fromMsg(const geometry_msgs::msg::Point & msg, tf2_2d::Vector2 & out)
{
  out.setX(msg.x);
  out.setY(msg.y);
}

inline void fromMsg(const geometry_msgs::msg::Point32 & msg, tf2_2d::Vector2 & out)
{
  out.setX(msg.x);
  out.setY(msg.y);
}

inline void toMsg(
  const tf2::Stamped<tf2_2d::Vector2> & in,
  geometry_msgs::msg::Vector3Stamped & msg)
{
  msg.header.stamp = tf2_ros::toMsg(in.stamp_);
  msg.header.frame_id = in.frame_id_;
  toMsg(static_cast<const tf2_2d::Vector2 &>(in), msg.vector);
}

inline void toMsg(const tf2::Stamped<tf2_2d::Vector2> & in, geometry_msgs::msg::PointStamped & msg)
{
  msg.header.stamp = tf2_ros::toMsg(in.stamp_);
  msg.header.frame_id = in.frame_id_;
  toMsg(static_cast<const tf2_2d::Vector2 &>(in), msg.point);
}

inline geometry_msgs::msg::Vector3Stamped toMsg(const tf2::Stamped<tf2_2d::Vector2> & in)
{
  geometry_msgs::msg::Vector3Stamped msg;
  toMsg(in, msg);
  return msg;
}

inline void fromMsg(
  const geometry_msgs::msg::Vector3Stamped & msg,
  tf2::Stamped<tf2_2d::Vector2> & out)
{
  out.stamp_ = tf2_ros::fromMsg(msg.header.stamp);
  out.frame_id_ = msg.header.frame_id;
  fromMsg(msg.vector, static_cast<tf2_2d::Vector2 &>(out));
}

inline void fromMsg(
  const geometry_msgs::msg::PointStamped & msg,
  tf2::Stamped<tf2_2d::Vector2> & out)
{
  out.stamp_ = tf2_ros::fromMsg(msg.header.stamp);
  out.frame_id_ = msg.header.frame_id;
  fromMsg(msg.point, static_cast<tf2_2d::Vector2 &>(out));
}

inline void toMsg(const tf2_2d::Rotation & in, geometry_msgs::msg::Quaternion & msg)
{
  msg.x = 0;
  msg.y = 0;
  msg.z = std::sin(0.5 * in.angle());
  msg.w = std::cos(0.5 * in.angle());
}

inline geometry_msgs::msg::Quaternion toMsg(const tf2_2d::Rotation & in)
{
  geometry_msgs::msg::Quaternion msg;
  toMsg(in, msg);
  return msg;
}

inline void fromMsg(const geometry_msgs::msg::Quaternion & msg, tf2_2d::Rotation & out)
{
  out.setAngle(tf2::getYaw(msg));
}

inline void toMsg(
  const tf2::Stamped<tf2_2d::Rotation> & in,
  geometry_msgs::msg::QuaternionStamped & msg)
{
  msg.header.stamp = tf2_ros::toMsg(in.stamp_);
  msg.header.frame_id = in.frame_id_;
  toMsg(static_cast<const tf2_2d::Rotation &>(in), msg.quaternion);
}

inline geometry_msgs::msg::QuaternionStamped toMsg(const tf2::Stamped<tf2_2d::Rotation> & in)
{
  geometry_msgs::msg::QuaternionStamped msg;
  toMsg(in, msg);
  return msg;
}

inline void fromMsg(
  const geometry_msgs::msg::QuaternionStamped & msg,
  tf2::Stamped<tf2_2d::Rotation> & out)
{
  out.stamp_ = tf2_ros::fromMsg(msg.header.stamp);
  out.frame_id_ = msg.header.frame_id;
  fromMsg(msg.quaternion, static_cast<tf2_2d::Rotation &>(out));
}

inline void toMsg(const tf2_2d::Transform & in, geometry_msgs::msg::Transform & msg)
{
  toMsg(in.translation(), msg.translation);
  toMsg(in.rotation(), msg.rotation);
}

inline void toMsg(const tf2_2d::Transform & in, geometry_msgs::msg::Pose & msg)
{
  toMsg(in.translation(), msg.position);
  toMsg(in.rotation(), msg.orientation);
}

inline void toMsg(const tf2_2d::Transform & in, geometry_msgs::msg::Pose2D & msg)
{
  msg.x = in.x();
  msg.y = in.y();
  msg.theta = in.theta();
}

inline geometry_msgs::msg::Transform toMsg(const tf2_2d::Transform & in)
{
  geometry_msgs::msg::Transform msg;
  toMsg(in, msg);
  return msg;
}

inline void fromMsg(const geometry_msgs::msg::Transform & msg, tf2_2d::Transform & out)
{
  tf2_2d::Rotation rotation;
  fromMsg(msg.rotation, rotation);
  out.setRotation(rotation);
  tf2_2d::Vector2 translation;
  fromMsg(msg.translation, translation);
  out.setTranslation(translation);
}

inline void fromMsg(const geometry_msgs::msg::Pose & msg, tf2_2d::Transform & out)
{
  tf2_2d::Rotation rotation;
  fromMsg(msg.orientation, rotation);
  out.setRotation(rotation);
  tf2_2d::Vector2 translation;
  fromMsg(msg.position, translation);
  out.setTranslation(translation);
}

inline void fromMsg(const geometry_msgs::msg::Pose2D & msg, tf2_2d::Transform & out)
{
  out.setAngle(msg.theta);
  out.setX(msg.x);
  out.setY(msg.y);
}

inline void toMsg(
  const tf2::Stamped<tf2_2d::Transform> & in,
  geometry_msgs::msg::TransformStamped & msg)
{
  msg.header.stamp = tf2_ros::toMsg(in.stamp_);
  msg.header.frame_id = in.frame_id_;
  toMsg(static_cast<const tf2_2d::Transform &>(in), msg.transform);
}

inline void toMsg(const tf2::Stamped<tf2_2d::Transform> & in, geometry_msgs::msg::PoseStamped & msg)
{
  msg.header.stamp = tf2_ros::toMsg(in.stamp_);
  msg.header.frame_id = in.frame_id_;
  toMsg(static_cast<const tf2_2d::Transform &>(in), msg.pose);
}

inline geometry_msgs::msg::TransformStamped toMsg(const tf2::Stamped<tf2_2d::Transform> & in)
{
  geometry_msgs::msg::TransformStamped msg;
  msg.header.stamp = tf2_ros::toMsg(in.stamp_);
  msg.header.frame_id = in.frame_id_;
  msg.transform = toMsg(static_cast<const tf2_2d::Transform &>(in));
  return msg;
}

inline void fromMsg(
  const geometry_msgs::msg::TransformStamped & msg,
  tf2::Stamped<tf2_2d::Transform> & out)
{
  out.stamp_ = tf2_ros::fromMsg(msg.header.stamp);
  out.frame_id_ = msg.header.frame_id;
  fromMsg(msg.transform, static_cast<tf2_2d::Transform &>(out));
}

inline void fromMsg(
  const geometry_msgs::msg::PoseStamped & msg,
  tf2::Stamped<tf2_2d::Transform> & out)
{
  out.stamp_ = tf2_ros::fromMsg(msg.header.stamp);
  out.frame_id_ = msg.header.frame_id;
  fromMsg(msg.pose, static_cast<tf2_2d::Transform &>(out));
}


inline
Eigen::Matrix3d transformCovariance(
  const Eigen::Matrix3d & cov_in,
  const tf2_2d::Transform & transform)
{
  Eigen::Matrix3d R = transform.rotation().getHomogeneousMatrix();
  return R * cov_in * R.transpose();
}

inline
std::array<double, 9> transformCovariance(
  const std::array<double, 9> & cov_in,
  const tf2_2d::Transform & transform)
{
  Eigen::Matrix3d R = transform.rotation().getHomogeneousMatrix();
  std::array<double, 9> cov_out;
  Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> cov_out_map(cov_out.data());
  Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> cov_in_map(cov_in.data());
  cov_out_map = R * cov_in_map * R.transpose();
  return cov_out;
}

inline
boost::array<double, 9> transformCovariance(
  const boost::array<double, 9> & cov_in,
  const tf2_2d::Transform & transform)
{
  Eigen::Matrix3d R = transform.rotation().getHomogeneousMatrix();
  boost::array<double, 9> cov_out;
  Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> cov_out_map(cov_out.data());
  Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> cov_in_map(cov_in.data());
  cov_out_map = R * cov_in_map * R.transpose();
  return cov_out;
}

}  // namespace tf2

#endif  // TF2_2D__TF2_2D_HPP_
