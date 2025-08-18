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
#include <Eigen/Core>
#include <gtest/gtest.h>

#include <boost/array.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <rclcpp/time.hpp>
#include <tf2/transform_datatypes.hpp>
#include <tf2_2d/rotation.hpp>
#include <tf2_2d/tf2_2d.hpp>
#include <tf2_2d/transform.hpp>
#include <tf2_2d/vector2.hpp>
#include <tf2_ros/buffer_interface.hpp>


TEST(Conversions, Vector2)
{
  // Test conversion to/from the Vector2 type
  {
    tf2_2d::Vector2 in(1.0, 2.0);
    geometry_msgs::msg::Vector3 actual;
    tf2::toMsg(in, actual);
    geometry_msgs::msg::Vector3 expected;
    expected.x = 1.0;
    expected.y = 2.0;
    expected.z = 0.0;
    EXPECT_EQ(expected.x, actual.x);
    EXPECT_EQ(expected.y, actual.y);
    EXPECT_EQ(expected.z, actual.z);
  }
  {
    tf2_2d::Vector2 in(1.0, 2.0);
    geometry_msgs::msg::Point actual;
    tf2::toMsg(in, actual);
    geometry_msgs::msg::Point expected;
    expected.x = 1.0;
    expected.y = 2.0;
    expected.z = 0.0;
    EXPECT_EQ(expected.x, actual.x);
    EXPECT_EQ(expected.y, actual.y);
    EXPECT_EQ(expected.z, actual.z);
  }
  {
    tf2_2d::Vector2 in(1.0, 2.0);
    geometry_msgs::msg::Point32 actual;
    tf2::toMsg(in, actual);
    geometry_msgs::msg::Point32 expected;
    expected.x = 1.0;
    expected.y = 2.0;
    expected.z = 0.0;
    EXPECT_EQ(expected.x, actual.x);
    EXPECT_EQ(expected.y, actual.y);
    EXPECT_EQ(expected.z, actual.z);
  }
  {
    tf2_2d::Vector2 in(1.0, 2.0);
    geometry_msgs::msg::Vector3 actual = tf2::toMsg(in);
    geometry_msgs::msg::Vector3 expected;
    expected.x = 1.0;
    expected.y = 2.0;
    expected.z = 0.0;
    EXPECT_EQ(expected.x, actual.x);
    EXPECT_EQ(expected.y, actual.y);
    EXPECT_EQ(expected.z, actual.z);
  }
  {
    geometry_msgs::msg::Vector3 msg;
    msg.x = 1.0;
    msg.y = 2.0;
    msg.z = 3.0;
    tf2_2d::Vector2 actual;
    tf2::fromMsg(msg, actual);
    tf2_2d::Vector2 expected(1.0, 2.0);
    EXPECT_EQ(expected.x(), actual.x());
    EXPECT_EQ(expected.y(), actual.y());
  }
  {
    geometry_msgs::msg::Point msg;
    msg.x = 1.0;
    msg.y = 2.0;
    msg.z = 3.0;
    tf2_2d::Vector2 actual;
    tf2::fromMsg(msg, actual);
    tf2_2d::Vector2 expected(1.0, 2.0);
    EXPECT_EQ(expected.x(), actual.x());
    EXPECT_EQ(expected.y(), actual.y());
  }
  {
    geometry_msgs::msg::Point32 msg;
    msg.x = 1.0;
    msg.y = 2.0;
    msg.z = 3.0;
    tf2_2d::Vector2 actual;
    tf2::fromMsg(msg, actual);
    tf2_2d::Vector2 expected(1.0, 2.0);
    EXPECT_EQ(expected.x(), actual.x());
    EXPECT_EQ(expected.y(), actual.y());
  }
  {
    tf2::Stamped<tf2_2d::Vector2> in(tf2_2d::Vector2(1.0, 2.0),
      tf2_ros::fromRclcpp(rclcpp::Time(1234, 5678)), "test_frame");
    geometry_msgs::msg::Vector3Stamped actual;
    tf2::toMsg(in, actual);
    geometry_msgs::msg::Vector3Stamped expected;
    expected.header.stamp = rclcpp::Time(1234, 5678);
    expected.header.frame_id = "test_frame";
    expected.vector.x = 1.0;
    expected.vector.y = 2.0;
    expected.vector.z = 0.0;
    EXPECT_EQ(expected.header.stamp, actual.header.stamp);
    EXPECT_EQ(expected.header.frame_id, actual.header.frame_id);
    EXPECT_EQ(expected.vector.x, actual.vector.x);
    EXPECT_EQ(expected.vector.y, actual.vector.y);
    EXPECT_EQ(expected.vector.z, actual.vector.z);
  }
  {
    tf2::Stamped<tf2_2d::Vector2> in(tf2_2d::Vector2(1.0, 2.0),
      tf2_ros::fromRclcpp(rclcpp::Time(1234, 5678)), "test_frame");
    geometry_msgs::msg::PointStamped actual;
    tf2::toMsg(in, actual);
    geometry_msgs::msg::PointStamped expected;
    expected.header.stamp = rclcpp::Time(1234, 5678);
    expected.header.frame_id = "test_frame";
    expected.point.x = 1.0;
    expected.point.y = 2.0;
    expected.point.z = 0.0;
    EXPECT_EQ(expected.header.stamp, actual.header.stamp);
    EXPECT_EQ(expected.header.frame_id, actual.header.frame_id);
    EXPECT_EQ(expected.point.x, actual.point.x);
    EXPECT_EQ(expected.point.y, actual.point.y);
    EXPECT_EQ(expected.point.z, actual.point.z);
  }
  {
    tf2::Stamped<tf2_2d::Vector2> in(tf2_2d::Vector2(1.0, 2.0),
      tf2_ros::fromRclcpp(rclcpp::Time(1234, 5678)), "test_frame");
    geometry_msgs::msg::Vector3Stamped actual = tf2::toMsg(in);
    geometry_msgs::msg::Vector3Stamped expected;
    expected.header.stamp = rclcpp::Time(1234, 5678);
    expected.header.frame_id = "test_frame";
    expected.vector.x = 1.0;
    expected.vector.y = 2.0;
    expected.vector.z = 0.0;
    EXPECT_EQ(expected.header.stamp, actual.header.stamp);
    EXPECT_EQ(expected.header.frame_id, actual.header.frame_id);
    EXPECT_EQ(expected.vector.x, actual.vector.x);
    EXPECT_EQ(expected.vector.y, actual.vector.y);
    EXPECT_EQ(expected.vector.z, actual.vector.z);
  }
  {
    geometry_msgs::msg::Vector3Stamped msg;
    msg.header.stamp = rclcpp::Time(1234, 5678);
    msg.header.frame_id = "test_frame";
    msg.vector.x = 1.0;
    msg.vector.y = 2.0;
    msg.vector.z = 3.0;
    tf2::Stamped<tf2_2d::Vector2> actual;
    tf2::fromMsg(msg, actual);
    tf2::Stamped<tf2_2d::Vector2> expected(tf2_2d::Vector2(1.0, 2.0),
      tf2_ros::fromRclcpp(rclcpp::Time(1234, 5678)), "test_frame");
    EXPECT_EQ(expected.stamp_, actual.stamp_);
    EXPECT_EQ(expected.frame_id_, actual.frame_id_);
    EXPECT_EQ(expected.x(), actual.x());
    EXPECT_EQ(expected.y(), actual.y());
  }
  {
    geometry_msgs::msg::PointStamped msg;
    msg.header.stamp = rclcpp::Time(1234, 5678);
    msg.header.frame_id = "test_frame";
    msg.point.x = 1.0;
    msg.point.y = 2.0;
    msg.point.z = 3.0;
    tf2::Stamped<tf2_2d::Vector2> actual;
    tf2::fromMsg(msg, actual);
    tf2::Stamped<tf2_2d::Vector2> expected(tf2_2d::Vector2(1.0, 2.0),
      tf2_ros::fromRclcpp(rclcpp::Time(1234, 5678)), "test_frame");
    EXPECT_EQ(expected.stamp_, actual.stamp_);
    EXPECT_EQ(expected.frame_id_, actual.frame_id_);
    EXPECT_EQ(expected.x(), actual.x());
    EXPECT_EQ(expected.y(), actual.y());
  }
}

TEST(Conversions, Rotation)
{
  // Test conversion to/from the Rotation type
  {
    tf2_2d::Rotation in(2.0);
    geometry_msgs::msg::Quaternion actual;
    tf2::toMsg(in, actual);
    geometry_msgs::msg::Quaternion expected;
    expected.x = 0.0;
    expected.y = 0.0;
    expected.z = 0.841470984807897;
    expected.w = 0.540302305868140;
    EXPECT_NEAR(expected.x, actual.x, 1.0e-9);
    EXPECT_NEAR(expected.y, actual.y, 1.0e-9);
    EXPECT_NEAR(expected.z, actual.z, 1.0e-9);
    EXPECT_NEAR(expected.w, actual.w, 1.0e-9);
  }
  {
    tf2_2d::Rotation in(2.0);
    geometry_msgs::msg::Quaternion actual = tf2::toMsg(in);
    geometry_msgs::msg::Quaternion expected;
    expected.x = 0.0;
    expected.y = 0.0;
    expected.z = 0.841470984807897;
    expected.w = 0.540302305868140;
    EXPECT_NEAR(expected.x, actual.x, 1.0e-9);
    EXPECT_NEAR(expected.y, actual.y, 1.0e-9);
    EXPECT_NEAR(expected.z, actual.z, 1.0e-9);
    EXPECT_NEAR(expected.w, actual.w, 1.0e-9);
  }
  {
    geometry_msgs::msg::Quaternion msg;
    msg.x = 0.0;
    msg.y = 0.0;
    msg.z = 0.841470984807897;
    msg.w = 0.540302305868140;
    tf2_2d::Rotation actual;
    tf2::fromMsg(msg, actual);
    tf2_2d::Rotation expected(2.0);
    EXPECT_NEAR(expected.angle(), actual.angle(), 1.0e-9);
  }
  {
    tf2::Stamped<tf2_2d::Rotation> in(tf2_2d::Rotation(2.0),
      tf2_ros::fromRclcpp(rclcpp::Time(1234, 5678)), "test_frame");
    geometry_msgs::msg::QuaternionStamped actual;
    tf2::toMsg(in, actual);
    geometry_msgs::msg::QuaternionStamped expected;
    expected.header.stamp = rclcpp::Time(1234, 5678);
    expected.header.frame_id = "test_frame";
    expected.quaternion.x = 0.0;
    expected.quaternion.y = 0.0;
    expected.quaternion.z = 0.841470984807897;
    expected.quaternion.w = 0.540302305868140;
    EXPECT_EQ(expected.header.stamp, actual.header.stamp);
    EXPECT_EQ(expected.header.frame_id, actual.header.frame_id);
    EXPECT_NEAR(expected.quaternion.x, actual.quaternion.x, 1.0e-9);
    EXPECT_NEAR(expected.quaternion.y, actual.quaternion.y, 1.0e-9);
    EXPECT_NEAR(expected.quaternion.z, actual.quaternion.z, 1.0e-9);
    EXPECT_NEAR(expected.quaternion.w, actual.quaternion.w, 1.0e-9);
  }
  {
    tf2::Stamped<tf2_2d::Rotation> in(tf2_2d::Rotation(2.0),
      tf2_ros::fromRclcpp(rclcpp::Time(1234, 5678)), "test_frame");
    geometry_msgs::msg::QuaternionStamped actual = tf2::toMsg(in);
    geometry_msgs::msg::QuaternionStamped expected;
    expected.header.stamp = rclcpp::Time(1234, 5678);
    expected.header.frame_id = "test_frame";
    expected.quaternion.x = 0.0;
    expected.quaternion.y = 0.0;
    expected.quaternion.z = 0.841470984807897;
    expected.quaternion.w = 0.540302305868140;
    EXPECT_EQ(expected.header.stamp, actual.header.stamp);
    EXPECT_EQ(expected.header.frame_id, actual.header.frame_id);
    EXPECT_NEAR(expected.quaternion.x, actual.quaternion.x, 1.0e-9);
    EXPECT_NEAR(expected.quaternion.y, actual.quaternion.y, 1.0e-9);
    EXPECT_NEAR(expected.quaternion.z, actual.quaternion.z, 1.0e-9);
    EXPECT_NEAR(expected.quaternion.w, actual.quaternion.w, 1.0e-9);
  }
  {
    geometry_msgs::msg::QuaternionStamped msg;
    msg.header.stamp = rclcpp::Time(1234, 5678);
    msg.header.frame_id = "test_frame";
    msg.quaternion.x = 0.0;
    msg.quaternion.y = 0.0;
    msg.quaternion.z = 0.841470984807897;
    msg.quaternion.w = 0.540302305868140;
    tf2::Stamped<tf2_2d::Rotation> actual;
    tf2::fromMsg(msg, actual);
    tf2::Stamped<tf2_2d::Rotation> expected(tf2_2d::Rotation(2.0),
      tf2_ros::fromRclcpp(rclcpp::Time(1234, 5678)), "test_frame");
    EXPECT_EQ(expected.stamp_, actual.stamp_);
    EXPECT_EQ(expected.frame_id_, actual.frame_id_);
    EXPECT_NEAR(expected.angle(), actual.angle(), 1.0e-9);
  }
}

TEST(Conversions, Transform)
{
  // Test conversion to/from the Transform type
  {
    tf2_2d::Transform in(1.0, 1.5, 2.0);
    geometry_msgs::msg::Transform actual;
    tf2::toMsg(in, actual);
    geometry_msgs::msg::Transform expected;
    expected.translation.x = 1.0;
    expected.translation.y = 1.5;
    expected.translation.z = 0;
    expected.rotation.x = 0.0;
    expected.rotation.y = 0.0;
    expected.rotation.z = 0.841470984807897;
    expected.rotation.w = 0.540302305868140;
    EXPECT_NEAR(expected.translation.x, actual.translation.x, 1.0e-9);
    EXPECT_NEAR(expected.translation.y, actual.translation.y, 1.0e-9);
    EXPECT_NEAR(expected.translation.z, actual.translation.z, 1.0e-9);
    EXPECT_NEAR(expected.rotation.x, actual.rotation.x, 1.0e-9);
    EXPECT_NEAR(expected.rotation.y, actual.rotation.y, 1.0e-9);
    EXPECT_NEAR(expected.rotation.z, actual.rotation.z, 1.0e-9);
    EXPECT_NEAR(expected.rotation.w, actual.rotation.w, 1.0e-9);
  }
  {
    tf2_2d::Transform in(1.0, 1.5, 2.0);
    geometry_msgs::msg::Pose actual;
    tf2::toMsg(in, actual);
    geometry_msgs::msg::Pose expected;
    expected.position.x = 1.0;
    expected.position.y = 1.5;
    expected.position.z = 0;
    expected.orientation.x = 0.0;
    expected.orientation.y = 0.0;
    expected.orientation.z = 0.841470984807897;
    expected.orientation.w = 0.540302305868140;
    EXPECT_NEAR(expected.position.x, actual.position.x, 1.0e-9);
    EXPECT_NEAR(expected.position.y, actual.position.y, 1.0e-9);
    EXPECT_NEAR(expected.position.z, actual.position.z, 1.0e-9);
    EXPECT_NEAR(expected.orientation.x, actual.orientation.x, 1.0e-9);
    EXPECT_NEAR(expected.orientation.y, actual.orientation.y, 1.0e-9);
    EXPECT_NEAR(expected.orientation.z, actual.orientation.z, 1.0e-9);
    EXPECT_NEAR(expected.orientation.w, actual.orientation.w, 1.0e-9);
  }
  {
    tf2_2d::Transform in(1.0, 1.5, 2.0);
    geometry_msgs::msg::Transform actual = tf2::toMsg(in);
    geometry_msgs::msg::Transform expected;
    expected.translation.x = 1.0;
    expected.translation.y = 1.5;
    expected.translation.z = 0;
    expected.rotation.x = 0.0;
    expected.rotation.y = 0.0;
    expected.rotation.z = 0.841470984807897;
    expected.rotation.w = 0.540302305868140;
    EXPECT_NEAR(expected.translation.x, actual.translation.x, 1.0e-9);
    EXPECT_NEAR(expected.translation.y, actual.translation.y, 1.0e-9);
    EXPECT_NEAR(expected.translation.z, actual.translation.z, 1.0e-9);
    EXPECT_NEAR(expected.rotation.x, actual.rotation.x, 1.0e-9);
    EXPECT_NEAR(expected.rotation.y, actual.rotation.y, 1.0e-9);
    EXPECT_NEAR(expected.rotation.z, actual.rotation.z, 1.0e-9);
    EXPECT_NEAR(expected.rotation.w, actual.rotation.w, 1.0e-9);
  }
  {
    geometry_msgs::msg::Transform msg;
    msg.translation.x = 1.0;
    msg.translation.y = 1.5;
    msg.translation.z = 0;
    msg.rotation.x = 0.0;
    msg.rotation.y = 0.0;
    msg.rotation.z = 0.841470984807897;
    msg.rotation.w = 0.540302305868140;
    tf2_2d::Transform actual;
    tf2::fromMsg(msg, actual);
    tf2_2d::Transform expected(1.0, 1.5, 2.0);
    EXPECT_NEAR(expected.x(), actual.x(), 1.0e-9);
    EXPECT_NEAR(expected.y(), actual.y(), 1.0e-9);
    EXPECT_NEAR(expected.angle(), actual.angle(), 1.0e-9);
  }
  {
    geometry_msgs::msg::Pose msg;
    msg.position.x = 1.0;
    msg.position.y = 1.5;
    msg.position.z = 0;
    msg.orientation.x = 0.0;
    msg.orientation.y = 0.0;
    msg.orientation.z = 0.841470984807897;
    msg.orientation.w = 0.540302305868140;
    tf2_2d::Transform actual;
    tf2::fromMsg(msg, actual);
    tf2_2d::Transform expected(1.0, 1.5, 2.0);
    EXPECT_NEAR(expected.x(), actual.x(), 1.0e-9);
    EXPECT_NEAR(expected.y(), actual.y(), 1.0e-9);
    EXPECT_NEAR(expected.angle(), actual.angle(), 1.0e-9);
  }
  {
    tf2::Stamped<tf2_2d::Transform> in(tf2_2d::Transform(1.0, 1.5, 2.0),
      tf2_ros::fromRclcpp(rclcpp::Time(1234, 5678)), "test_frame");
    geometry_msgs::msg::TransformStamped actual;
    tf2::toMsg(in, actual);
    geometry_msgs::msg::TransformStamped expected;
    expected.header.stamp = rclcpp::Time(1234, 5678);
    expected.header.frame_id = "test_frame";
    expected.transform.translation.x = 1.0;
    expected.transform.translation.y = 1.5;
    expected.transform.translation.z = 0;
    expected.transform.rotation.x = 0.0;
    expected.transform.rotation.y = 0.0;
    expected.transform.rotation.z = 0.841470984807897;
    expected.transform.rotation.w = 0.540302305868140;
    EXPECT_EQ(expected.header.stamp, actual.header.stamp);
    EXPECT_EQ(expected.header.frame_id, actual.header.frame_id);
    EXPECT_NEAR(expected.transform.translation.x, actual.transform.translation.x, 1.0e-9);
    EXPECT_NEAR(expected.transform.translation.y, actual.transform.translation.y, 1.0e-9);
    EXPECT_NEAR(expected.transform.translation.z, actual.transform.translation.z, 1.0e-9);
    EXPECT_NEAR(expected.transform.rotation.x, actual.transform.rotation.x, 1.0e-9);
    EXPECT_NEAR(expected.transform.rotation.y, actual.transform.rotation.y, 1.0e-9);
    EXPECT_NEAR(expected.transform.rotation.z, actual.transform.rotation.z, 1.0e-9);
    EXPECT_NEAR(expected.transform.rotation.w, actual.transform.rotation.w, 1.0e-9);
  }
  {
    tf2::Stamped<tf2_2d::Transform> in(tf2_2d::Transform(1.0, 1.5, 2.0),
      tf2_ros::fromRclcpp(rclcpp::Time(1234, 5678)), "test_frame");
    geometry_msgs::msg::PoseStamped actual;
    tf2::toMsg(in, actual);
    geometry_msgs::msg::PoseStamped expected;
    expected.header.stamp = rclcpp::Time(1234, 5678);
    expected.header.frame_id = "test_frame";
    expected.pose.position.x = 1.0;
    expected.pose.position.y = 1.5;
    expected.pose.position.z = 0;
    expected.pose.orientation.x = 0.0;
    expected.pose.orientation.y = 0.0;
    expected.pose.orientation.z = 0.841470984807897;
    expected.pose.orientation.w = 0.540302305868140;
    EXPECT_EQ(expected.header.stamp, actual.header.stamp);
    EXPECT_EQ(expected.header.frame_id, actual.header.frame_id);
    EXPECT_NEAR(expected.pose.position.x, actual.pose.position.x, 1.0e-9);
    EXPECT_NEAR(expected.pose.position.y, actual.pose.position.y, 1.0e-9);
    EXPECT_NEAR(expected.pose.position.z, actual.pose.position.z, 1.0e-9);
    EXPECT_NEAR(expected.pose.orientation.x, actual.pose.orientation.x, 1.0e-9);
    EXPECT_NEAR(expected.pose.orientation.y, actual.pose.orientation.y, 1.0e-9);
    EXPECT_NEAR(expected.pose.orientation.z, actual.pose.orientation.z, 1.0e-9);
    EXPECT_NEAR(expected.pose.orientation.w, actual.pose.orientation.w, 1.0e-9);
  }
  {
    tf2::Stamped<tf2_2d::Transform> in(tf2_2d::Transform(1.0, 1.5, 2.0),
      tf2_ros::fromRclcpp(rclcpp::Time(1234, 5678)), "test_frame");
    geometry_msgs::msg::TransformStamped actual = tf2::toMsg(in);
    geometry_msgs::msg::TransformStamped expected;
    expected.header.stamp = rclcpp::Time(1234, 5678);
    expected.header.frame_id = "test_frame";
    expected.transform.translation.x = 1.0;
    expected.transform.translation.y = 1.5;
    expected.transform.translation.z = 0;
    expected.transform.rotation.x = 0.0;
    expected.transform.rotation.y = 0.0;
    expected.transform.rotation.z = 0.841470984807897;
    expected.transform.rotation.w = 0.540302305868140;
    EXPECT_EQ(expected.header.stamp, actual.header.stamp);
    EXPECT_EQ(expected.header.frame_id, actual.header.frame_id);
    EXPECT_NEAR(expected.transform.translation.x, actual.transform.translation.x, 1.0e-9);
    EXPECT_NEAR(expected.transform.translation.y, actual.transform.translation.y, 1.0e-9);
    EXPECT_NEAR(expected.transform.translation.z, actual.transform.translation.z, 1.0e-9);
    EXPECT_NEAR(expected.transform.rotation.x, actual.transform.rotation.x, 1.0e-9);
    EXPECT_NEAR(expected.transform.rotation.y, actual.transform.rotation.y, 1.0e-9);
    EXPECT_NEAR(expected.transform.rotation.z, actual.transform.rotation.z, 1.0e-9);
    EXPECT_NEAR(expected.transform.rotation.w, actual.transform.rotation.w, 1.0e-9);
  }
  {
    geometry_msgs::msg::TransformStamped msg;
    msg.header.stamp = rclcpp::Time(1234, 5678);
    msg.header.frame_id = "test_frame";
    msg.transform.translation.x = 1.0;
    msg.transform.translation.y = 1.5;
    msg.transform.translation.z = 0;
    msg.transform.rotation.x = 0.0;
    msg.transform.rotation.y = 0.0;
    msg.transform.rotation.z = 0.841470984807897;
    msg.transform.rotation.w = 0.540302305868140;
    tf2::Stamped<tf2_2d::Transform> actual;
    tf2::fromMsg(msg, actual);
    tf2::Stamped<tf2_2d::Transform> expected(tf2_2d::Transform(1.0, 1.5, 2.0),
      tf2_ros::fromRclcpp(rclcpp::Time(1234, 5678)), "test_frame");
    EXPECT_EQ(expected.stamp_, actual.stamp_);
    EXPECT_EQ(expected.frame_id_, actual.frame_id_);
    EXPECT_NEAR(expected.x(), actual.x(), 1.0e-9);
    EXPECT_NEAR(expected.y(), actual.y(), 1.0e-9);
    EXPECT_NEAR(expected.angle(), actual.angle(), 1.0e-9);
  }
  {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = rclcpp::Time(1234, 5678);
    msg.header.frame_id = "test_frame";
    msg.pose.position.x = 1.0;
    msg.pose.position.y = 1.5;
    msg.pose.position.z = 0;
    msg.pose.orientation.x = 0.0;
    msg.pose.orientation.y = 0.0;
    msg.pose.orientation.z = 0.841470984807897;
    msg.pose.orientation.w = 0.540302305868140;
    tf2::Stamped<tf2_2d::Transform> actual;
    tf2::fromMsg(msg, actual);
    tf2::Stamped<tf2_2d::Transform> expected(tf2_2d::Transform(1.0, 1.5, 2.0),
      tf2_ros::fromRclcpp(rclcpp::Time(1234, 5678)), "test_frame");
    EXPECT_EQ(expected.stamp_, actual.stamp_);
    EXPECT_EQ(expected.frame_id_, actual.frame_id_);
    EXPECT_NEAR(expected.x(), actual.x(), 1.0e-9);
    EXPECT_NEAR(expected.y(), actual.y(), 1.0e-9);
    EXPECT_NEAR(expected.angle(), actual.angle(), 1.0e-9);
  }
}

TEST(Conversions, TransformCovariance)
{
  {
    Eigen::Matrix3d cov_in;
    // *INDENT-OFF*
    cov_in << 1.0, 0.2, 0.3,
              0.2, 2.0, 0.4,
              0.3, 0.4, 3.0;
    // *INDENT-ON*
    tf2_2d::Transform trans(1.0, 1.5, 2.0);
    Eigen::Matrix3d actual = tf2::transformCovariance(cov_in, trans);
    Eigen::Matrix3d expected;
    // *INDENT-OFF*
    expected <<  1.978182309493392,   0.247672523481242,  -0.488563021694415,
                0.247672523481242,   1.021817690506609,   0.106330493428848,
                -0.488563021694415,   0.106330493428848,   3.000000000000000;
    // *INDENT-ON*
    EXPECT_NEAR(expected(0, 0), actual(0, 0), 1.0e-9);
    EXPECT_NEAR(expected(0, 1), actual(0, 1), 1.0e-9);
    EXPECT_NEAR(expected(0, 2), actual(0, 2), 1.0e-9);
    EXPECT_NEAR(expected(1, 0), actual(1, 0), 1.0e-9);
    EXPECT_NEAR(expected(1, 1), actual(1, 1), 1.0e-9);
    EXPECT_NEAR(expected(1, 2), actual(1, 2), 1.0e-9);
    EXPECT_NEAR(expected(2, 0), actual(2, 0), 1.0e-9);
    EXPECT_NEAR(expected(2, 1), actual(2, 1), 1.0e-9);
    EXPECT_NEAR(expected(2, 2), actual(2, 2), 1.0e-9);
  }
  {
    // *INDENT-OFF*
    std::array<double, 9> cov_in{1.0, 0.2, 0.3,  // NOLINT(whitespace/braces)
                                 0.2, 2.0, 0.4,
                                 0.3, 0.4, 3.0};  // NOLINT(whitespace/braces)
    // *INDENT-ON*
    tf2_2d::Transform trans(1.0, 1.5, 2.0);
    std::array<double, 9> actual = tf2::transformCovariance(cov_in, trans);
    // *INDENT-OFF*
    std::array<double, 9> expected{1.978182309493392,   0.247672523481242,  -0.488563021694415,  // NOLINT
                                   0.247672523481242,   1.021817690506609,   0.106330493428848,
                                  -0.488563021694415,   0.106330493428848,   3.000000000000000};  // NOLINT
    // *INDENT-ON*
    EXPECT_NEAR(expected[0], actual[0], 1.0e-9);
    EXPECT_NEAR(expected[1], actual[1], 1.0e-9);
    EXPECT_NEAR(expected[2], actual[2], 1.0e-9);
    EXPECT_NEAR(expected[3], actual[3], 1.0e-9);
    EXPECT_NEAR(expected[4], actual[4], 1.0e-9);
    EXPECT_NEAR(expected[5], actual[5], 1.0e-9);
    EXPECT_NEAR(expected[6], actual[6], 1.0e-9);
    EXPECT_NEAR(expected[7], actual[7], 1.0e-9);
    EXPECT_NEAR(expected[8], actual[8], 1.0e-9);
  }
  {
    // *INDENT-OFF*
    boost::array<double, 9> cov_in{1.0, 0.2, 0.3,  // NOLINT(whitespace/braces)
                                   0.2, 2.0, 0.4,
                                   0.3, 0.4, 3.0};  // NOLINT(whitespace/braces)
    // *INDENT-ON*
    tf2_2d::Transform trans(1.0, 1.5, 2.0);
    boost::array<double, 9> actual = tf2::transformCovariance(cov_in, trans);
    // *INDENT-OFF*
    boost::array<double, 9> expected{1.978182309493392,   0.247672523481242,  -0.488563021694415,  // NOLINT
                                     0.247672523481242,   1.021817690506609,   0.106330493428848,
                                    -0.488563021694415,   0.106330493428848,   3.000000000000000};  // NOLINT
    // *INDENT-ON*
    EXPECT_NEAR(expected[0], actual[0], 1.0e-9);
    EXPECT_NEAR(expected[1], actual[1], 1.0e-9);
    EXPECT_NEAR(expected[2], actual[2], 1.0e-9);
    EXPECT_NEAR(expected[3], actual[3], 1.0e-9);
    EXPECT_NEAR(expected[4], actual[4], 1.0e-9);
    EXPECT_NEAR(expected[5], actual[5], 1.0e-9);
    EXPECT_NEAR(expected[6], actual[6], 1.0e-9);
    EXPECT_NEAR(expected[7], actual[7], 1.0e-9);
    EXPECT_NEAR(expected[8], actual[8], 1.0e-9);
  }
}
