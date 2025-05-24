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

#include <cmath>

#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2/LinearMath/Vector3.hpp>
#include <tf2_2d/rotation.hpp>
#include <tf2_2d/transform.hpp>
#include <tf2_2d/vector2.hpp>


TEST(Transform, Constructor)
{
  // Verify the constructors execute, and the expected values exist post-construction
  EXPECT_NO_THROW(tf2_2d::Transform());
  EXPECT_NO_THROW(tf2_2d::Transform(tf2_2d::Rotation(3.0), tf2_2d::Vector2(1.0, 2.0)));
  EXPECT_NO_THROW(tf2_2d::Transform(1.0, 2.0, 3.0));

  tf2::Quaternion rotation_3d;
  rotation_3d.setRPY(M_PI / 2.0, M_PI / 3.0, M_PI / 4.0);
  tf2::Vector3 translation_3d(1.0, 2.0, 3.0);
  tf2::Transform transform_3d(rotation_3d, translation_3d);
  EXPECT_NO_THROW(tf2_2d::Transform test_trans(transform_3d));

  {
    tf2_2d::Transform t(tf2_2d::Rotation(3.0), tf2_2d::Vector2(1.0, 2.0));
    EXPECT_EQ(1.0, t.x());
    EXPECT_EQ(2.0, t.y());
    EXPECT_EQ(3.0, t.angle());
  }
  {
    tf2_2d::Transform t(1.0, 2.0, 3.0);
    EXPECT_EQ(1.0, t.x());
    EXPECT_EQ(2.0, t.y());
    EXPECT_EQ(3.0, t.angle());
  }
  {
    tf2_2d::Transform t(transform_3d);
    EXPECT_EQ(1.0, t.x());
    EXPECT_EQ(2.0, t.y());
    EXPECT_NEAR(M_PI / 4.0, t.angle(), 1.0e-9);
  }
}

TEST(Transform, Compose)
{
  // Test composing a transform with other objects using the * operator
  {
    tf2_2d::Transform actual(1.0, 2.0, 3.0);
    tf2_2d::Transform b(-2.0, -1.0, -1.5);
    actual *= b;
    tf2_2d::Transform expected(3.121105001260758, 2.707752480480711, 1.5);
    EXPECT_NEAR(expected.x(), actual.x(), 1.0e-9);
    EXPECT_NEAR(expected.y(), actual.y(), 1.0e-9);
    EXPECT_NEAR(expected.angle(), actual.angle(), 1.0e-9);
  }
  {
    tf2_2d::Transform a(1.0, 2.0, 3.0);
    tf2_2d::Transform b(-2.0, -1.0, -1.5);
    tf2_2d::Transform actual = a * b;
    tf2_2d::Transform expected(3.121105001260758, 2.707752480480711, 1.5);
    EXPECT_NEAR(expected.x(), actual.x(), 1.0e-9);
    EXPECT_NEAR(expected.y(), actual.y(), 1.0e-9);
    EXPECT_NEAR(expected.angle(), actual.angle(), 1.0e-9);
  }
}

TEST(Transform, Equal)
{
  // Check the equal comparison operators
  tf2_2d::Transform a(1.0, 2.0, 3.0);
  tf2_2d::Transform b(1.0, 2.0, 3.0);
  tf2_2d::Transform c(1.1, 2.0, 3.0);
  tf2_2d::Transform d(1.0, 2.1, 3.0);
  tf2_2d::Transform e(1.0, 2.0, 3.1);
  EXPECT_TRUE(a == b);
  EXPECT_FALSE(a == c);
  EXPECT_FALSE(a == d);
  EXPECT_FALSE(a == e);
}

TEST(Transform, NotEqual)
{
  // Check the equal comparison operators
  tf2_2d::Transform a(1.0, 2.0, 3.0);
  tf2_2d::Transform b(1.0, 2.0, 3.0);
  tf2_2d::Transform c(1.1, 2.0, 3.0);
  tf2_2d::Transform d(1.0, 2.1, 3.0);
  tf2_2d::Transform e(1.0, 2.0, 3.1);
  EXPECT_FALSE(a != b);
  EXPECT_TRUE(a != c);
  EXPECT_TRUE(a != d);
  EXPECT_TRUE(a != e);
}

TEST(Transform, Lerp)
{
  {
    tf2_2d::Transform a(1.0, 2.0, 3.0);
    tf2_2d::Transform b(-2.0, -3.0, 1.0);
    tf2_2d::Transform actual = a.lerp(b, 0.0);
    tf2_2d::Transform expected(1.0, 2.0, 3.0);
    EXPECT_NEAR(expected.x(), actual.x(), 1.0e-9);
    EXPECT_NEAR(expected.y(), actual.y(), 1.0e-9);
    EXPECT_NEAR(expected.angle(), actual.angle(), 1.0e-9);
  }
  {
    tf2_2d::Transform a(1.0, 2.0, 3.0);
    tf2_2d::Transform b(-2.0, -3.0, 1.0);
    tf2_2d::Transform actual = a.lerp(b, 1.0);
    tf2_2d::Transform expected(-2.0, -3.0, 1.0);
    EXPECT_NEAR(expected.x(), actual.x(), 1.0e-9);
    EXPECT_NEAR(expected.y(), actual.y(), 1.0e-9);
    EXPECT_NEAR(expected.angle(), actual.angle(), 1.0e-9);
  }
  {
    tf2_2d::Transform a(1.0, 1.5, 2.0);
    tf2_2d::Transform b(-2.0, -1.0, 1.0);
    tf2_2d::Transform actual = a.lerp(b, 0.5);
    tf2_2d::Transform expected(-0.5, 0.25, 1.5);
    EXPECT_NEAR(expected.x(), actual.x(), 1.0e-9);
    EXPECT_NEAR(expected.y(), actual.y(), 1.0e-9);
    EXPECT_NEAR(expected.angle(), actual.angle(), 1.0e-9);
  }
}

TEST(Transform, AccessRotation)
{
  // Check functions that provide access to the rotation value
  tf2_2d::Transform t(1.0, 2.0, 3.0);
  EXPECT_EQ(3.0, t.getRotation().getAngle());
  EXPECT_EQ(3.0, t.rotation().angle());
  EXPECT_EQ(3.0, t.getAngle());
  EXPECT_EQ(3.0, t.angle());
  t.setAngle(-3.1);
  EXPECT_EQ(-3.1, t.getRotation().getAngle());
  EXPECT_EQ(-3.1, t.rotation().angle());
  EXPECT_EQ(-3.1, t.getAngle());
  EXPECT_EQ(-3.1, t.angle());
  t.setRotation(tf2_2d::Rotation(1.5));
  EXPECT_EQ(1.5, t.getRotation().getAngle());
  EXPECT_EQ(1.5, t.rotation().angle());
  EXPECT_EQ(1.5, t.getAngle());
  EXPECT_EQ(1.5, t.angle());
}

TEST(Transform, AccessTranslation)
{
  // Check functions that provide access to the translation values
  {
    tf2_2d::Transform t(1.0, 2.0, 3.0);
    EXPECT_EQ(1.0, t.getTranslation().getX());
    EXPECT_EQ(1.0, t.translation().x());
    EXPECT_EQ(1.0, t.getX());
    EXPECT_EQ(1.0, t.x());
    t.setX(1.5);
    EXPECT_EQ(1.5, t.getTranslation().getX());
    EXPECT_EQ(1.5, t.translation().x());
    EXPECT_EQ(1.5, t.getX());
    EXPECT_EQ(1.5, t.x());
    t.setTranslation(tf2_2d::Vector2(-1.2, 2.0));
    EXPECT_EQ(-1.2, t.getTranslation().getX());
    EXPECT_EQ(-1.2, t.translation().x());
    EXPECT_EQ(-1.2, t.getX());
    EXPECT_EQ(-1.2, t.x());
  }
  {
    tf2_2d::Transform t(1.0, 2.0, 3.0);
    EXPECT_EQ(2.0, t.getTranslation().getY());
    EXPECT_EQ(2.0, t.translation().y());
    EXPECT_EQ(2.0, t.getY());
    EXPECT_EQ(2.0, t.y());
    t.setY(1.5);
    EXPECT_EQ(1.5, t.getTranslation().getY());
    EXPECT_EQ(1.5, t.translation().y());
    EXPECT_EQ(1.5, t.getY());
    EXPECT_EQ(1.5, t.y());
    t.setTranslation(tf2_2d::Vector2(1.0, -1.2));
    EXPECT_EQ(-1.2, t.getTranslation().getY());
    EXPECT_EQ(-1.2, t.translation().y());
    EXPECT_EQ(-1.2, t.getY());
    EXPECT_EQ(-1.2, t.y());
  }
}

TEST(Transform, SetIdentity)
{
  tf2_2d::Transform actual(1.0, 2.0, 3.0);
  actual.setIdentity();
  tf2_2d::Transform expected(0, 0, 0);
  EXPECT_EQ(expected.x(), actual.x());
  EXPECT_EQ(expected.y(), actual.y());
  EXPECT_EQ(expected.angle(), actual.angle());
}

TEST(Transform, Inverse)
{
  tf2_2d::Transform original(1.0, 2.0, 3.0);
  tf2_2d::Transform actual = original.inverse();
  tf2_2d::Transform expected(0.707752480480711, 2.121105001260758, -3.0);
  EXPECT_NEAR(expected.x(), actual.x(), 1.0e-9);
  EXPECT_NEAR(expected.y(), actual.y(), 1.0e-9);
  EXPECT_NEAR(expected.angle(), actual.angle(), 1.0e-9);
}

TEST(Transform, InverseTimes)
{
  tf2_2d::Transform a(1.0, 2.0, 3.0);
  tf2_2d::Transform b(-2.0, -1.0, -1.5);
  tf2_2d::Transform actual = a.inverseTimes(b);
  tf2_2d::Transform expected(2.546617465621735, 3.393337513980938, 1.78318530717959);
  EXPECT_NEAR(expected.x(), actual.x(), 1.0e-9);
  EXPECT_NEAR(expected.y(), actual.y(), 1.0e-9);
  EXPECT_NEAR(expected.angle(), actual.angle(), 1.0e-9);
}

TEST(Transform, GetHomogeneousMatrix)
{
  tf2_2d::Transform t(1.0, 2.0, 3.0);
  Eigen::Matrix3d actual = t.getHomogeneousMatrix();
  Eigen::Matrix3d expected;
  expected << std::cos(3.0), -std::sin(3.0), 1.0, std::sin(3.0), std::cos(3.0), 2.0, 0, 0, 1;
  EXPECT_DOUBLE_EQ(expected(0, 0), actual(0, 0));
  EXPECT_DOUBLE_EQ(expected(0, 1), actual(0, 1));
  EXPECT_DOUBLE_EQ(expected(0, 2), actual(0, 2));
  EXPECT_DOUBLE_EQ(expected(1, 0), actual(1, 0));
  EXPECT_DOUBLE_EQ(expected(1, 1), actual(1, 1));
  EXPECT_DOUBLE_EQ(expected(1, 2), actual(1, 2));
  EXPECT_DOUBLE_EQ(expected(2, 0), actual(2, 0));
  EXPECT_DOUBLE_EQ(expected(2, 1), actual(2, 1));
  EXPECT_DOUBLE_EQ(expected(2, 2), actual(2, 2));
}

TEST(Transform, Stream)
{
  tf2_2d::Transform t(1.0, 2.0, 3.0);
  std::cout << t << std::endl;
  SUCCEED();
}
