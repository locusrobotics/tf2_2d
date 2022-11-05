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

#include <tf2_2d/rotation.hpp>
#include <tf2_2d/vector2.hpp>


TEST(Rotation, Constructor)
{
  // Verify the constructors execute, and the expected values exist post-construction
  EXPECT_NO_THROW(tf2_2d::Rotation());
  EXPECT_NO_THROW(tf2_2d::Rotation(1.0));

  tf2_2d::Rotation r(1.0);
  EXPECT_EQ(1.0, r.angle());
}

TEST(Rotation, Negate)
{
  // Check the negation operator
  {
    tf2_2d::Rotation original(1.0);
    tf2_2d::Rotation actual = -original;
    tf2_2d::Rotation expected(-1.0);
    EXPECT_EQ(expected.angle(), actual.angle());
  }
  {
    tf2_2d::Rotation original(-1.0);
    tf2_2d::Rotation actual = -original;
    tf2_2d::Rotation expected(1.0);
    EXPECT_EQ(expected.angle(), actual.angle());
  }
}

TEST(Rotation, Sum)
{
  // Check the + operators
  {
    tf2_2d::Rotation actual(1.0);
    tf2_2d::Rotation b(-0.1);
    actual += b;
    tf2_2d::Rotation expected(0.9);
    EXPECT_NEAR(expected.angle(), actual.angle(), 1.0e-9);
  }
  {
    tf2_2d::Rotation a(1.0);
    tf2_2d::Rotation b(-0.1);
    tf2_2d::Rotation actual = a + b;
    tf2_2d::Rotation expected(0.9);
    EXPECT_NEAR(expected.angle(), actual.angle(), 1.0e-9);
  }
}

TEST(Rotation, Difference)
{
  // Check the - operators
  {
    tf2_2d::Rotation actual(1.5);
    tf2_2d::Rotation b(-0.1);
    actual -= b;
    tf2_2d::Rotation expected(1.6);
    EXPECT_NEAR(expected.angle(), actual.angle(), 1.0e-9);
  }
  {
    tf2_2d::Rotation a(1.5);
    tf2_2d::Rotation b(-0.1);
    tf2_2d::Rotation actual = a - b;
    tf2_2d::Rotation expected(1.6);
    EXPECT_NEAR(expected.angle(), actual.angle(), 1.0e-9);
  }
}

TEST(Rotation, Scale)
{
  // Check the scalar * operators
  {
    tf2_2d::Rotation actual(2.0);
    actual *= 1.5;
    tf2_2d::Rotation expected(3.0);
    EXPECT_NEAR(expected.angle(), actual.angle(), 1.0e-9);
  }
  {
    tf2_2d::Rotation original(2.0);
    tf2_2d::Rotation actual = original * 1.5;
    tf2_2d::Rotation expected(3.0);
    EXPECT_NEAR(expected.angle(), actual.angle(), 1.0e-9);
  }
  {
    tf2_2d::Rotation original(2.0);
    tf2_2d::Rotation actual = 1.5 * original;
    tf2_2d::Rotation expected(3.0);
    EXPECT_NEAR(expected.angle(), actual.angle(), 1.0e-9);
  }
  {
    tf2_2d::Rotation actual(-2.0);
    actual /= 1.25;
    tf2_2d::Rotation expected(-1.6);
    EXPECT_NEAR(expected.angle(), actual.angle(), 1.0e-9);
  }
  {
    tf2_2d::Rotation original(-2.0);
    tf2_2d::Rotation actual = original / 1.25;
    tf2_2d::Rotation expected(-1.6);
    EXPECT_NEAR(expected.angle(), actual.angle(), 1.0e-9);
  }
}

TEST(Rotation, Equal)
{
  // Check the equal comparison operators
  tf2_2d::Rotation a(2.0);
  tf2_2d::Rotation b(2.0);
  tf2_2d::Rotation c(2.1);
  EXPECT_TRUE(a == b);
  EXPECT_FALSE(a == c);
}

TEST(Rotation, NotEqual)
{
  // Check the equal comparison operators
  tf2_2d::Rotation a(2.0);
  tf2_2d::Rotation b(2.0);
  tf2_2d::Rotation c(2.1);
  EXPECT_FALSE(a != b);
  EXPECT_TRUE(a != c);
}

TEST(Rotation, Distance2)
{
  tf2_2d::Rotation a(1.5);
  tf2_2d::Rotation b(3.0);
  double actual = a.distance2(b);
  EXPECT_NEAR(2.25, actual, 1.0e-9);
}

TEST(Rotation, Distance)
{
  tf2_2d::Rotation a(1.5);
  tf2_2d::Rotation b(3.0);
  double actual = a.distance(b);
  EXPECT_NEAR(1.5, actual, 1.0e-9);
}

TEST(Rotation, Rotate)
{
  {
    tf2_2d::Rotation r(0);
    tf2_2d::Vector2 v(3.0, 4.0);
    tf2_2d::Vector2 actual = r.rotate(v);
    tf2_2d::Vector2 expected(3.0, 4.0);
    EXPECT_NEAR(expected.x(), actual.x(), 1.0e-9);
    EXPECT_NEAR(expected.y(), actual.y(), 1.0e-9);
  }
  {
    tf2_2d::Rotation r(M_PI_2);
    tf2_2d::Vector2 v(3.0, 4.0);
    tf2_2d::Vector2 actual = r.rotate(v);
    tf2_2d::Vector2 expected(-4.0, 3.0);
    EXPECT_NEAR(expected.x(), actual.x(), 1.0e-9);
    EXPECT_NEAR(expected.y(), actual.y(), 1.0e-9);
  }
  {
    tf2_2d::Rotation r(-M_PI_2);
    tf2_2d::Vector2 v(3.0, 4.0);
    tf2_2d::Vector2 actual = r.rotate(v);
    tf2_2d::Vector2 expected(4.0, -3.0);
    EXPECT_NEAR(expected.x(), actual.x(), 1.0e-9);
    EXPECT_NEAR(expected.y(), actual.y(), 1.0e-9);
  }
}

TEST(Rotation, Unrotate)
{
  {
    tf2_2d::Rotation r(0);
    tf2_2d::Vector2 v(3.0, 4.0);
    tf2_2d::Vector2 actual = r.unrotate(v);
    tf2_2d::Vector2 expected(3.0, 4.0);
    EXPECT_NEAR(expected.x(), actual.x(), 1.0e-9);
    EXPECT_NEAR(expected.y(), actual.y(), 1.0e-9);
  }
  {
    tf2_2d::Rotation r(M_PI_2);
    tf2_2d::Vector2 v(-4.0, 3.0);
    tf2_2d::Vector2 actual = r.unrotate(v);
    tf2_2d::Vector2 expected(3.0, 4.0);
    EXPECT_NEAR(expected.x(), actual.x(), 1.0e-9);
    EXPECT_NEAR(expected.y(), actual.y(), 1.0e-9);
  }
  {
    tf2_2d::Rotation r(-M_PI_2);
    tf2_2d::Vector2 v(4.0, -3.0);
    tf2_2d::Vector2 actual = r.unrotate(v);
    tf2_2d::Vector2 expected(3.0, 4.0);
    EXPECT_NEAR(expected.x(), actual.x(), 1.0e-9);
    EXPECT_NEAR(expected.y(), actual.y(), 1.0e-9);
  }
}

TEST(Rotation, Inverse)
{
  tf2_2d::Rotation r(M_PI_2);
  tf2_2d::Rotation actual = r.inverse();
  tf2_2d::Rotation expected(-M_PI_2);
  EXPECT_EQ(expected.angle(), actual.angle());
}

TEST(Rotation, Absolute)
{
  {
    tf2_2d::Rotation r(3.0);
    tf2_2d::Rotation actual = r.absolute();
    tf2_2d::Rotation expected(3.0);
    EXPECT_EQ(expected.angle(), actual.angle());
  }
  {
    tf2_2d::Rotation r(-3.0);
    tf2_2d::Rotation actual = r.absolute();
    tf2_2d::Rotation expected(3.0);
    EXPECT_EQ(expected.angle(), actual.angle());
  }
}

TEST(Rotation, Lerp)
{
  {
    tf2_2d::Rotation a(-1.0);
    tf2_2d::Rotation b(2.0);
    tf2_2d::Rotation actual = a.lerp(b, 0.0);
    tf2_2d::Rotation expected(-1.0);
    EXPECT_NEAR(expected.angle(), actual.angle(), 1.0e-9);
  }
  {
    tf2_2d::Rotation a(-1.0);
    tf2_2d::Rotation b(2.0);
    tf2_2d::Rotation actual = a.lerp(b, 1.0);
    tf2_2d::Rotation expected(2.0);
    EXPECT_NEAR(expected.angle(), actual.angle(), 1.0e-9);
  }
  {
    tf2_2d::Rotation a(-1.0);
    tf2_2d::Rotation b(2.0);
    tf2_2d::Rotation actual = a.lerp(b, 0.5);
    tf2_2d::Rotation expected(0.5);
    EXPECT_NEAR(expected.angle(), actual.angle(), 1.0e-9);
  }
  {
    // Shortest path goes the other way
    tf2_2d::Rotation a(-3.0);
    tf2_2d::Rotation b(2.0);
    tf2_2d::Rotation actual = a.lerp(b, 0.5);
    tf2_2d::Rotation expected(2.641592653589793);
    EXPECT_NEAR(expected.angle(), actual.angle(), 1.0e-9);
  }
}

TEST(Rotation, AccessAngle)
{
  // Check functions that provide access to the angle value
  tf2_2d::Rotation r(1.5);
  EXPECT_EQ(1.5, r.getAngle());
  EXPECT_EQ(1.5, r.angle());
  r.setAngle(-3.1);
  EXPECT_EQ(-3.1, r.getAngle());
  EXPECT_EQ(-3.1, r.angle());
}

TEST(Rotation, SetMax)
{
  {
    tf2_2d::Rotation actual(2.0);
    tf2_2d::Rotation r(-3.0);
    actual.setMax(r);
    tf2_2d::Rotation expected(2.0);
    EXPECT_EQ(expected.angle(), actual.angle());
  }
  {
    tf2_2d::Rotation actual(2.0);
    tf2_2d::Rotation r(3.0);
    actual.setMax(r);
    tf2_2d::Rotation expected(3.0);
    EXPECT_EQ(expected.angle(), actual.angle());
  }
}

TEST(Rotation, SetMin)
{
  {
    tf2_2d::Rotation actual(2.0);
    tf2_2d::Rotation r(-3.0);
    actual.setMin(r);
    tf2_2d::Rotation expected(-3.0);
    EXPECT_EQ(expected.angle(), actual.angle());
  }
  {
    tf2_2d::Rotation actual(2.0);
    tf2_2d::Rotation r(3.0);
    actual.setMin(r);
    tf2_2d::Rotation expected(2.0);
    EXPECT_EQ(expected.angle(), actual.angle());
  }
}

TEST(Rotation, SetValue)
{
  tf2_2d::Rotation actual(3.0);
  actual.setValue(-2.1);
  tf2_2d::Rotation expected(-2.1);
  EXPECT_EQ(expected.angle(), actual.angle());
}

TEST(Rotation, SetZero)
{
  tf2_2d::Rotation actual(3.0);
  actual.setZero();
  tf2_2d::Rotation expected(0);
  EXPECT_EQ(expected.angle(), actual.angle());
}

TEST(Rotation, IsZero)
{
  tf2_2d::Rotation a(3.0);
  tf2_2d::Rotation b(0);
  tf2_2d::Rotation c(1.0e-8);
  EXPECT_FALSE(a.isZero());
  EXPECT_TRUE(b.isZero());
  EXPECT_FALSE(c.isZero());
}

TEST(Rotation, FuzzyZero)
{
  tf2_2d::Rotation a(3.0);
  tf2_2d::Rotation b(0);
  tf2_2d::Rotation c(1.0e-8);
  tf2_2d::Rotation d(2.0e-8);
  EXPECT_FALSE(a.fuzzyZero());
  EXPECT_TRUE(b.fuzzyZero());
  EXPECT_TRUE(c.fuzzyZero());
  EXPECT_FALSE(d.fuzzyZero());
}

TEST(Rotation, GetRotationMatrix)
{
  tf2_2d::Rotation r(3.0);
  Eigen::Matrix2d actual = r.getRotationMatrix();
  Eigen::Matrix2d expected;
  expected << std::cos(3.0), -std::sin(3.0), std::sin(3.0), std::cos(3.0);
  EXPECT_DOUBLE_EQ(expected(0, 0), actual(0, 0));
  EXPECT_DOUBLE_EQ(expected(0, 1), actual(0, 1));
  EXPECT_DOUBLE_EQ(expected(1, 0), actual(1, 0));
  EXPECT_DOUBLE_EQ(expected(1, 1), actual(1, 1));
}

TEST(Rotation, GetHomogeneousMatrix)
{
  tf2_2d::Rotation r(3.0);
  Eigen::Matrix3d actual = r.getHomogeneousMatrix();
  Eigen::Matrix3d expected;
  expected << std::cos(3.0), -std::sin(3.0), 0, std::sin(3.0), std::cos(3.0), 0, 0, 0, 1;
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

TEST(Rotation, Stream)
{
  tf2_2d::Rotation r(3.0);
  std::cout << r << std::endl;
  SUCCEED();
}
