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

#include <tf2_2d/vector2.hpp>


TEST(Vector2, Constructor)
{
  // Verify the constructors execute, and the expected values exist post-construction
  EXPECT_NO_THROW(tf2_2d::Vector2());
  EXPECT_NO_THROW(tf2_2d::Vector2(1.0, 2.0));

  tf2_2d::Vector2 v(1.0, 2.0);
  EXPECT_EQ(1.0, v.x());
  EXPECT_EQ(2.0, v.y());
}

TEST(Vector2, Negate)
{
  // Check the negation operator
  {
    tf2_2d::Vector2 original(1.0, 2.0);
    tf2_2d::Vector2 actual = -original;
    tf2_2d::Vector2 expected(-1.0, -2.0);
    EXPECT_EQ(expected.x(), actual.x());
    EXPECT_EQ(expected.y(), actual.y());
  }
  {
    tf2_2d::Vector2 original(-1.0, -2.0);
    tf2_2d::Vector2 actual = -original;
    tf2_2d::Vector2 expected(1.0, 2.0);
    EXPECT_EQ(expected.x(), actual.x());
    EXPECT_EQ(expected.y(), actual.y());
  }
  {
    tf2_2d::Vector2 original(1.0, -2.0);
    tf2_2d::Vector2 actual = -original;
    tf2_2d::Vector2 expected(-1.0, 2.0);
    EXPECT_EQ(expected.x(), actual.x());
    EXPECT_EQ(expected.y(), actual.y());
  }
}

TEST(Vector2, Sum)
{
  // Check the + operators
  {
    tf2_2d::Vector2 actual(1.0, -2.0);
    tf2_2d::Vector2 b(-0.1, 0.2);
    actual += b;
    tf2_2d::Vector2 expected(0.9, -1.8);
    EXPECT_NEAR(expected.x(), actual.x(), 1.0e-9);
    EXPECT_NEAR(expected.y(), actual.y(), 1.0e-9);
  }
  {
    tf2_2d::Vector2 a(1.0, -2.0);
    tf2_2d::Vector2 b(-0.1, 0.2);
    tf2_2d::Vector2 actual = a + b;
    tf2_2d::Vector2 expected(0.9, -1.8);
    EXPECT_NEAR(expected.x(), actual.x(), 1.0e-9);
    EXPECT_NEAR(expected.y(), actual.y(), 1.0e-9);
  }
}

TEST(Vector2, Difference)
{
  // Check the - operators
  {
    tf2_2d::Vector2 actual(1.0, -2.0);
    tf2_2d::Vector2 b(-0.1, 0.2);
    actual -= b;
    tf2_2d::Vector2 expected(1.1, -2.2);
    EXPECT_NEAR(expected.x(), actual.x(), 1.0e-9);
    EXPECT_NEAR(expected.y(), actual.y(), 1.0e-9);
  }
  {
    tf2_2d::Vector2 a(1.0, -2.0);
    tf2_2d::Vector2 b(-0.1, 0.2);
    tf2_2d::Vector2 actual = a - b;
    tf2_2d::Vector2 expected(1.1, -2.2);
    EXPECT_NEAR(expected.x(), actual.x(), 1.0e-9);
    EXPECT_NEAR(expected.y(), actual.y(), 1.0e-9);
  }
}

TEST(Vector2, Scale)
{
  // Check the scalar * operators
  {
    tf2_2d::Vector2 actual(1.0, -2.0);
    actual *= 1.5;
    tf2_2d::Vector2 expected(1.5, -3.0);
    EXPECT_NEAR(expected.x(), actual.x(), 1.0e-9);
    EXPECT_NEAR(expected.y(), actual.y(), 1.0e-9);
  }
  {
    tf2_2d::Vector2 original(1.0, -2.0);
    tf2_2d::Vector2 actual = original * 1.5;
    tf2_2d::Vector2 expected(1.5, -3.0);
    EXPECT_NEAR(expected.x(), actual.x(), 1.0e-9);
    EXPECT_NEAR(expected.y(), actual.y(), 1.0e-9);
  }
  {
    tf2_2d::Vector2 original(1.0, -2.0);
    tf2_2d::Vector2 actual = 1.5 * original;
    tf2_2d::Vector2 expected(1.5, -3.0);
    EXPECT_NEAR(expected.x(), actual.x(), 1.0e-9);
    EXPECT_NEAR(expected.y(), actual.y(), 1.0e-9);
  }
  {
    tf2_2d::Vector2 actual(1.0, -2.0);
    actual /= 1.25;
    tf2_2d::Vector2 expected(0.8, -1.6);
    EXPECT_NEAR(expected.x(), actual.x(), 1.0e-9);
    EXPECT_NEAR(expected.y(), actual.y(), 1.0e-9);
  }
  {
    tf2_2d::Vector2 original(1.0, -2.0);
    tf2_2d::Vector2 actual = original / 1.25;
    tf2_2d::Vector2 expected(0.8, -1.6);
    EXPECT_EQ(expected.x(), actual.x());
    EXPECT_EQ(expected.y(), actual.y());
  }
}

TEST(Vector2, Multiplication)
{
  // Check the element-wise multiplication operators
  {
    tf2_2d::Vector2 actual(2.0, -2.0);
    tf2_2d::Vector2 b(1.5, -0.8);
    actual *= b;
    tf2_2d::Vector2 expected(3.0, 1.6);
    EXPECT_NEAR(expected.x(), actual.x(), 1.0e-9);
    EXPECT_NEAR(expected.y(), actual.y(), 1.0e-9);
  }
  {
    tf2_2d::Vector2 a(2.0, -2.0);
    tf2_2d::Vector2 b(1.5, -0.8);
    tf2_2d::Vector2 actual = a * b;
    tf2_2d::Vector2 expected(3.0, 1.6);
    EXPECT_NEAR(expected.x(), actual.x(), 1.0e-9);
    EXPECT_NEAR(expected.y(), actual.y(), 1.0e-9);
  }
}

TEST(Vector2, Division)
{
  // Check the element-wise division operators
  {
    tf2_2d::Vector2 actual(3.0, 1.6);
    tf2_2d::Vector2 b(1.5, -0.8);
    actual /= b;
    tf2_2d::Vector2 expected(2.0, -2.0);
    EXPECT_NEAR(expected.x(), actual.x(), 1.0e-9);
    EXPECT_NEAR(expected.y(), actual.y(), 1.0e-9);
  }
  {
    tf2_2d::Vector2 a(3.0, 1.6);
    tf2_2d::Vector2 b(1.5, -0.8);
    tf2_2d::Vector2 actual = a / b;
    tf2_2d::Vector2 expected(2.0, -2.0);
    EXPECT_NEAR(expected.x(), actual.x(), 1.0e-9);
    EXPECT_NEAR(expected.y(), actual.y(), 1.0e-9);
  }
}

TEST(Vector2, Equal)
{
  // Check the equal comparison operators
  tf2_2d::Vector2 a(1.0, 2.0);
  tf2_2d::Vector2 b(1.0, 2.0);
  tf2_2d::Vector2 c(1.1, 2.0);
  tf2_2d::Vector2 d(1.0, 2.1);
  EXPECT_TRUE(a == b);
  EXPECT_FALSE(a == c);
  EXPECT_FALSE(a == d);
}

TEST(Vector2, NotEqual)
{
  // Check the equal comparison operators
  tf2_2d::Vector2 a(1.0, 2.0);
  tf2_2d::Vector2 b(1.0, 2.0);
  tf2_2d::Vector2 c(1.1, 2.0);
  tf2_2d::Vector2 d(1.0, 2.1);
  EXPECT_FALSE(a != b);
  EXPECT_TRUE(a != c);
  EXPECT_TRUE(a != d);
}

TEST(Vector2, DotProduct)
{
  tf2_2d::Vector2 a(3.0, 1.6);
  tf2_2d::Vector2 b(1.5, -0.8);
  double actual = a.dot(b);
  EXPECT_NEAR(3.22, actual, 1.0e-9);
}

TEST(Vector2, Length2)
{
  tf2_2d::Vector2 a(3.0, -1.6);
  double actual = a.length2();
  EXPECT_NEAR(11.56, actual, 1.0e-9);
}

TEST(Vector2, Length)
{
  tf2_2d::Vector2 a(3.0, -1.6);
  double actual = a.length();
  EXPECT_NEAR(3.4, actual, 1.0e-9);
}

TEST(Vector2, Distance2)
{
  tf2_2d::Vector2 a(3.0, 1.6);
  tf2_2d::Vector2 b(1.5, -0.8);
  double actual = a.distance2(b);
  EXPECT_NEAR(8.01, actual, 1.0e-9);
}

TEST(Vector2, Distance)
{
  tf2_2d::Vector2 a(3.0, 1.6);
  tf2_2d::Vector2 b(1.5, -0.8);
  double actual = a.distance(b);
  EXPECT_NEAR(2.830194339616981, actual, 1.0e-9);
}

TEST(Vector2, Normalize)
{
  tf2_2d::Vector2 actual(3.0, -4.0);
  actual.normalize();
  tf2_2d::Vector2 expected(0.6, -0.8);
  EXPECT_NEAR(1.0, actual.length(), 1.0e-9);
  EXPECT_NEAR(expected.x(), actual.x(), 1.0e-9);
  EXPECT_NEAR(expected.y(), actual.y(), 1.0e-9);
}

TEST(Vector2, Normalized)
{
  tf2_2d::Vector2 v(3.0, -4.0);
  tf2_2d::Vector2 actual = v.normalized();
  tf2_2d::Vector2 expected(0.6, -0.8);
  EXPECT_NEAR(1.0, actual.length(), 1.0e-9);
  EXPECT_NEAR(expected.x(), actual.x(), 1.0e-9);
  EXPECT_NEAR(expected.y(), actual.y(), 1.0e-9);
}

TEST(Vector2, Angle)
{
  {
    tf2_2d::Vector2 a(4.0, 4.0);
    tf2_2d::Vector2 b(0.0, 4.0);
    double actual = a.angle(b);
    double expected = M_PI_4;
    EXPECT_NEAR(expected, actual, 1.0e-9);
  }
  {
    tf2_2d::Vector2 a(4.0, 4.0);
    tf2_2d::Vector2 b(4.0, 0.0);
    double actual = a.angle(b);
    double expected = M_PI_4;
    EXPECT_NEAR(expected, actual, 1.0e-9);
  }
}

TEST(Vector2, Absolute)
{
  tf2_2d::Vector2 v(3.0, -4.0);
  tf2_2d::Vector2 actual = v.absolute();
  tf2_2d::Vector2 expected(3.0, 4.0);
  EXPECT_EQ(expected.x(), actual.x());
  EXPECT_EQ(expected.y(), actual.y());
}

TEST(Vector2, MinAxis)
{
  {
    tf2_2d::Vector2 v(3.0, 4.0);
    tf2_2d::Vector2::Axis actual = v.minAxis();
    EXPECT_EQ(tf2_2d::Vector2::X, actual);
  }
  {
    tf2_2d::Vector2 v(3.0, 2.0);
    tf2_2d::Vector2::Axis actual = v.minAxis();
    EXPECT_EQ(tf2_2d::Vector2::Y, actual);
  }
  {
    tf2_2d::Vector2 v(-3.0, -4.0);
    tf2_2d::Vector2::Axis actual = v.minAxis();
    EXPECT_EQ(tf2_2d::Vector2::Y, actual);
  }
  {
    tf2_2d::Vector2 v(-3.0, -2.0);
    tf2_2d::Vector2::Axis actual = v.minAxis();
    EXPECT_EQ(tf2_2d::Vector2::X, actual);
  }
}

TEST(Vector2, MaxAxis)
{
  {
    tf2_2d::Vector2 v(3.0, 4.0);
    tf2_2d::Vector2::Axis actual = v.maxAxis();
    EXPECT_EQ(tf2_2d::Vector2::Y, actual);
  }
  {
    tf2_2d::Vector2 v(3.0, 2.0);
    tf2_2d::Vector2::Axis actual = v.maxAxis();
    EXPECT_EQ(tf2_2d::Vector2::X, actual);
  }
  {
    tf2_2d::Vector2 v(-3.0, -4.0);
    tf2_2d::Vector2::Axis actual = v.maxAxis();
    EXPECT_EQ(tf2_2d::Vector2::X, actual);
  }
  {
    tf2_2d::Vector2 v(-3.0, -2.0);
    tf2_2d::Vector2::Axis actual = v.maxAxis();
    EXPECT_EQ(tf2_2d::Vector2::Y, actual);
  }
}

TEST(Vector2, FurthestAxis)
{
  {
    tf2_2d::Vector2 v(3.0, 4.0);
    tf2_2d::Vector2::Axis actual = v.furthestAxis();
    EXPECT_EQ(tf2_2d::Vector2::X, actual);
  }
  {
    tf2_2d::Vector2 v(3.0, 2.0);
    tf2_2d::Vector2::Axis actual = v.furthestAxis();
    EXPECT_EQ(tf2_2d::Vector2::Y, actual);
  }
  {
    tf2_2d::Vector2 v(-3.0, -4.0);
    tf2_2d::Vector2::Axis actual = v.furthestAxis();
    EXPECT_EQ(tf2_2d::Vector2::X, actual);
  }
  {
    tf2_2d::Vector2 v(-3.0, -2.0);
    tf2_2d::Vector2::Axis actual = v.furthestAxis();
    EXPECT_EQ(tf2_2d::Vector2::Y, actual);
  }
}

TEST(Vector2, ClosestAxis)
{
  {
    tf2_2d::Vector2 v(3.0, 4.0);
    tf2_2d::Vector2::Axis actual = v.closestAxis();
    EXPECT_EQ(tf2_2d::Vector2::Y, actual);
  }
  {
    tf2_2d::Vector2 v(3.0, 2.0);
    tf2_2d::Vector2::Axis actual = v.closestAxis();
    EXPECT_EQ(tf2_2d::Vector2::X, actual);
  }
  {
    tf2_2d::Vector2 v(-3.0, -4.0);
    tf2_2d::Vector2::Axis actual = v.closestAxis();
    EXPECT_EQ(tf2_2d::Vector2::Y, actual);
  }
  {
    tf2_2d::Vector2 v(-3.0, -2.0);
    tf2_2d::Vector2::Axis actual = v.closestAxis();
    EXPECT_EQ(tf2_2d::Vector2::X, actual);
  }
}

TEST(Vector2, Lerp)
{
  {
    tf2_2d::Vector2 a(-3.0, -2.0);
    tf2_2d::Vector2 b(5.0, 6.0);
    tf2_2d::Vector2 actual = a.lerp(b, 0.0);
    tf2_2d::Vector2 expected(-3.0, -2.0);
    EXPECT_NEAR(expected.x(), actual.x(), 1.0e-9);
    EXPECT_NEAR(expected.y(), actual.y(), 1.0e-9);
  }
  {
    tf2_2d::Vector2 a(-3.0, -2.0);
    tf2_2d::Vector2 b(5.0, 6.0);
    tf2_2d::Vector2 actual = a.lerp(b, 1.0);
    tf2_2d::Vector2 expected(5.0, 6.0);
    EXPECT_NEAR(expected.x(), actual.x(), 1.0e-9);
    EXPECT_NEAR(expected.y(), actual.y(), 1.0e-9);
  }
  {
    tf2_2d::Vector2 a(-3.0, -2.0);
    tf2_2d::Vector2 b(5.0, 6.0);
    tf2_2d::Vector2 actual = a.lerp(b, 0.5);
    tf2_2d::Vector2 expected(1.0, 2.0);
    EXPECT_NEAR(expected.x(), actual.x(), 1.0e-9);
    EXPECT_NEAR(expected.y(), actual.y(), 1.0e-9);
  }
}

TEST(Vector2, AccessX)
{
  // Check functions that provide access to the X value
  tf2_2d::Vector2 v(1.5, 2.3);
  EXPECT_EQ(1.5, v.getX());
  EXPECT_EQ(1.5, v.x());
  v.setX(-3.1);
  EXPECT_EQ(-3.1, v.getX());
  EXPECT_EQ(-3.1, v.x());
}

TEST(Vector2, AccessY)
{
  // Check functions that provide access to the Y value
  tf2_2d::Vector2 v(1.5, 2.3);
  EXPECT_EQ(2.3, v.getY());
  EXPECT_EQ(2.3, v.y());
  v.setY(-3.1);
  EXPECT_EQ(-3.1, v.getY());
  EXPECT_EQ(-3.1, v.y());
}

TEST(Vector2, SetMax)
{
  {
    tf2_2d::Vector2 actual(3.0, 4.0);
    tf2_2d::Vector2 v(-5.0, -6.0);
    actual.setMax(v);
    tf2_2d::Vector2 expected(3.0, 4.0);
    EXPECT_EQ(expected.x(), actual.x());
    EXPECT_EQ(expected.y(), actual.y());
  }
  {
    tf2_2d::Vector2 actual(3.0, 4.0);
    tf2_2d::Vector2 v(5.0, 6.0);
    actual.setMax(v);
    tf2_2d::Vector2 expected(5.0, 6.0);
    EXPECT_EQ(expected.x(), actual.x());
    EXPECT_EQ(expected.y(), actual.y());
  }
  {
    tf2_2d::Vector2 actual(3.0, 4.0);
    tf2_2d::Vector2 v(5.0, -6.0);
    actual.setMax(v);
    tf2_2d::Vector2 expected(5.0, 4.0);
    EXPECT_EQ(expected.x(), actual.x());
    EXPECT_EQ(expected.y(), actual.y());
  }
  {
    tf2_2d::Vector2 actual(3.0, 4.0);
    tf2_2d::Vector2 v(-5.0, 6.0);
    actual.setMax(v);
    tf2_2d::Vector2 expected(3.0, 6.0);
    EXPECT_EQ(expected.x(), actual.x());
    EXPECT_EQ(expected.y(), actual.y());
  }
}

TEST(Vector2, SetMin)
{
  {
    tf2_2d::Vector2 actual(3.0, 4.0);
    tf2_2d::Vector2 v(-5.0, -6.0);
    actual.setMin(v);
    tf2_2d::Vector2 expected(-5.0, -6.0);
    EXPECT_EQ(expected.x(), actual.x());
    EXPECT_EQ(expected.y(), actual.y());
  }
  {
    tf2_2d::Vector2 actual(3.0, 4.0);
    tf2_2d::Vector2 v(5.0, 6.0);
    actual.setMin(v);
    tf2_2d::Vector2 expected(3.0, 4.0);
    EXPECT_EQ(expected.x(), actual.x());
    EXPECT_EQ(expected.y(), actual.y());
  }
  {
    tf2_2d::Vector2 actual(3.0, 4.0);
    tf2_2d::Vector2 v(5.0, -6.0);
    actual.setMin(v);
    tf2_2d::Vector2 expected(3.0, -6.0);
    EXPECT_EQ(expected.x(), actual.x());
    EXPECT_EQ(expected.y(), actual.y());
  }
  {
    tf2_2d::Vector2 actual(3.0, 4.0);
    tf2_2d::Vector2 v(-5.0, 6.0);
    actual.setMin(v);
    tf2_2d::Vector2 expected(-5.0, 4.0);
    EXPECT_EQ(expected.x(), actual.x());
    EXPECT_EQ(expected.y(), actual.y());
  }
}

TEST(Vector2, SetValue)
{
  tf2_2d::Vector2 actual(3.0, 4.0);
  actual.setValue(-5.0, -6.0);
  tf2_2d::Vector2 expected(-5.0, -6.0);
  EXPECT_EQ(expected.x(), actual.x());
  EXPECT_EQ(expected.y(), actual.y());
}

TEST(Vector2, SetZero)
{
  tf2_2d::Vector2 actual(3.0, 4.0);
  actual.setZero();
  tf2_2d::Vector2 expected(0, 0);
  EXPECT_EQ(expected.x(), actual.x());
  EXPECT_EQ(expected.y(), actual.y());
}

TEST(Vector2, IsZero)
{
  tf2_2d::Vector2 a(3.0, 4.0);
  tf2_2d::Vector2 b(0, 0);
  tf2_2d::Vector2 c(1.0e-8, 0);
  tf2_2d::Vector2 d(0, 1.0e-8);
  tf2_2d::Vector2 e(1.0e-8, 1.0e-8);
  EXPECT_FALSE(a.isZero());
  EXPECT_TRUE(b.isZero());
  EXPECT_FALSE(c.isZero());
  EXPECT_FALSE(d.isZero());
  EXPECT_FALSE(e.isZero());
}

TEST(Vector2, FuzzyZero)
{
  tf2_2d::Vector2 a(3.0, 4.0);
  tf2_2d::Vector2 b(0, 0);
  tf2_2d::Vector2 c(1.0e-8, 0);
  tf2_2d::Vector2 d(0, 1.0e-8);
  tf2_2d::Vector2 e(1.0e-8, 1.0e-8);
  tf2_2d::Vector2 f(2.0e-8, 0);
  EXPECT_FALSE(a.fuzzyZero());
  EXPECT_TRUE(b.fuzzyZero());
  EXPECT_TRUE(c.fuzzyZero());
  EXPECT_TRUE(d.fuzzyZero());
  EXPECT_TRUE(e.fuzzyZero());
  EXPECT_FALSE(f.fuzzyZero());
}

TEST(Vector2, GetVector)
{
  tf2_2d::Vector2 v(3.0, 4.0);
  Eigen::Vector2d actual = v.getVector();
  Eigen::Vector2d expected;
  expected << 3.0, 4.0;
  EXPECT_DOUBLE_EQ(expected(0), actual(0));
  EXPECT_DOUBLE_EQ(expected(1), actual(1));
}

TEST(Vector2, GetHomogeneousMatrix)
{
  tf2_2d::Vector2 v(3.0, 4.0);
  Eigen::Matrix3d actual = v.getHomogeneousMatrix();
  Eigen::Matrix3d expected;
  expected << 0, 0, 3.0, 0, 0, 4.0, 0, 0, 1;
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

TEST(Vector2, Stream)
{
  tf2_2d::Vector2 v(3.0, 4.0);
  std::cout << v << std::endl;
  SUCCEED();
}
