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
#ifndef TF2_2D__VECTOR2_IMPL_HPP_
#define TF2_2D__VECTOR2_IMPL_HPP_

#include <Eigen/Core>

#include <tf2/LinearMath/MinMax.h>


namespace tf2_2d
{

inline Vector2::Vector2()
{
  setZero();
}

inline Vector2::Vector2(const tf2Scalar x, const tf2Scalar y)
{
  x_ = x;
  y_ = y;
}

inline Vector2 Vector2::operator-() const
{
  return Vector2(-x_, -y_);
}

inline Vector2 & Vector2::operator+=(const Vector2 & rhs)
{
  x_ += rhs.x_;
  y_ += rhs.y_;
  return *this;
}

inline Vector2 & Vector2::operator-=(const Vector2 & rhs)
{
  x_ -= rhs.x_;
  y_ -= rhs.y_;
  return *this;
}

inline Vector2 & Vector2::operator*=(const tf2Scalar s)
{
  x_ *= s;
  y_ *= s;
  return *this;
}

inline Vector2 & Vector2::operator*=(const Vector2 & rhs)
{
  x_ *= rhs.x_;
  y_ *= rhs.y_;
  return *this;
}

inline Vector2 & Vector2::operator/=(const tf2Scalar rhs)
{
  tf2FullAssert(rhs != tf2Scalar(0.0));
  return *this *= tf2Scalar(1.0) / rhs;
}

inline Vector2 & Vector2::operator/=(const Vector2 & rhs)
{
  x_ /= rhs.x_;
  y_ /= rhs.y_;
  return *this;
}

inline bool Vector2::operator==(const Vector2 & other) const
{
  return (x_ == other.x_) && (y_ == other.y_);
}

inline bool Vector2::operator!=(const Vector2 & other) const
{
  return !operator==(other);
}

inline tf2Scalar Vector2::dot(const Vector2 & other) const
{
  return x_ * other.x_ + y_ * other.y_;
}

inline tf2Scalar Vector2::length2() const
{
  return dot(*this);
}

inline tf2Scalar Vector2::length() const
{
  return tf2Sqrt(length2());
}

inline tf2Scalar Vector2::distance2(const Vector2 & other) const
{
  return (other - *this).length2();
}

inline tf2Scalar Vector2::distance(const Vector2 & other) const
{
  return (other - *this).length();
}

inline Vector2 & Vector2::normalize()
{
  return *this /= length();
}

inline Vector2 Vector2::normalized() const
{
  return *this / length();
}

inline tf2Scalar Vector2::angle(const Vector2 & other) const
{
  tf2Scalar s = tf2Sqrt(length2() * other.length2());
  tf2FullAssert(s != tf2Scalar(0));
  return tf2Acos(dot(other) / s);
}

inline Vector2 Vector2::absolute() const
{
  return Vector2(tf2Fabs(x_), tf2Fabs(y_));
}

inline Vector2::Axis Vector2::minAxis() const
{
  return x_ < y_ ? X : Y;
}

inline Vector2::Axis Vector2::maxAxis() const
{
  return x_ < y_ ? Y : X;
}

inline Vector2::Axis Vector2::furthestAxis() const
{
  return absolute().minAxis();
}

inline Vector2::Axis Vector2::closestAxis() const
{
  return absolute().maxAxis();
}

inline Vector2 Vector2::lerp(const Vector2 & other, const tf2Scalar ratio) const
{
  return Vector2(
    x_ + (other.x_ - x_) * ratio,
    y_ + (other.y_ - y_) * ratio);
}

inline const tf2Scalar & Vector2::getX() const
{
  return x_;
}

inline const tf2Scalar & Vector2::getY() const
{
  return y_;
}

inline void Vector2::setX(const tf2Scalar x)
{
  x_ = x;
}

inline void Vector2::setY(const tf2Scalar y)
{
  y_ = y;
}

inline void Vector2::setMax(const Vector2 & other)
{
  tf2SetMax(x_, other.x_);
  tf2SetMax(y_, other.y_);
}

inline void Vector2::setMin(const Vector2 & other)
{
  tf2SetMin(x_, other.x_);
  tf2SetMin(y_, other.y_);
}

inline void Vector2::setValue(const tf2Scalar x, const tf2Scalar y)
{
  x_ = x;
  y_ = y;
}

inline void Vector2::setZero()
{
  setValue(0.0, 0.0);
}

inline bool Vector2::isZero() const
{
  return x_ == 0.0 && y_ == 0.0;
}

inline bool Vector2::fuzzyZero() const
{
  return length2() < TF2SIMD_EPSILON;
}

inline Eigen::Vector2d Vector2::getVector() const
{
  Eigen::Vector2d vec;
  vec << x_, y_;
  return vec;
}

inline Eigen::Matrix3d Vector2::getHomogeneousMatrix() const
{
  Eigen::Matrix3d matrix;
  matrix << 0, 0, x_, 0, 0, y_, 0, 0, 1;
  return matrix;
}

inline Vector2 operator+(Vector2 lhs, const Vector2 & rhs)
{
  lhs += rhs;
  return lhs;
}

inline Vector2 operator-(Vector2 lhs, const Vector2 & rhs)
{
  lhs -= rhs;
  return lhs;
}

inline Vector2 operator*(Vector2 lhs, const tf2Scalar rhs)
{
  lhs *= rhs;
  return lhs;
}

inline Vector2 operator*(const tf2Scalar lhs, Vector2 rhs)
{
  rhs *= lhs;
  return rhs;
}

inline Vector2 operator*(Vector2 lhs, const Vector2 & rhs)
{
  lhs *= rhs;
  return lhs;
}

inline Vector2 operator/(Vector2 lhs, const tf2Scalar rhs)
{
  lhs /= rhs;
  return lhs;
}

inline Vector2 operator/(Vector2 lhs, const Vector2 & rhs)
{
  lhs /= rhs;
  return lhs;
}

inline std::ostream & operator<<(std::ostream & stream, const Vector2 & vector)
{
  return stream << "x: " << vector.x() << ", y: " << vector.y();
}

}  // namespace tf2_2d

#endif  // TF2_2D__VECTOR2_IMPL_HPP_
