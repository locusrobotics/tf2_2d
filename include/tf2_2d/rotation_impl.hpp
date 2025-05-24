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
#ifndef TF2_2D__ROTATION_IMPL_HPP_
#define TF2_2D__ROTATION_IMPL_HPP_

#include <Eigen/Core>

#include <cmath>

#include <tf2/LinearMath/MinMax.hpp>
#include <tf2_2d/vector2.hpp>

namespace tf2_2d
{

inline Rotation::Rotation()
{
  setZero();
}

inline Rotation::Rotation(const tf2Scalar angle)
{
  setValue(angle);
}

inline Rotation::Rotation(const tf2Scalar angle, tf2Scalar cos_angle, tf2Scalar sin_angle)
: angle_(angle),
  cos_angle_(cos_angle),
  sin_angle_(sin_angle)
{
}

inline Rotation Rotation::operator-() const
{
  return inverse();
}

inline Rotation & Rotation::operator+=(const Rotation & rhs)
{
  setValue(angle_ + rhs.angle_);
  return *this;
}

inline Rotation & Rotation::operator-=(const Rotation & rhs)
{
  setValue(angle_ - rhs.angle_);
  return *this;
}

inline Rotation & Rotation::operator*=(const tf2Scalar rhs)
{
  setValue(angle_ * rhs);
  return *this;
}

inline Rotation & Rotation::operator/=(const tf2Scalar rhs)
{
  tf2FullAssert(rhs != tf2Scalar(0.0));
  return *this *= tf2Scalar(1.0) / rhs;
}

inline bool Rotation::operator==(const Rotation & other) const
{
  return angle_ == other.angle_;
}

inline bool Rotation::operator!=(const Rotation & other) const
{
  return !operator==(other);
}

inline tf2Scalar Rotation::distance2(const Rotation & other) const
{
  tf2Scalar d = distance(other);
  return d * d;
}

inline tf2Scalar Rotation::distance(const Rotation & other) const
{
  return (other - *this).angle();
}

inline Vector2 Rotation::rotate(const Vector2 & vec) const
{
  populateTrigCache();
  return Vector2(
    vec.x() * cos_angle_ - vec.y() * sin_angle_,
    vec.x() * sin_angle_ + vec.y() * cos_angle_);
}

inline Vector2 Rotation::unrotate(const Vector2 & vec) const
{
  // Populate the cache before inverse() is called.
  // The cache is propagated to inverse() temporary object, and we get
  // the benefit of having the cache populated for future calls using this object.
  populateTrigCache();
  return inverse().rotate(vec);
}

inline Rotation Rotation::inverse() const
{
  // Use trig identities to keep the trig cache populated
  return Rotation(-angle_, cos_angle_, -sin_angle_);
}

inline Rotation Rotation::absolute() const
{
  // A little effort to keep the trig cache populated
  if (angle_ < 0) {
    return inverse();
  } else {
    return *this;
  }
}

inline Rotation Rotation::lerp(const Rotation & other, const tf2Scalar ratio) const
{
  return Rotation(angle_ + ratio * (other - *this).angle());
}

inline const tf2Scalar & Rotation::getAngle() const
{
  return angle_;
}

inline void Rotation::setAngle(const tf2Scalar angle)
{
  setValue(angle);
}

inline void Rotation::setMax(const Rotation & other)
{
  if (other.angle_ > angle_) {
    *this = other;
  }
}

inline void Rotation::setMin(const Rotation & other)
{
  if (other.angle_ < angle_) {
    *this = other;
  }
}

inline void Rotation::setValue(const tf2Scalar angle)
{
  angle_ = angle;
  cos_angle_ = 0.0;
  sin_angle_ = 0.0;
  wrap();
}

inline void Rotation::setZero()
{
  angle_ = 0.0;
  cos_angle_ = 1.0;
  sin_angle_ = 0.0;
}

inline bool Rotation::isZero() const
{
  return angle_ == 0.0;
}

inline bool Rotation::fuzzyZero() const
{
  // Squaring to make it consistent with the Vector2 fuzzyZero check
  return (angle_ * angle_) < TF2SIMD_EPSILON;
}

inline Eigen::Matrix2d Rotation::getRotationMatrix() const
{
  populateTrigCache();
  Eigen::Matrix2d matrix;
  matrix << cos_angle_, -sin_angle_, sin_angle_, cos_angle_;
  return matrix;
}

inline Eigen::Matrix3d Rotation::getHomogeneousMatrix() const
{
  populateTrigCache();
  Eigen::Matrix3d matrix;
  matrix << cos_angle_, -sin_angle_, 0, sin_angle_, cos_angle_, 0, 0, 0, 1;
  return matrix;
}

inline Rotation & Rotation::wrap()
{
  // Handle the 2*Pi roll-over
  angle_ -= TF2SIMD_2_PI * std::floor((angle_ + TF2SIMD_PI) / TF2SIMD_2_PI);
  return *this;
}

inline void Rotation::populateTrigCache() const
{
  if ((cos_angle_ == 0) && (sin_angle_ == 0)) {
    cos_angle_ = tf2Cos(angle_);
    sin_angle_ = tf2Sin(angle_);
  }
}

inline Rotation operator+(Rotation lhs, const Rotation & rhs)
{
  lhs += rhs;
  return lhs;
}

inline Rotation operator-(Rotation lhs, const Rotation & rhs)
{
  lhs -= rhs;
  return lhs;
}

inline Rotation operator*(Rotation lhs, const tf2Scalar rhs)
{
  lhs *= rhs;
  return lhs;
}

inline Rotation operator*(const tf2Scalar lhs, Rotation rhs)
{
  rhs *= lhs;
  return rhs;
}

inline Rotation operator/(Rotation lhs, const tf2Scalar rhs)
{
  lhs /= rhs;
  return lhs;
}

inline Vector2 operator*(const Rotation & lhs, const Vector2 & rhs)
{
  return lhs.rotate(rhs);
}

inline std::ostream & operator<<(std::ostream & stream, const Rotation & rotation)
{
  return stream << "angle: " << rotation.angle();
}

}  // namespace tf2_2d

#endif  // TF2_2D__ROTATION_IMPL_HPP_
