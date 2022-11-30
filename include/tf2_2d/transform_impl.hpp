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
#ifndef TF2_2D__TRANSFORM_IMPL_HPP_
#define TF2_2D__TRANSFORM_IMPL_HPP_

#include <Eigen/Core>

#include <tf2/LinearMath/MinMax.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/utils.h>
#include <tf2_2d/rotation.hpp>
#include <tf2_2d/vector2.hpp>

namespace tf2_2d
{

inline Transform::Transform()
{
}

inline Transform::Transform(const Rotation & rotation, const Vector2 & translation)
: rotation_(rotation),
  translation_(translation)
{
}

inline Transform::Transform(const tf2Scalar x, const tf2Scalar y, const tf2Scalar yaw)
: rotation_(yaw),
  translation_(x, y)
{
}

inline Transform::Transform(const tf2::Transform & transform)
: rotation_(tf2::getYaw(transform.getRotation())),
  translation_(transform.getOrigin().getX(), transform.getOrigin().getY())
{
}

inline Transform & Transform::operator*=(const Transform & rhs)
{
  translation_ = translation_ + rotation_.rotate(rhs.translation_);
  rotation_ = rotation_ + rhs.rotation_;
  return *this;
}

inline bool Transform::operator==(const Transform & rhs)
{
  return (rotation_ == rhs.rotation_) && (translation_ == rhs.translation_);
}

inline bool Transform::operator!=(const Transform & rhs)
{
  return !operator==(rhs);
}

inline Transform Transform::lerp(const Transform & other, const tf2Scalar ratio) const
{
  // Following the tf2 3D implementation, interpolation of translation and rotation
  // is performed independently of each other
  return Transform(
    rotation_.lerp(other.rotation_, ratio),
    translation_.lerp(other.translation_, ratio));
}

inline const Rotation & Transform::getRotation() const
{
  return rotation_;
}

inline const Vector2 & Transform::getTranslation() const
{
  return translation_;
}

inline const tf2Scalar & Transform::getX() const
{
  return translation_.getX();
}

inline const tf2Scalar & Transform::getY() const
{
  return translation_.getY();
}

inline const tf2Scalar & Transform::getYaw() const
{
  return rotation_.getAngle();
}

inline void Transform::setRotation(const Rotation & other)
{
  rotation_ = other;
}

inline void Transform::setTranslation(const Vector2 & other)
{
  translation_ = other;
}

inline void Transform::setX(const tf2Scalar x)
{
  translation_.setX(x);
}

inline void Transform::setY(const tf2Scalar y)
{
  translation_.setY(y);
}

inline void Transform::setYaw(const tf2Scalar yaw)
{
  rotation_.setAngle(yaw);
}

inline void Transform::setIdentity()
{
  rotation_.setZero();
  translation_.setZero();
}

inline Transform Transform::inverse() const
{
  return Transform(rotation_.inverse(), rotation_.unrotate(-translation_));
}

inline Transform Transform::inverseTimes(const Transform & other) const
{
  return inverse() * other;
}

inline Eigen::Matrix3d Transform::getHomogeneousMatrix() const
{
  Eigen::Matrix3d matrix;
  matrix.topLeftCorner<2, 2>() = rotation_.getRotationMatrix();
  matrix.topRightCorner<2, 1>() = translation_.getVector();
  matrix.bottomRows<1>() << 0, 0, 1;
  return matrix;
}

inline Transform operator*(Transform lhs, const Transform & rhs)
{
  lhs *= rhs;
  return lhs;
}

inline Vector2 operator*(const Transform & lhs, const Vector2 & rhs)
{
  return lhs.rotation().rotate(rhs) + lhs.translation();
}

inline Rotation operator*(const Transform & lhs, const Rotation & rhs)
{
  return lhs.rotation() + rhs;
}

inline std::ostream & operator<<(std::ostream & stream, const Transform & transform)
{
  return stream << "x: " << transform.x() << ", y: " << transform.y() << ", yaw: " <<
         transform.yaw();
}

}  // namespace tf2_2d

#endif  // TF2_2D__TRANSFORM_IMPL_HPP_
