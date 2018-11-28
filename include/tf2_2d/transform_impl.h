/***************************************************************************
 * Copyright (C) 2017 Locus Robotics. All rights reserved.
 * Unauthorized copying of this file, via any medium, is strictly prohibited
 * Proprietary and confidential
 ***************************************************************************/
#ifndef TF2_2D_TRANSFORM_IMPL_H
#define TF2_2D_TRANSFORM_IMPL_H

#include <tf2/utils.h>                  // NOLINT: The tf2 MinMax.h file does not include all requirements. Consequently,
#include <tf2/LinearMath/Scalar.h>      // NOLINT: the order of the headers here is important.
#include <tf2/LinearMath/Transform.h>   //
#include <tf2/LinearMath/MinMax.h>      //
#include <tf2_2d/rotation.h>
#include <tf2_2d/vector2.h>

#include <Eigen/Core>

namespace tf2_2d
{

inline Transform::Transform()
{
}

inline Transform::Transform(const Rotation& rotation, const Vector2& translation) :
  rotation_(rotation),
  translation_(translation)
{
}

inline Transform::Transform(const tf2Scalar x, const tf2Scalar y, const tf2Scalar yaw) :
  rotation_(yaw),
  translation_(x, y)
{
}

inline Transform::Transform(const tf2::Transform &transform) :
  rotation_(tf2::getYaw(transform.getRotation())),
  translation_(transform.getOrigin().getX(), transform.getOrigin().getY())
{
}

inline Transform& Transform::operator*=(const Transform& rhs)
{
  translation_ = translation_ + rotation_.rotate(rhs.translation_);
  rotation_ = rotation_ + rhs.rotation_;
  return *this;
}

inline bool Transform::operator==(const Transform& rhs)
{
  return ((rotation_ == rhs.rotation_) && (translation_ == rhs.translation_));
}

inline bool Transform::operator!=(const Transform& rhs)
{
  return !operator==(rhs);
}

inline Transform Transform::lerp(const Transform& other, const tf2Scalar ratio) const
{
  // Following the tf2 3D implementation, interpolation of translation and rotation
  // is performed independently of each other
  return Transform(rotation_.lerp(other.rotation_, ratio), translation_.lerp(other.translation_, ratio));
}

inline const Rotation& Transform::getRotation() const
{
  return rotation_;
}

inline const Vector2& Transform::getTranslation() const
{
  return translation_;
}

inline const tf2Scalar& Transform::getX() const
{
  return translation_.getX();
}

inline const tf2Scalar& Transform::getY() const
{
  return translation_.getY();
}

inline const tf2Scalar& Transform::getYaw() const
{
  return rotation_.getAngle();
}

inline void Transform::setRotation(const Rotation& other)
{
  rotation_ = other;
}

inline void Transform::setTranslation(const Vector2& other)
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

inline Transform Transform::inverseTimes(const Transform& other) const
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

inline Transform operator*(Transform lhs, const Transform& rhs)
{
  lhs *= rhs;
  return lhs;
}

inline Vector2 operator*(const Transform& lhs, const Vector2& rhs)
{
  return lhs.rotation().rotate(rhs) + lhs.translation();
}

inline Rotation operator*(const Transform& lhs, const Rotation& rhs)
{
  return lhs.rotation() + rhs;
}

inline std::ostream& operator<< (std::ostream& stream, const Transform& transform)
{
  return stream << "x: " << transform.x() << ", y: " << transform.y() << ", yaw: " << transform.yaw();
}

}  // namespace tf2_2d

#endif  // TF2_2D_TRANSFORM_IMPL_H
