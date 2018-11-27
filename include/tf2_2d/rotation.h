/***************************************************************************
 * Copyright (C) 2017 Locus Robotics. All rights reserved.
 * Unauthorized copying of this file, via any medium, is strictly prohibited
 * Proprietary and confidential
 ***************************************************************************/
#ifndef TF2_2D_ROTATION_H
#define TF2_2D_ROTATION_H

#include <tf2/LinearMath/Scalar.h>
#include <tf2_2d/vector2.h>

#include <Eigen/Core>


namespace tf2_2d
{

/**
 * @brief A class representing a planar rotation
 *
 * The rotation angle is always within the range [-PI, PI]. This class mimics the tf2::Matrix3x3/tf2::Quaternion
 * classes to the extent possible.
 */
class Rotation
{
public:
  /**
   * @brief No initialization constructor
   */
  Rotation();

  /**
   * @brief Constructor from scalars
   *
   * @param angle The rotation angle in radians
   */
  explicit Rotation(const tf2Scalar angle);

  /**
   * @brief Return the reverse rotation
   */
  Rotation operator-() const;

  /**
   * @brief Add a rotation to this one
   *
   * @param rhs The rotation to add to this one
   */
  Rotation& operator+=(const Rotation& rhs);

  /**
   * @brief Subtract a rotation from this one
   *
   * @param rhs The rotation to subtract
   */
  Rotation& operator-=(const Rotation& rhs);

  /**
   * @brief Scale the rotation
   *
   * @param rhs Scale factor
   */
  Rotation& operator*=(const tf2Scalar rhs);

  /**
   * @brief Inversely scale the rotation
   *
   * @param rhs Scale factor to divide by
   */
  Rotation& operator/=(const tf2Scalar rhs);

  /**
   * @brief Check if two rotations are equal
   */
  bool operator==(const Rotation& other) const;

  /**
   * @brief Check if two vectors are not equal
   */
  bool operator!=(const Rotation& other) const;

  /**
   * @brief Return the distance squared between this and another rotation
   */
  tf2Scalar distance2(const Rotation& other) const;

  /**
   * @brief Return the distance between this and another rotation
   */
  tf2Scalar distance(const Rotation& other) const;

  /**
   * @brief Rotate a vector by this rotation
   *
   * @param vec The vector to rotate
   */
  Vector2 rotate(const Vector2& vec) const;

  /**
   * @brief Rotate a vector by the inverse of this rotation
   *
   * @param vec The vector to rotate
   */
  Vector2 unrotate(const Vector2& vec) const;

  /**
   * @brief Return the inverse of this rotation
   */
  Rotation inverse() const;

  /**
   * @brief Return a rotation with the absolute values of each element
   */
  Rotation absolute() const;

  /**
   * @brief Return the linear interpolation between this and another rotation
   *
   * The interpolation always uses the shortest path between this and other
   *
   * @param other The other rotation
   * @param ratio The ratio of this to other (ratio=0 => return this, ratio=1 => return other)
   */
  Rotation lerp(const Rotation& other, const tf2Scalar ratio) const;

  /**
   * @brief Return the angle value in radians
   */
  const tf2Scalar& getAngle() const;
  const tf2Scalar& angle() const { return getAngle(); }

  /**
   * @brief Set the angle value in radians
   */
  void setAngle(const tf2Scalar angle);

  /**
   * @brief Set the angle to the max of this and another rotation
   *
   * @param other The other Rotation to compare with
   */
  void setMax(const Rotation& other);

  /**
   * @brief Set the angle to the min of this and another rotation
   *
   * @param other The other Rotation to compare with
   */
  void setMin(const Rotation& other);

  /**
   * @brief Set the rotation to the provided angle in radians
   *
   * @param angle The angle value in radians
   */
  void setValue(const tf2Scalar angle);

  /**
   * Set the rotation to zero
   */
  void setZero();

  /**
   * Check if all the elements of this vector are zero
   */
  bool isZero() const;

  /**
   * Check if all the elements of this vector close to zero
   */
  bool fuzzyZero() const;

  /**
   * @brief Get a 2x2 rotation matrix
   */
  Eigen::Matrix2d getRotationMatrix() const;

  /**
   * @brief Get a 3x3 homogeneous transformation matrix with just the rotation portion populated
   */
  Eigen::Matrix3d getHomogeneousMatrix() const;

private:
  tf2Scalar angle_;  //!< Storage for the angle in radians
  mutable tf2Scalar cos_angle_;  //!< Storage for cos(angle) so we only compute it once
  mutable tf2Scalar sin_angle_;  //!< Storage for sin(angle) so we only compute it once

  /**
   * @brief Private constructor that allows populating the trig cache in special cases when it is known at construction
   *
   * @param[IN] angle     The rotation angle in radians
   * @param[IN] cos_angle The cos(angle) value; this is assumed to be correct
   * @param[IN] sin_angle The sin(angle) value; this is assumed to be correct
   */
  Rotation(const tf2Scalar angle, tf2Scalar cos_angle, tf2Scalar sin_angle);

  /**
   * @brief Wrap the rotation to the range [-Pi, Pi)
   */
  Rotation& wrap();

  /**
   * @brief Populate the cos/sin cache if it is not already populated.
   *
   * This only modifies mutable variables, so it is logically const.
   */
  void populateTrigCache() const;
};

/**
 * @brief Add two rotations
 */
Rotation operator+(Rotation lhs, const Rotation& rhs);

/**
 * @brief Subtract two rotations
 */
Rotation operator-(Rotation lhs, const Rotation& rhs);

/**
 * @brief Scale a rotation
 */
Rotation operator*(Rotation lhs, const tf2Scalar rhs);

/**
 * @brief Scale a rotation
 */
Rotation operator*(const tf2Scalar lhs, Rotation rhs);

/**
 * @brief Inversely scale a rotation
 */
Rotation operator/(Rotation lhs, const tf2Scalar rhs);

/**
 * @brief Rotate a vector by this rotation
 */
Vector2 operator*(const Rotation& lhs, const Vector2& rhs);

}  // namespace tf2_2d

#include <tf2_2d/rotation_impl.h>

#endif  // TF2_2D_ROTATION_H
