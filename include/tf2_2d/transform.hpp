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
#ifndef TF2_2D__TRANSFORM_HPP_
#define TF2_2D__TRANSFORM_HPP_

#include <Eigen/Core>

#include <tf2/LinearMath/Scalar.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_2d/rotation.hpp>
#include <tf2_2d/vector2.hpp>


namespace tf2_2d
{

/**
 * @brief A class representing a 2D frame transformation or 2D pose
 *
 * This class mimics the tf2::Transform class to the extent possible.
 */
class Transform
{
public:
  /**
   * @brief No initialization constructor
   */
  Transform();

  /**
   * @brief Constructor from a rotation and translation
   *
   * @param rotation    Rotation
   * @param translation Translation
   */
  Transform(const Rotation & rotation, const Vector2 & translation);

  /**
   * @brief Constructor from individual x, y, yaw elements
   *
   * @param x The X value
   * @param y The Y value
   * @param yaw The yaw value in radians
   */
  Transform(const tf2Scalar x, const tf2Scalar y, const tf2Scalar yaw);

  /**
   * @brief Constructor from a tf2 Transform object
   *
   * @param transform - The tf2 Transform object from which we will initialize this object
   */
  explicit Transform(const tf2::Transform & transform);

  /**
   * @brief Compose this transform with another transform on the right
   */
  Transform & operator*=(const Transform & rhs);

  /**
   * @brief Test if two transforms have all elements equal
   */
  bool operator==(const Transform & rhs);

  /**
   * @brief Test if two transforms are not equal
   */
  bool operator!=(const Transform & rhs);

  /**
   * @brief Return the linear interpolation between this and another transform
   *
   * @param other The other transform
   * @param ratio The ratio of this to other (ratio=0 => return this, ratio=1 => return other)
   */
  Transform lerp(const Transform & other, const tf2Scalar ratio) const;

  /**
   * @brief Return the rotation for this transform
   */
  const Rotation & getRotation() const;
  const Rotation & rotation() const {return getRotation();}

  /**
   * @brief Return the translation for this transform
   */
  const Vector2 & getTranslation() const;
  const Vector2 & translation() const {return getTranslation();}

  /**
   * @brief Set the rotation for this transform
   */
  void setRotation(const Rotation & other);

  /**
   * @brief Set the translation for this transform
   */
  void setTranslation(const Vector2 & other);

  /**
   * @brief Return the X value
   */
  const tf2Scalar & getX() const;
  const tf2Scalar & x() const {return getX();}

  /**
   * @brief Return the Y value
   */
  const tf2Scalar & getY() const;
  const tf2Scalar & y() const {return getY();}

  /**
   * @brief Return the yaw value
   */
  const tf2Scalar & getYaw() const;
  const tf2Scalar & yaw() const {return getYaw();}
  // There are a lot of common aliases for yaw
  const tf2Scalar & getAngle() const {return getYaw();}
  const tf2Scalar & angle() const {return getYaw();}
  const tf2Scalar & getHeading() const {return getYaw();}
  const tf2Scalar & heading() const {return getYaw();}
  const tf2Scalar & getTheta() const {return getYaw();}
  const tf2Scalar & theta() const {return getYaw();}

  /**
   * @brief Set the X value
   */
  void setX(const tf2Scalar x);

  /**
   * @brief Set the Y value
   */
  void setY(const tf2Scalar y);

  /**
   * @brief Set the yaw value
   */
  void setYaw(const tf2Scalar yaw);
  // There are a lot of common aliases for yaw
  void setAngle(const tf2Scalar angle) {setYaw(angle);}
  void setHeading(const tf2Scalar heading) {setYaw(heading);}
  void setTheta(const tf2Scalar theta) {setYaw(theta);}

  /**
   * @brief Set this transformation to the identity
   */
  void setIdentity();

  /**
   * @brief Return the inverse of this transform
   */
  Transform inverse() const;

  /**
   * @brief Return the inverse of this transform times the other transform
   *
   * returns this.inverse() * other
   *
   * @param other The other transform
   */
  Transform inverseTimes(const Transform & other) const;

  /**
   * @brief Get a 3x3 homogeneous transformation matrix
   */
  Eigen::Matrix3d getHomogeneousMatrix() const;

private:
  Rotation rotation_;  //!< Storage for the rotation
  Vector2 translation_;  //!< Storage for the translation
};

/**
 * @brief Compose two transforms
 */
Transform operator*(Transform lhs, const Transform & rhs);

/**
 * @brief Return the transformed vector
 *
 * Output = This * Vector
 */
Vector2 operator*(const Transform & lhs, const Vector2 & rhs);

/**
 * @brief Return the transformed rotation
 */
Rotation operator*(const Transform & lhs, const Rotation & rhs);

/**
 * @brief Stream the transformation in human-readable format
 */
std::ostream & operator<<(std::ostream & stream, const Transform & transform);

}  // namespace tf2_2d

#include <tf2_2d/transform_impl.hpp>

#endif  // TF2_2D__TRANSFORM_HPP_
