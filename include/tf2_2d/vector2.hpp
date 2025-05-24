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
#ifndef TF2_2D__VECTOR2_HPP_
#define TF2_2D__VECTOR2_HPP_

#include <Eigen/Core>

#include <tf2/LinearMath/Scalar.hpp>


namespace tf2_2d
{

/**
 * @brief A class representing a 2D vector or 2D point
 *
 * This class mimics the tf2::Vector3 class to the extent possible.
 */
class Vector2
{
public:
  /**
   * @brief Axis constants
   */
  enum Axis
  {
    X = 0,  //!< X axis
    Y = 1   //!< Y axis
  };

  /**
   * @brief No initialization constructor
   */
  Vector2();

  /**
   * @brief Constructor from scalars
   *
   * @param x X value
   * @param y Y value
   */
  Vector2(const tf2Scalar x, const tf2Scalar y);

  /**
   * @brief Return the negative of the vector
   */
  Vector2 operator-() const;

  /**
   * @brief Add a vector to this one
   *
   * @param rhs The vector to add to this one
   */
  Vector2 & operator+=(const Vector2 & rhs);

  /**
   * @brief Subtract a vector from this one
   *
   * @param rhs The vector to subtract
   */
  Vector2 & operator-=(const Vector2 & rhs);

  /**
   * @brief Scale the vector
   *
   * @param rhs Scale factor
   */
  Vector2 & operator*=(const tf2Scalar rhs);

  /**
   * @brief Element-wise multiply this vector by the other
   *
   * @param v The other vector
   */
  Vector2 & operator*=(const Vector2 & rhs);

  /**
   * @brief Inversely scale the vector
   *
   * @param rhs Scale factor to divide by
   */
  Vector2 & operator/=(const tf2Scalar rhs);

  /**
   * @brief Element-wise divide this vector by the other
   *
   * @param rhs The other vector
   */
  Vector2 & operator/=(const Vector2 & rhs);

  /**
   * @brief Check if two vectors are equal
   */
  bool operator==(const Vector2 & other) const;

  /**
   * @brief Check if two vectors are not equal
   */
  bool operator!=(const Vector2 & other) const;

  /**
   * @brief Return the dot product
   *
   * @param v The other vector in the dot product
   */
  tf2Scalar dot(const Vector2 & other) const;

  /**
   * @brief Return the length of the vector squared
   */
  tf2Scalar length2() const;

  /**
   * @brief Return the length of the vector
   */
  tf2Scalar length() const;

  /**
   * @brief Return the distance squared between the ends of this and another vector
   *
   * This is semantically treating the vector like a point
   */
  tf2Scalar distance2(const Vector2 & other) const;

  /**
   * @brief Return the distance between the ends of this and another vector
   *
   * This is semantically treating the vector like a point
   */
  tf2Scalar distance(const Vector2 & other) const;

  /**
   * @brief Normalize this vector
   *
   * x^2 + y^2 = 1
   */
  Vector2 & normalize();

  /**
   * @brief Return a normalized version of this vector
   */
  Vector2 normalized() const;

  /**
   * @brief Return the angle between this and another vector
   *
   * @param other The other vector
   */
  tf2Scalar angle(const Vector2 & other) const;

  /**
   * @brief Return a vector with the absolute values of each element
   */
  Vector2 absolute() const;

  /**
   * @brief Return the axis with the smallest value
   *
   * Note return values are 0,1 for x or y
   */
  Axis minAxis() const;

  /**
   * @brief Return the axis with the largest value
   *
   * Note return values are 0,1 for x or y
   */
  Axis maxAxis() const;

  /**
   * @brief Returns the axis with the smallest absolute value
   *
   * Note return values are 0,1 for x or y
   * Seems like a poorly named function, but mimicking the tf2::Vector3 class
   */
  Axis furthestAxis() const;

  /**
   * @brief Returns the axis with the largest absolute value
   *
   * Note return values are 0,1 for x or y
   * Seems like a poorly named function, but mimicking the tf2::Vector3 class
   */
  Axis closestAxis() const;

  /**
   * @brief Return the linear interpolation between this and another vector
   *
   * @param other The other vector
   * @param ratio The ratio of this to other (ratio=0 => return this, ratio=1 => return other)
   */
  Vector2 lerp(const Vector2 & other, const tf2Scalar ratio) const;

  /**
   * @brief Return the x value
   */
  const tf2Scalar & getX() const;
  const tf2Scalar & x() const {return getX();}

  /**
   * @brief Return the y value
   */
  const tf2Scalar & getY() const;
  const tf2Scalar & y() const {return getY();}

  /**
   * @brief Set the x value
   */
  void setX(const tf2Scalar x);

  /**
   * @brief Set the y value
   */
  void setY(const tf2Scalar y);

  /**
   * @brief Set each element to the max of the current values and the values of another Vector3
   *
   * @param other The other Vector3 to compare with
   */
  void setMax(const Vector2 & other);

  /**
   * @brief Set each element to the min of the current values and the values of another Vector3
   *
   * @param other The other Vector3 to compare with
   */
  void setMin(const Vector2 & other);

  /**
   * @brief Set each element to the provided value
   *
   * @param x The X value
   * @param y The Y value
   */
  void setValue(const tf2Scalar x, const tf2Scalar y);

  /**
   * Set all elements of this vector to zero
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
   * @brief Get an Eigen vector representation
   */
  Eigen::Vector2d getVector() const;

  /**
   * @brief Get a 3x3 homogeneous transformation matrix with just the translation portion populated
   */
  Eigen::Matrix3d getHomogeneousMatrix() const;

private:
  tf2Scalar x_;  //!< Storage for the X value
  tf2Scalar y_;  //!< Storage for the Y value
};

/**
 * @brief Add two vectors
 */
Vector2 operator+(Vector2 lhs, const Vector2 & rhs);

/**
 * @brief Subtract two vectors
 */
Vector2 operator-(Vector2 lhs, const Vector2 & rhs);

/**
 * @brief Scale a vector
 */
Vector2 operator*(Vector2 lhs, const tf2Scalar rhs);

/**
 * @brief Scale a vector
 */
Vector2 operator*(const tf2Scalar lhs, Vector2 rhs);

/**
 * @brief Element-wise multiplication of two vectors
 */
Vector2 operator*(Vector2 lhs, const Vector2 & rhs);

/**
 * @brief Inversely scale a vector
 */
Vector2 operator/(Vector2 lhs, const tf2Scalar rhs);

/**
 * @brief Element-wise division of two vectors
 */
Vector2 operator/(Vector2 lhs, const Vector2 & rhs);

/**
 * @brief Stream the vector in human-readable format
 */
std::ostream & operator<<(std::ostream & stream, const Vector2 & vector);

}  // namespace tf2_2d

#include <tf2_2d/vector2_impl.hpp>

#endif  // TF2_2D__VECTOR2_HPP_
