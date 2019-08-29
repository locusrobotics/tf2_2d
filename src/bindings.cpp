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
#include <tf2_2d/vector2.h>

#include <boost/python.hpp>
using namespace boost::python;  // NOLINT

/**
 * @file Wrap the tf2_2d objects for use in Python.
 *
 * This uses the Boost Python library:
 * https://www.boost.org/doc/libs/1_63_0/libs/python/doc/html/tutorial/index.html
 */

// TODO(swilliams) Not sure why the 'self_ns::' namespace is required.
// https://stackoverflow.com/questions/2828903/build-problems-when-adding-str-method-to-boost-python-c-class

// TODO(swilliams) Printing an object displays the object name and address. It might be nice to display the data
//                 instead. I'm not sure if that is possible.
// >>> v2 = Vector2(1, 2)
// >>> v2
// <tf2_2d.Vector2 object at 0x7f4f38ea1d60>

// TODO(swilliams) I skipped the functions that covert to Eigen objects. The Python equivalent is probably to return
//                 numpy objects. This is possible, and Boost Python has special support for numpy.

// TODO(swilliams) I skipped the minAxis(), maxAxis and similar functions. Aside from being mostly useless, I'd need
//                 to wrap the Axis enum, or otherwise figure out how to return the value.


BOOST_PYTHON_MODULE(tf2_2d)
{
  class_<tf2_2d::Vector2>("Vector2")
    .def(init<double, double>())
    .add_property("x", make_function(&tf2_2d::Vector2::getX, return_value_policy<copy_const_reference>()),
                       &tf2_2d::Vector2::setX)
    .add_property("y", make_function(&tf2_2d::Vector2::getY, return_value_policy<copy_const_reference>()),
                       &tf2_2d::Vector2::setY)
    .def("dot", &tf2_2d::Vector2::dot)
    .def("length2", &tf2_2d::Vector2::length2)
    .def("length", &tf2_2d::Vector2::length)
    .def("distance2", &tf2_2d::Vector2::distance2)
    .def("distance", &tf2_2d::Vector2::distance)
    .def("normalize", &tf2_2d::Vector2::normalize, return_value_policy<reference_existing_object>())
    .def("normalized", &tf2_2d::Vector2::normalized)
    .def("angle", &tf2_2d::Vector2::angle)
    .def("absolute", &tf2_2d::Vector2::absolute)
    .def("lerp", &tf2_2d::Vector2::lerp)
    .def("set_max", &tf2_2d::Vector2::setMax)
    .def("set_min", &tf2_2d::Vector2::setMin)
    .def("set_value", &tf2_2d::Vector2::setValue)
    .def("set_zero", &tf2_2d::Vector2::setZero)
    .def("is_zero", &tf2_2d::Vector2::isZero)
    .def("fuzzy_zero", &tf2_2d::Vector2::fuzzyZero)
    .def(self_ns::str(self))  // __str__
    .def(self + self)
    .def(self - self)
    .def(self * self)
    .def(double() * self)
    .def(self * double())
    .def(self / self)
    .def(self / double())
    .def("__eq__", &tf2_2d::Vector2::operator==)
    .def("__ne__", &tf2_2d::Vector2::operator!=);
}
