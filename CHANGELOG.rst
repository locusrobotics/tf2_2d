^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tf2_2d
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.5.0 (2025-07-28)
------------------

1.4.0 (2025-07-28)
------------------
* Updated tf2 header extensions from .h to .hpp (`#11 <https://github.com/locusrobotics/tf2_2d/issues/11>`_)
* Contributors: Stephen Williams, locus-services

1.3.0 (2025-02-04)
------------------
* Tailor: Updating Jenkinsfile
* Tailor: Updating Jenkinsfile
* Tailor: Updating Jenkinsfile
* Tailor: Updating Jenkinsfile
* Tailor: Updating Jenkinsfile
* Contributors: locus-services

1.2.0 (2024-09-16)
------------------

1.1.0 (2024-06-17)
------------------
* Exporting boost dependency (#7)
  * Exporting Boost and Eigen dependencies
* Tailor: Updating Jenkinsfile
* Contributors: Tom Moore, locus-services

1.0.1 (2023-03-03)
------------------
* Port tf2_2d to ROS 2 (`#5 <https://github.com/locusrobotics/tf2_2d/issues/5>`_)
  * Build sytem changes for ROS 2
  * Code changes for ROS 2
  * Don't need int main(); because linking to GTEST_MAIN_LIBRARIES
  * Skip ament_cmake_copyright
  * Move .h headers to .hpp
  * Add .h headers with warning for backwards compatability
  * Linter fixes: Satisfy Uncrustify
  * Linter fixes: Satisfy cpplint
  * Minimum CMake 3.14.4
  * ${tf2_geometry_msgs_TARGETS} -> tf2_geometry_msgs::tf2_geometry_msgs
  * Bump copywrite date on redirection headers
  * Remove Scalar.h include
* Tailor: Updating Jenkinsfile
* Tailor: Updating Jenkinsfile
* Tailor: Updating Jenkinsfile
* Contributors: Shane Loretz, locus-services

0.6.4 (2021-07-14)
------------------
* Handling issue with tf2 circular dependency (`#4 <https://github.com/locusrobotics/tf2_2d/issues/4>`_)
  * Handling issue with tf2 circular dependency and other warnings
* Contributors: Tom Moore

0.6.3 (2021-07-13)
------------------
* Fixing package license
* Contributors: Tom Moore

0.6.2 (2021-07-12)
------------------
* Fix test build issue in noetic
* Contributors: Tom Moore

0.6.1 (2021-07-11)
------------------
* Contributors: locus-services

0.6.0 (2020-10-02)
------------------
* Contributors: locus-services

0.5.0 (2019-07-12)
------------------
* License cleanup (`#3 <https://github.com/locusrobotics/tf2_2d/issues/3>`_)
* Contributors: Stephen Williams, locus-services

0.4.0 (2019-03-18)
------------------
* Contributors: locus-services

0.3.0 (2019-01-16)
------------------
* [RST-1548] transform covariance (`#2 <https://github.com/locusrobotics/tf2_2d/issues/2>`_)
  * Added conversions to Eigen types
  * Added stream operators
  * Added some additional toMsg conversions using a non-tf2-standard signature
  * Added functions to transform SE2 covariance matrices
* Tailor: Creating Jenkinsfile
* Expanded readme
* Renamed the Transform rotation value to 'yaw'
* Moved into separate repo
* Contributors: Stephen Williams, locus-services

0.2.0 (2018-04-16)
------------------
* Adding tf_2d constructor overload from a standard tf transform
* Contributors: Stephen Williams, Tom Moore

0.1.0 (2018-02-14)
------------------
* Small trig cache optimization in the 'unrotate()' function
* Added 2D geometry classes (vector, rotation, transform) and conversion functions in the style of the tf2 3D geometry classes
* Contributors: Stephen Williams
