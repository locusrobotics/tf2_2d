^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tf2_2d
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
