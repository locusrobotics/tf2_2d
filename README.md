# tf2_2d
A set of 2D geometry classes modeled after the 3D geometry classes in tf2.

## Using tf2 toMsg() and fromMsg()
I've tried to include fromMsg() implementations for anything that remotely makes sense. The tf2 toMsg() signature
does limit its use to a single output type, so I've had to make a guess as to the most useful variation there.

For example:
```
#include <tf2_2d/tf2_2d.h>     // This header includes the tf2 conversion functions
#include <tf2_2d/transform.h>  // Then include what you use

// Convert a tf2_2d object into a 3D message
auto transform_2d = tf2_2d::Transform(1.0, 1.5, 2.0);
geometry_msgs::Transform transform_3d_msg = tf2::toMsg(transform_2d);

// Convert a 3D message into a 2D tf2_2d object
geometry_msgs::Transform transform_3d_msg;
tf2_2d::Transform transform_2d;
tf2::fromMsg(transform_3d_msg, transform_2d);

// You can do Stamped<> things as well
auto transform_2d = tf2::Stamped<tf2_2d::Transform>(tf2_2d::Transform(1.0, 1.5, 2.0), ros::Time(1.2), "frame");
geometry_msgs::TransformStamped transform_3d_msg = tf2::toMsg(transform_2d);
```
Conversion of Points and Quaternions are also supported, as are conversions to other datatypes that are handled by tf2's conversion system.

## Transformation math
The `tf2_2d` types also implement all of the expected transformation math.
```
auto v1 = tf2_2d::Vector2(1.0, 2.0);
auto v2 = tf2_2d::Vector2(1.5, 2.5);
auto v3 = v1 + v2;                             // v3 == (2.5, 4.5)

tf2_2d::Rotation r1(M_PI);
auto v4 = r1.rotate(v1);                       // v4 == (-2.0, 1.0)

auto t1 = tf2_2d::Transform(1.0, 2.0, 3.0);
auto t2 = tf2_2d::Transform(-2.0, -1.0, -1.5);
auto t3 = t1 * t2;                             // t3 == (3.12, 2.70, 1.5)

auto t4 = tf2_2d::Transform(1.0, 2.0, 3.0);
auto t5 = tf2_2d::Transform(-2.0, -1.0, -1.5);
auto t6 = t4.inverseTimes(t5);                 // t6 == (2.54, 3.39, 1.78)
```

The `tf2_2d::Rotation` class deserves a few additional notes. The angle stored in a `Rotation` object is always
within the (-Pi, Pi] range. You can construct a `Rotation` object with any floating point value, but it will be
wrapped to that range. You can also perform arithmetic with the angles, without worrying about wrapping issues.
```
tf2_2d::Rotation r1(1.0);
auto r2 = 17.0 * r1;       // r2.angle() == -1.84956

tf2_2d::Rotation r3(1.0);
tf2_2d::Rotation r4(3.0);
auto r5 = r3 + r4;         // r5.angle() == -2.28319

tf2_2d::Rotation r6(-3.0);
tf2_2d::Rotation r7(1.0);
auto r8 = r6 - r7;         // r8.angle() == 2.28319
```

Additionally, the `tf2_td::Rotation` class caches the sin/cos results needed to perform rotations. So, once a
`Rotation` object is used to rotate something, the trig functions will never be evaluated again. And the cached
sin/cos values will propagate to any derived objects that it can. This includes the `Rotation` object built into
a `Transform`.
```
tf2_2d::Vector2 v1(1.0, 2.0);
tf2_2d::Rotation r1(1.0);            // No sin/cos calls have been made, since it is not needed yet
auto v2 = r1.rotate(v1);             // Computes sin/cos and remembers it
auto v3 = r1.unrotate(v2);           // Does not need to compute sin/cos again
tf2_2d::Rotation r2 = r1;            // Transfers sin/cos to r2
auto v4 = r2.rotate(v1);             // Does not need to compute sin/cos because it stole the cached values from r1
tf2_2d::Rotation r3 = r1.inverse();  // Inverts and transfers sin/cos to r3
auto v5 = r3.rotate(v1);             // Does not need to compute sin/cos because it stole the cached values from r1
```
