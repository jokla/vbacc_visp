# Vision-Based Adaptive Cruise Control



Subscriber:
* Intrinsic camera parameters sensor_msgs::CameraInfo
* Polygon of the object detected: geometry_msgs::Polygon

Publisher:
* Command velocities in geometry_msgs::TwistStamped

An Image-Based Visual Servoing is implemented to follow an object. The current visual features that are used are s = (x, log(Z/Z*)). The desired one are s* = (x*, 0), with:

- x the abscissa of the point corresponding to the center of gravity measured at each iteration,
- x* the desired abscissa position of the point (x* = 0)
- Z the depth of the point measured at each iteration
- Z* the desired depth of the point
The degrees of freedom that are controlled are Vx and Wz where Wz is the rotational velocity and Vx the translational velocity of the robot.

The value of Z is estimated from the area of the bounding box that is proportional to the depth Z.

For the Visual Servoing I used ViSP:
https://visp.inria.fr/

To detect and track the object ROS OpenTLD is used:
https://github.com/pandora-auth-ros-pkg/open_tld
https://github.com/pandora-auth-ros-pkg/pandora_tld


To launch:
`$ roslaunch vbacc vbacc.launch`
