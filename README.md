This is a simple demonstrated project for the trajectory planning and control in Coppliasim with a two-link arm robot.

The trajectory for the robot arm is generated with the Position-Velocity-Time (PVT) trajectory interpolation, *i.e*, a polynomial trajectory interpolation approach.

The control of the simple robot arm is with PID joint position controllers.

There is a robot kinematics model for the two-link arm robot, pleas refer to the source code for details.



The demonstrated videos can be found as follows：

1.The demo for the simple control of the robot arm, with two joints.

<img src="video/two_link_demo_1.gif" alt="two_link_demo_1" style="zoom:150%;" />

<img src="video/two_link_demo_2.gif" alt="two_link_demo_2" style="zoom:150%;" />



2.The demo for the trajectory planning and control with some given via-points.

<img src="video/two_link_trajectory.gif" alt="two_link_trajectory" style="zoom:150%;" />
