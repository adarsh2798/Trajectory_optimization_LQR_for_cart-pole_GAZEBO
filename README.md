# Trajectory_optimization_LQR_for_cart-pole_GAZEBO

In this project, I have attemped to swing up and balance the cart-pole system in ROS Gazebo. The swing up ws achieved by trajectory optimization, where trapezoidal collocation was used to discretize the continuous trajectories and convert the "optimization of functionals" to a discrete Non Linear Program, whicch was then numerically solved using python's scipy.minimze package.
To capture the pole in up position and balance simple LQR was implemented.
Here you can find the ROS package uploaded that contains the necesssary files for urdf model and .stl files for simulating cart-pole system in gazebo. It also contains the python scripts which i wrote to implement my control algorithm. They can be found in "cart_pole_LQR/src/commander/scripts" directory.

**<span style="font-size:24px;">NOTE: I used the ROS package for necessary urdf files to simulate cart-pole in gazebo from these links: [package link1](https://drive.google.com/drive/folders/1Jm95jbwaTYvgVHVDriG0kf2x7KYrbRvT) and [package link2 in youtube video's decription](https://www.youtube.com/watch?v=dLnKvFEnSBw&t=1s)
</span>**

Below is a gif of the simulation in gazebo.
