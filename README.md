# qflight_descriptions
#### Qualcomm Flight Board and Vehicle URDFs

This ROS package contains URDFs, a launch file, and meshes for Qualcomm Flight<sup>TM</sup> boards and vehicles

### Getting Started

Clone this repo into your catkin_ws on your workstation:

```bash
cd ~/catkin_ws/src
git clone https://github.com/ATLFlight/qflight_descriptions.git
rosdep install . --from-paths
```

If you are using a Qualcomm Flight<sup>TM</sup> board with a [Dragon Drone Development Kit](https://worldsway.com/product/dragon-drone-development-kit), just run:


```bash
roslaunch qflight_descriptions qflight_robot.launch rviz:=true
```

You should see The vehicle, board and all associated TF coordinate frames. Note that this package uses the non-standard robot_description names: board_desription and vehicle_description.  If you are using the DDK without the stereo upgrade kit, run:

```bash
roslaunch qflight_descriptions qflight_robot.launch rviz:=true board:=sdf vehicle:=ddk
```

You should now see the vehicle and board without the stereo tray and also without the associated left and right stereo TF frames.  Note that if and when you install and run this repository on your target board, you should also have a checkout in your ROS workspace on your workstation so that you can see the vehicle/board meshes in Rviz.

### Notes

This ros package uses scripts/base_link_imu_static_publisher.py to connect the TF trees of the flight board and the vehicle. In general, the vehicle URDF will be rooted at base_link and the board rooted at "imu".  The script will publish the transform between them by way of the "flight_board_center" frame.  The offsets are defined in the urdf/board_mounts.yaml file.  The reason for this extra step is that with the param "imu_is_root_tf", the user can either choose for "imu" to be the root of the joint tree or "base_link" to be the root.  When using [snav_ros](https://github.com/ATLFlight/snav_ros), base_link should be the root tf, as snav_ros publishes /odom->/base_link; however, when using [snap_vio](https://github.com/ATLFlight/snap_vio), imu should be the root tf, as snap_vio publishes /imu_start->/imu.

### Adding a new vehilce type

To add a new vehicle, simply add an STL of the vehilce to the meshes foler, a URDF to the urdf folder, and add an entry for the vehicle name into the urdf/board_mounts.yaml folder defining the position of the flight board on the vehilce.
