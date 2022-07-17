# Bobs very own bringup package!!!

## Contains launch files for launching bob's software either for simlation in gazebo or IRL

This ros2 package contains all the launch files required for launching the relevant ros nodes/ packages etc for bob to come alive. 

## Bringing up bob IRL

To wake bob up IRL, make sure that the relevant hardware are connected (e.g. odrive and motors) and run:

```
ros2 launch bob_base_bringup bob_base_bringup.launch.py
```

## Moving bob around

Bob uses ros2 control's [differential drive controller](https://github.com/ros-controls/ros2_controllers/tree/master/diff_drive_controller) for locomotion. To control her, publish `geometry_msgs/msg/Twist` messages to `/diffbot_base_controller/cmd_vel_unstamped`. 

To run a demo, install the [teleop_twist_keyboard](https://github.com/ros2/teleop_twist_keyboard) ros2 package, and run:

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Follow the instructions on the terminal in which you ran the command to control bob using your keyboard

