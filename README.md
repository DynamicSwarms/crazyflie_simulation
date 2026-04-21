# Crazyflie Simulation

A Simulation for Crazyflie Quadcopters in ROS2. 

This software allows to control a large number of simulated crazyflies through a ROS2 interface.
Its key feature is that there are no dependencies apart from Eigen library, so no simulation framework such as Webots or Gazebo is neccessary.
It has support for the HighLevelCommander, the Logging and a limited GenericCommander Interface for the Crazyflie Quadcopter.
The interface implementations follow the ones in the [Crazyswarm2](https://github.com/IMRCLab/crazyswarm2) project, and should therefore be compatible.

This package is a submodule for [ds-crazyflie](https://github.com/DynamicSwarms/ds-crazyflies), we recommend to not use this software in standalone mode but in the DynamicSwarms ecosystem.


Build and testet with Ubuntu 22.04, ROS2 Humble.

# Standalone Installation and Usage

This software can also be used as a standalone simulator.

## Installation

### 1. Install dependencies:

1. If you have not already done so, install ROS2 with the following guide:

    https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

2. Install some ROS2 dependencies:

    ```
    sudo apt-get install ros-humble-tf-transformations
    ```


### 2. Build this repository with its dependencies in a dedicated workspace folder

1. Create a workspace folder, with a `src` folder inside

    ```
    mkdir ros2_ws
    cd ros2_ws
    mkdir src
    ```

2. Navigate into src and clone this repo as well as crazyflie_interfaces

    ```
    cd src
    git clone https://github.com/DynamicSwarms/crazyflie_simulation.git
    git clone https://github.com/DynamicSwarms/crazyflie_interfaces.git
    cd ..
    ```

3. Build the project from `ros2_ws` folder

    ```
    source /opt/ros/humble/setup.bash
    colcon build
    ```


## Using the Examples

There are two examples provided with this software. 
The simple example starts a crazyflie and moves it with some highlevel commands, it is supposed to help verify the correct installation. 
The second example allows to test with multiple crazyflies and allows for manual movement.

### The simple example
    
1. Start the script:

    ```
    source install/setup.bash
    ros2 launch crazyflie_simulation_examples example.launch.py 
    ```

2. Then in a second Terminal:

    ```
    source install/setup.bash
    ros2 topic echo /cf_positions
    ```

You should see the Crazyflie taking off (increasing z value).
Then flying to a predifined position. 
And finally landing (not at the origin).

### Scalability Test

1. In one terminal

    ```
    source install/setup.bash
    ros2 launch crazyflie_simulation_examples scalability.launch.py 
    ```
2. Verify the positions

    ```
    source install/setup.bash
    ros2 topic echo /cf_positions
    ```

    All crazyflies should report [0,0,0] as there position.

3. Move the crazyflie with goto commands:

    ```
    source install/setup.bash
    ros2 service call /cf49/go_to crazyflie_interfaces/srv/GoTo "group_mask: 0
        relative: false
        goal:
            x: 0.0
            y: 1.0
            z: 2.0
        yaw: 0.0
        duration:
            sec: 0
            nanosec: 0" 
    ```


## Advanced Usage

To use the full potential of this software the following gives a brief overview of its functionalities:

### Logging

The logging framework is implemented similarly to the one in the Crazyswarm2 implementation.
A short guide can be found [here](https://imrclab.github.io/crazyswarm2/howto.html#enabling-logblocks-at-runtime).

Additionally to adding and removing logblocks the following command allows to list all currently available logging variables:

```
ros2 topic pub /cf0/get_logging_toc_info std_msgs/msg/Empty --once
```

#### The `state` topic

The simulated crazyflie continously publishes onto the `cfID/state` topic with a frequency of 2Hz.
The values represent `pm.vbat, pm.chargeCurrent, pm.state, sys.canfly, sys.isFlying, sys.isTumbled`, as they are simulated `pm` values never change. 
Only the `sys.isFlying` vale updates.

### Getting crazyflie position in ROS2

In order not to overwhelm the tf-Graph positions are published on a seperate topic called `cf_positions`.
To view them: 

```
ros2 topic echo /cf_positions 
```

Also checkout the [DynamicSwarms-Documentation](https://dynamicswarms.github.io/ds-crazyflies/position_topics.html) regarding this topic.


### The Gateway

The launch file used in the SimpleUsage section is just a wrapper to start the gateway and immediately after adding the crazyflie with id 0.
Instead you can also: 

```
ros2 run crazyflie_simulation_gateway gateway 
```

and in a second terminal: 

```
ros2 service call /crazyflie_simulation_gateway/add_crazyflie crazyflie_interfaces/srv/AddCrazyflie 
    "uri: 'sim://0'
        initial_pose:
        position:
            x: 0.0
            y: 0.0
            z: 0.0
        orientation:
            x: 0.0
            y: 0.0
            z: 0.0
            w: 1.0
        type: default" 
```

Also check out the `crazyflie_spawner` script which does this efficiently for the scalability test.

### Sim Time

This software is desiged to be able to simulate the crazyflies in "faster than real-time" mode.
When the gateway spawns crazyflies it automatically passes its own "use_sim_time" parameter to them. 

In `scalability.launch.py` you can set `use_sim_time` for the gateway.
You now have to publish your own clock to the `\clock` topic. 

We provide a simple test clock node for this purpose: 

```
ros2 run crazyflie_simulation_examples clock --ros-args -p rate:=2.0
```


# The Future

## Jazzy

```
sudo apt-get install ros-jazzy-tf-transformations
```
