# AWSIM - UNITY x FUTU-RE

![](/README_img/AWSIM.png)

AWSIM is the best scene simulator for [Autoware](https://github.com/autowarefoundation/autoware).

This scene has been modified for use with the FT Delivery vehicle. 

## Features

- Simulator components included (Vehicle, Sensor, Environment, ROS2, etc.)
- Support for Ubuntu 22.04 and windows10/11
- ROS2 native communication
- Open source software
- Made with Unity Game Engine

- Support for Mecanum Wheel Chair (2024-04-01)

## Tutorial

First, try the tutorial !  
[AWSIM Document - Quick Start Demo](https://tier4.github.io/AWSIM/GettingStarted/QuickStartDemo/)

## Documentation

https://tier4.github.io/AWSIM/

## License

AWSIM License
Applies to `tier4/AWSIM` repositories and all content contained in the [Releases](https://github.com/tier4/AWSIM/releases).

- code : Apache 2.0
- assets : CC BY-NC

See also [LICENSE](./LICENSE)

## Mecanum Wheelchair Support

A mecanum wheelchair simulator that subscribes tocpis for each motor `/wheel_XX/dcmotor/duty_ratio` (`XX`: wheel label (`fl`, `fr`, `rl` or `rr`).

![Mecanum Wheel Chair Simulated](</README_img/README_MecanumVehicleSimView.png>)

### How to Use Mecanum Simulator

- Run ]mecanum_drive_sim](https://github.com/Futu-reADS/mecanum_drive_sim.git), a stub for mecanum_drive

  - Prepare mecanum_drive_sim on your computer, if not

    ```git clone git@github.com:Futu-reADS/mecanum_drive_sim.git
    cd mecanum_drive_sim
    colcon build --symlink-install
    ```

  - Run mecanum_drive_sim

    (inside directory for mecanum_drive_sim)

    ```. install/setup.bash
    ros2 run mecanum_drive_sim phidgets_control_sim
    ```

 
- Run FT_MecanumSimulation stored inside AWSIM on Unity

    - Prepare AWSIM if not

    ```git clone git@github.com:Futu-reADS/AWSIM.git  # this takes several minutes
    git checkout ft_mecanum
    git pull
    ```

    - Run Unity Hub


## Contact

日本語/English OK

e-mail : liam.jemmeson.ki@futu-re.co.jp

(c) 2023 FUTU-RE Co. LTD.

e-mail : takatoki.makino@tier4.jp  
twitter : [mackierx111](https://twitter.com/mackierx111)  
discord : mackie#6141

(c) 2022 TIER IV, inc
