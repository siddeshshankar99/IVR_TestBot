# 191.119 Autonomous Racing Cars (2024S) - Lecture WS

This repo provides you with a setup for developing with ROS2 in Docker.
It contains the F1Tenth simulator and a basic structure for developing your own nodes.

If you have no experience with Docker, we recommend this guide to get start: <!--TODO: link-->

Please feel free to further improve the layout of the workspace to your needs.
If you think an essential change would also greatly benefit other students, please create a merge request, and we will review the changes and, if they are beneficial, merge them.

If you have questions or discover things that are not working, open an issue or write your problem in the TUWEL forum.

## Structure

This workspace contains two different ways to run the simulator and develop with Docker:

- `docker-compose` comparably plain solution, graphical tools use a browser based VNC client to display windows
- `devcontainers` works with VSCode and JetBrains IDEs, launches the docker container and runs the IDE in the container, uses the native window system

The workspace uses the following directory structure:

```
.
├── docker-compose.yml -> file used by docker-compose
├── Dockerfile -> general Docker image description
├── get_env.sh -> script to print the environment (nice to have for JB IDEs & CMake)
├── maps -> mounted to /arc2024/maps/ in container
├── README.md
└── src -> mounted to /arc2024/ws/src/your_code/ in container
```

In addition, there are two hidden (at least on *nix systems) directories.
The `.cache` directory contains a build cache to improve build times.
The `.devcontainer` directory contains configuration files for `devcontainer`s.

Please note that all Docker-based approaches use `Dockerfile`.
Changes to any of the approaches in this file, will also be present in other environments.

Due to limited accessibility to MS Windows machines at the workgroup, we cannot guarantee compatibility with MS Windows.
If you have persistent problems with MS Windows, consider using a Virtual Machine with the Manual Installation instruction from `Lab 1`.

## Usage

If possible for you, we recommend to use `devcontainer`s with either VSCode or a JetBrains IDE.
VSCode is a free source code editor by Microsoft.
Students (and employees) of TU Wien can apply for academic licenses for JetBrains' all products pack, which includes CLion and PyCharm, which are full-fledged IDEs for C++ and Python respectively.

Alternatively, you can use `docker-compose`, which is just the bare system, without a source code editor or an IDE.

### `docker-compose`

**Dependencies**:
(also refer to ([https://docs.docker.com/engine/install](https://docs.docker.com/engine/install/) )

+ `docker`
+ `docker-compose`

To set up the system, first clone the repository.

```shell
git clone ssh://git@gitlab.tuwien.ac.at:822/cyber-physical-systems/lehre-public/arc2024/lecture-ws.git
```

First, build the Docker image and pull all required images.
Execute this command in the `lecture-ws` directory.

```shell
docker compose build
```

To start the environment, run the following command in the same directory.

```shell
docker compose up
```

This will start two containers, one for the display server and one that contains the simulator.

You can access the simulation container with (in another terminal):

```shell
docker exec -it lecture-ws-sim-1 /bin/bash
```

Execute the above command every time you need a new terminal session.

To see the graphical environment open [http://localhost:8000/vnc.html](http://localhost:8000/vnc.html) and click on `Connect`.

Alternatively you can use the docker-compose file `docker-compose-x11-fwd.yml` (instead of `docker-compose.yml`) that uses direct X11-forwarding to the host machine.

### VSCode `devcontainer`

**Dependencies**:
(also refer to ([https://docs.docker.com/engine/install](https://docs.docker.com/engine/install/) )

+ `docker`
+ VSCode
+ Remote Development Extension in VS Code

To set up the system, first clone the repository.

```shell
git clone ssh://git@gitlab.tuwien.ac.at:822/cyber-physical-systems/lehre-public/arc2024/lecture-ws.git
```

Navigate to the `.devcontainer/devcontainer.json` (`.devcontainer/.devcontainer-windows/devcontainer.json` in MS Windows) file.
Then use `View>Command Palette...` for the command `Dev containers: (Re-)build and Reopen in Container`.
This will initialize and start the `devcontainer` in VSCode.

After VSCode is finished with loading and set up, open a terminal in VSCode.
From now on, we will use the shell in this terminal to execute the commands.

### JetBrains `devcontainer`

**Dependencies**:
(also refer to ([https://docs.docker.com/engine/install](https://docs.docker.com/engine/install/) )

+ `docker`
+ JetBrains IDE of your choice (EAP versions recommended!)

Navigate to the `.devcontainer/devcontainer.json` (`.devcontainer/.devcontainer-windows/devcontainer.json` in MS Windows) file.
Click on the devcontainer icon (3D-ish cube) in the Gutter (next to the line numbers) and select `Create Dev Container and Mount Sources`.
After building has completed, the tool window will show a selection of IDE backends.
Pick the one corresponding to the IDE you intend to use and press `Continue`.
This will start a new instance of the IDE with a connection to the `devcontainer`.

**IMPORTANT: X-Forwarding**

JetBrains IDEs do not support the use of `--net=host` for docker.
It is therefore necessary to disable this but also to open up your X Server.

Run this to disable authorization on your X-Server

```shell
xhost +
```

**CAUTION:** This allows any and all hosts from anywhere to use your X-Server (graphical interface) to display windows on your PC! **UNDER NO CIRCUMSTANCES do this on an untrusted network!**

In the lower left if the JetBrains IDE window, click on the Terminal symbol to open the terminal tool window.
From now on, we will use these to execute commands.

## Starting the Simulator

Before you can run the simulator you need to build the workspace to make all packages in it available to you.
Make sure you run this in `/arc2024/ws/`.

```shell
source /opt/ros/foxy/setup.bash
colcon build
```

The first command sets up the ROS2 environment in your current session and makes everything from the global workspace available.
The second command invokes the build tool used in ROS2 called `colcon` to build all packages.

After the build is finished you can run the simulator:

```shell
source install/setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

The first command sets up the workspace located at `/arc2024/ws/` and makes all packages (initially only the simulator) in that workspace available to you.
The second command uses the ROS2 launch system to start the simulator and all dependencies.

This will output a lot of warning messages during the startup of the simulator:

```text
[rviz2-1] Warning: Invalid frame ID "map" passed to canTransform argument target_frame - frame does not exist
[rviz2-1]          at line 133 in /tmp/binarydeb/ros-foxy-tf2-0.13.14/src/buffer_core.cpp
```

You can ignore these messages.

After the messages have stopped flooding your terminal session, you should see the car in a visualization window (`rviz2`) and the lidar sensor visualized as colorful points along the wall.
To move the car, you can use the package `teleop_twist_keyboard` that is already installed in the container.
Run the following commands in a new(!) terminal session to start the keyboard control.

```shell
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

You can now (very crudely) control your car with your keyboard.
