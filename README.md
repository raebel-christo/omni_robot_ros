# ROS Omni Directional Bot
Project is built in ROS Foxy, upgrading it to higher versions might require fixes in other dependancies, specifically YDLIDAR drivers.

## Boot up commands for raspberry pi:
On boot up, run [^1] :
```bash
. run_docker.sh
```
This command runs a script to launch the docker file. Alternatively, you can use the command:
```bash
docker run -i --rm --net=host --privileged -t raebelchristo/rpilidar:eleven
```
Please note that `raebelchristo/rpilidar:eleven` will need to replaced with the correct image name of your latest docker image if you made any commits.

## Setup commands inside docker

Run these commands to setup permissions for lidar and selecting cycloneDDS [^2]:
```bash
cd
sudo bash run_commands.bash
```
The password for sudo inside the docker container is the default `ubuntu`

## Updating the ROS Nodes
Follow these commands:
```bash
cd lidar_ws/src/omni_bot_ros
git pull
cd ../..
colcon build --symlink-install --packages-select omni_bot
source install/setup.bash
```
## Launch the bot
Inside the `lidar_ws` workspace:
```bash
cd src/omni_bot_ros/launch
ros2 launch bot.launch.py
```
## PC side control setup
Clone the [fyp_bot_description ROS package](https://github.com/raebel-christo/fyp_bot_description) on to the desktop in a suitable workspace.
1. Build the package using `colcon build`
2. Source the setup file of your workstation.
3. `ros2 launch fyp_bot_description bot.launch.py`

This sets up the robots odometry and joint state publisher. This allows the bot to be ready to be used with AMCL or SLAM

## Mapping
1. Make sure the `omni_bot` nodes are running and `fyp_bot_description` nodes are running.
2. Run `ros2 launch slam_toolbox online_sync_launch.py use_sim_time:=false`
3. Open `rviz2` and check if mapping works

## Navigation
Follow tutorials from [Navigation2 Framework](https://docs.nav2.org)
Example command: `ros2 launch nav2_bringup bringup_launch.py`
You will need to tweak parameters to allow the robot to function properly.
```bash
sudo gedit /opt/ros/foxy/share/nav2_bringup/params/nav2_param.yaml
```
Ensure to change the differential configuration to omnidirection in the parameter file with reference to nav2 documentation depending on ros version.

## Debugging
### Topics are not visible across devices
Use the following commands to check environment variables:
```bash
printenv | grep DOMAIN
printenv | grep IMPLEMENTATION
```
The above commands will return `ROS_DOMAIN_ID` and `RMW_IMPLEMENTATION` variables. `ROS_DOMAIN_ID` must be either unset or the same across all devices. `RMW_IMPLEMENTATION` must be set to `rmw_cyclonedds_cpp`.
Use `export` command to set environment variable and `unset` command to remove an environment variable.
### Map is not generating
Try refreshing the map layer in `rviz2`
Alternatively edit `/opt/ros/foxy/share/slam_toolbox/config/mapper_params_online_sync.yaml` and update `base_frame` parameter from `base_footprint` to `base_link`.

### Notes:
[^1]: Ensure that the lidar is connected to the raspberry pi before launching docker
[^2]: All necessary commands are placed inside the bash file
