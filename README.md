# Follow-Me-Drone

## Setup

### Operating system:
Ubuntu 16.04.5

### ROS binary packages:
ros-kinetic-desktop-full    
ros-kinetic-ardrone-autonomy  
ros-kinetic-hector-*

### ROS source packages:
[angelsantamaria/tum_simulator](https://github.com/angelsantamaria/tum_simulator)

## Terminal commands

### Starting drone driver:
`rosrun ardrone_autonomy ardrone_driver`

### Starting simulation:
`roslaunch cvg_sim_gazebo ardrone_testworld.launch`

### Viewing video stream:
`rosrun image_view image_view image:="/ardrone/front/image_raw"`

### Recording video stream:
`rosrun image_view video_recorder image:="/ardrone/front/image_raw" _filename:="/path/to/video_file.avi"`
