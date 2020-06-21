# rclcpp_websocket


# Dependensies
* [ROS2 (Eloquent Elusor)](https://index.ros.org/doc/ros2/Installation/Eloquent/)
* [Boost](https://www.boost.org/)
* [Websocket++ (0.82)](https://github.com/zaphoyd/websocketpp) - C++ websocket client/server library

# Install

Install websocketpp

```
git clone https://github.com/zaphoyd/websocketpp
cd websocketpp
git checkout 0.82
mkdir build && cd build
cmake ..
make
sudo make install
```

Install boost

```
sudo apt install libboost-dev
```

Build from source

```
cd ros2_ws/src
git clone https://github.com/gusugusu1018/rclcpp_websocket
cd ..
colcon build --package-select rclcpp_websocket
source install/setup.bash // or setup.zsh
```

# Getting Started

run node

```
ros2 run rclcpp_websocket rclcpp_websocket
```

publish topic

```
ros2 topic pub /chatter std_msgs/String "data: Hello"
```

open browser

```
xdg-open viewer/index.html
```