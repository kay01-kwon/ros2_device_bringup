# ros2_device_bringup

## 1. Installation

```
mkdir -p ~/device_ws/src
```

```
cd ~/device_ws
```

```
colcon build --symlink-install
```

## 2. Launch px4

Terminal 1

```
cd ~/device_ws
```

```
source install/setup.bash
```

```
ros2 launch px4_launch px4.launch
```

Terminal 2 : Modify rate

```
cd ~/device_ws
```

```
source install/setup.bash
```

```
ros2 run px4_launch px4_client_node
```

## To do list

[] Mavros Local Position Estimator setting

[] Px4 --> EV Position offset setting

[] Create D435 launch file