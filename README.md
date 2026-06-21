# ros2_imu_orientation

ROS 2 node that computes orientation (3 DoF) as a quaternion from raw IMU data.
**Port** of the ROS 1 study project from the *Autonomous Systems* module (2020)
([`ros1_imu_orientation`](https://github.com/ChristianMartin868/ros1_imu_orientation)).

> Uses the **ROS 2** API (`rclcpp`, `rclcpp::Node`, `rclcpp::spin`).
> Built with `ament_cmake` / `colcon`.

## How it works

`src/st_imu_pose_node.cpp` subscribes to `imu/data_raw` (`sensor_msgs/msg/Imu`)
and estimates orientation in two independent ways:

- **Accelerometer** → topic `poseAcc`: the normalised acceleration vector
  (gravity) is compared against a reference captured at startup; the rotation
  between them yields the quaternion.
- **Gyroscope** → topic `poseGyro`: the angular velocity (offset-corrected) is
  integrated incrementally over the sampling time `dT`.

Both are published as `geometry_msgs/msg/PoseStamped` in the `map` frame
(offset at fixed positions so they do not overlap in RViz).

## Topics

| Direction | Topic          | Type                            |
|-----------|----------------|---------------------------------|
| sub       | `imu/data_raw` | `sensor_msgs/msg/Imu`           |
| pub       | `poseAcc`      | `geometry_msgs/msg/PoseStamped` |
| pub       | `poseGyro`     | `geometry_msgs/msg/PoseStamped` |

## Code dependencies

The node uses two helper classes in the `aut4` namespace which — as in the
ROS 1 original — are **not included** in this excerpt and are required to build.
Place them under `include/`:

- `include/vector3.h` — `aut4::Vector3` (3D vector, `normalize()`, `operator[]`,
  `operator-=`)
- `include/quaternion.h` — `aut4::Quaternion` with fields `x,y,z,w`,
  `operator*=` and the static methods
  `quaternionFromTwoVectors(...)` and `quaternionFromAngularVelocity(...)`

`CMakeLists.txt` already adds `include/` via `target_include_directories`.

## Build & run

In a colcon workspace (e.g. ROS 2 Humble / Jazzy):

```bash
# Clone the repo into <ws>/src/, put vector3.h / quaternion.h into include/, then:
colcon build --packages-select ros2_imu_orientation
source install/setup.bash
ros2 run ros2_imu_orientation st_imu_pose_node
```

Dependencies: ROS 2 with `sensor_msgs`, `geometry_msgs`, `rclcpp`.

## Differences from the ROS 1 original

- `ros::init` + global `NodeHandle`/publishers → `rclcpp::init` +
  `rclcpp::Node` subclass with member publishers.
- `n.subscribe` / `n.advertise` → `create_subscription` / `create_publisher`.
- `sensor_msgs::Imu` / `geometry_msgs::PoseStamped` → `…::msg::…`
  (headers `…/msg/*.hpp`).
- `ConstPtr` → `…::SharedPtr` in the callback.
- `ros::Time` / `ros::Duration` / `dT.toSec()` → `rclcpp::Time` /
  difference via `.seconds()`.
- Function-local `static` state (reference vector, gyro offset, integrated
  quaternion, previous timestamp) → members of the node class with
  `have_*` initialisation flags.
- Build: catkin → ament_cmake (`package.xml` format 3, `ament_package`).
