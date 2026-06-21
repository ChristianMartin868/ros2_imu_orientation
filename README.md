# ros2_imu_orientation

ROS-2-Node, der aus IMU-Rohdaten die Orientierung (3 DoF) als Quaternion
berechnet. **Portierung** des ROS-1-Studienprojekts aus dem Modul
*Autonome Systeme* (2020)
([`ros1_imu_orientation`](https://github.com/ChristianMartin868/ros1_imu_orientation)).

> Verwendet die **ROS 2**-API (`rclcpp`, `rclcpp::Node`, `rclcpp::spin`).
> Gebaut mit `ament_cmake` / `colcon`.

## Funktionsweise

`src/st_imu_pose_node.cpp` abonniert `imu/data_raw` (`sensor_msgs/msg/Imu`) und
schätzt die Orientierung in zwei unabhängigen Varianten:

- **Accelerometer** → Topic `poseAcc`: Der normierte Beschleunigungsvektor
  (Schwerkraft) wird gegen eine beim Start aufgenommene Referenz gestellt; die
  Drehung dazwischen ergibt das Quaternion.
- **Gyroskop** → Topic `poseGyro`: Die Winkelgeschwindigkeit (offset-korrigiert)
  wird über die Abtastzeit `dT` inkrementell integriert.

Beide werden als `geometry_msgs/msg/PoseStamped` im Frame `map` veröffentlicht
(an festen Positionen versetzt, damit sie sich in RViz nicht überlagern).

## Topics

| Richtung | Topic          | Typ                             |
|----------|----------------|---------------------------------|
| sub      | `imu/data_raw` | `sensor_msgs/msg/Imu`           |
| pub      | `poseAcc`      | `geometry_msgs/msg/PoseStamped` |
| pub      | `poseGyro`     | `geometry_msgs/msg/PoseStamped` |

## Abhängigkeiten im Code

Die Node nutzt zwei Hilfsklassen im Namespace `aut4`, die – wie im ROS-1-Original –
in diesem Auszug **nicht enthalten** sind und zum Bauen benötigt werden. Lege sie
unter `include/` ab:

- `include/vector3.h` — `aut4::Vector3` (3D-Vektor, `normalize()`, `operator[]`,
  `operator-=`)
- `include/quaternion.h` — `aut4::Quaternion` mit Feldern `x,y,z,w`,
  `operator*=` sowie den statischen Methoden
  `quaternionFromTwoVectors(...)` und `quaternionFromAngularVelocity(...)`

`CMakeLists.txt` bindet `include/` bereits per `target_include_directories` ein.

## Bauen & Ausführen

In einem colcon-Workspace (z. B. ROS 2 Humble / Jazzy):

```bash
# Repo nach <ws>/src/ klonen, vector3.h / quaternion.h nach include/ legen, dann:
colcon build --packages-select ros2_imu_orientation
source install/setup.bash
ros2 run ros2_imu_orientation st_imu_pose_node
```

Abhängigkeiten: ROS 2 mit `sensor_msgs`, `geometry_msgs`, `rclcpp`.

## Unterschiede zum ROS-1-Original

- `ros::init` + globale `NodeHandle`/Publisher → `rclcpp::init` +
  `rclcpp::Node`-Subklasse mit Member-Publishern.
- `n.subscribe` / `n.advertise` → `create_subscription` / `create_publisher`.
- `sensor_msgs::Imu` / `geometry_msgs::PoseStamped` → `…::msg::…`
  (Header `…/msg/*.hpp`).
- `ConstPtr` → `…::SharedPtr` im Callback.
- `ros::Time` / `ros::Duration` / `dT.toSec()` → `rclcpp::Time` /
  Differenz mit `.seconds()`.
- Funktionslokale `static`-Zustände (Referenzvektor, Gyro-Offset, integriertes
  Quaternion, vorheriger Zeitstempel) → Member der Node-Klasse mit
  `have_*`-Initialisierungsflags.
- Build: catkin → ament_cmake (`package.xml` format 3, `ament_package`).
