# carla_its_converter

This package contains the CarlaItsConverterNode - a simple ROS Node that converts incoming messages from the [carla-ros-bridge](https://gitlab.ika.rwth-aachen.de/fb-fi/simulation/carla/ros-bridge) and publishes some of the [fb-fi defined ros messages](https://gitlab.ika.rwth-aachen.de/fb-fi/definitions) for various its-applications.

- [Nodes](#nodes)
  - [carla_its_converter/CarlaItsConverterNode](#carla_its_convertercarlaitsConverternode)
- [Usage of docker-ros Images](#usage-of-docker-ros-images)
  - [Available Images](#available-images)
  - [Default Command](#default-command)
  - [Launch Files](#launch-files)
  - [Configuration Files](#configuration-files)
  - [Additional Remarks](#additional-remarks)
- [Official Documentation](#official-documentation)


## Nodes

| Package | Node | Description |
| --- | --- | --- |
| `carla_its_converter` | `CarlaItsConverterNode` | Converting carla-ros-messages to fb-fi defined its-messages |

### carla_its_converter/CarlaItsConverterNode

#### Subscribed Topics

| Topic | Type | Description | 
| --- | --- | --- |
| `/carla/objects` | `dom::ObjectArray` | Objects in the carla environment |
| `/carla/$(role_name)/vehicle_info` | `cm::CarlaEgoVehicleInfo` | Object id of the `role_name` from list `role_names` (default: `ego_vehicle`) |
| `/carla/$(role_name)/vehicle_status` | `cm::CarlaEgoVehicleStatus` | Steering angle and acceleration of the `role_name` from list `role_names` (default: `ego_vehicle`) |
| `/carla/$(role_name)/odometry` | `nm::Odometry` | Odometry of the `role_name` from list `role_names` (default: `ego_vehicle`) |

#### Published Topics

| Topic | Type | Description |
| --- | --- | --- |
| `/carla_its_converter/object_list/carla_map` | `perception_interfaces::msg::ObjectList` | Object list in carla map frame |
| `/carla_its_converter/object_list/$(role_name)` | `perception_interfaces::msg::ObjectList` | Object list in `role_name` frame from list `role_names` (default: `ego_vehicle`) |
| `/carla_its_converter/$(role_name)/ego_data` | `perception_interfaces::msg::EgoState` | Ego State of `role_name` from list `role_names` (default: `ego_vehicle`)  |

#### Parameters

| Parameter | Type | Description |
| --- | --- | --- |
| publish_ego_vehicle | bool | Whether to publish object list in `$(role_name)` frame or not. |
| role_names | string | List of strings separated with a comma, lists all role_names which should be subscribed and published (default: `ego_vehicle`) (example: `role_names:="hero, hero1"`). |

## Usage of docker-ros Images

### Available Images

| Tag | Description |
| --- | --- |
| ` ` | latest version |

### Default Command

##### ROS1
```bash
roslaunch carla_its_converter carla_its_converter.launch
```
##### ROS2
```bash
ros2 launch carla_its_converter carla_its_converter.launch.py
```

### Launch Files

| Package | File | Path | Description |
| --- | --- | --- | --- |
| `carla_its_converter` | `carla_its_converter.launch` | `launch/` | Launches CarlaItsConverterNode for ROS1. |
| `carla_its_converter` | `carla_its_converter.launch.py` | `launch/` | Launches CarlaItsConverterNode for ROS2. |



### Additional Remarks

\-


## Official Documentation

\-
