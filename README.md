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
| `/carla/ego_vehicle/vehicle_info` | `cm::CarlaEgoVehicleInfo` | Object id of the ego vehicle |
| `/carla/ego_vehicle/vehicle_status` | `cm::CarlaEgoVehicleStatus` | Steering angle and acceleration of the ego vehicle |
| `/carla/ego_vehicle/odometry` | `nm::Odometry` | Odometry of the ego vehicle |

#### Published Topics

| Topic | Type | Description |
| --- | --- | --- |
| `/carla_its_converter/object_list/carla_map` | `perception_interfaces::msg::ObjectList` | Object list in carla map frame |
| `/carla_its_converter/object_list/ego_vehicle` | `perception_interfaces::msg::ObjectList` | Object list in ego vehicle frame |
| `/carla_its_converter/ego_data` | `perception_interfaces::msg::EgoState` | Ego State of vehicle |

#### Parameters

| Parameter | Type | Description |
| --- | --- | --- |
| publish.object_list_carla_map_frame | bool | Whether to publish object list in carla map frame or not. |
| publish.object_list_ego_vehicle_frame | bool | Whether to publish object list in ego vehicle frame or not. |
| publish.ego_data | bool | Whether to publish egodata information or not. |

## Usage of docker-ros Images

### Available Images

| Tag | Description |
| --- | --- |
| ` ` | latest version |

### Default Command

##### ROS1
```bash
roslaunch carla_its_converter carla_its_converter_ros1.launch
```
##### ROS2
```bash
ros2 launch carla_its_converter carla_its_converter_ros2.launch.xml
```

### Launch Files

| Package | File | Path | Description |
| --- | --- | --- | --- |
| `carla_its_converter` | `carla_its_converter_ros1.launch` | `launch/` | Launches CarlaItsConverterNode for ROS1. |
| `carla_its_converter` | `carla_its_converter_ros2.launch.xml` | `launch/` | Launches CarlaItsConverterNode for ROS2. |



### Additional Remarks

\-


## Official Documentation

\-
