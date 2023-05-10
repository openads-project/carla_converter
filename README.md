# carla_its_converter

This package contains the CarlaItsConverterNode - a simple ROS Node that converts incoming messages from the [carla-ros-bridge](https://gitlab.ika.rwth-aachen.de/fb-fi/simulation/carla/ros-bridge) and publishes some of the [fb-fi defined ros messages](https://gitlab.ika.rwth-aachen.de/fb-fi/definitions) for various its-applications. Moreover the node is capable to broadcast necessary tf's for different its-applications (e.g. carla_map->map).

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
| `/carla/ego_vehicle/objects` | `dom::ObjectArray` | Objects in the carla environment |
| `/carla/ego_vehicle/odometry` | `nam::Odometry` | Odometry of the ego vehicle |

#### Published Topics

| Topic | Type | Description |
| --- | --- | --- |
| `/carla_its_converter/objectList/carla_map` | `pin::ObjectList` | Object list in carla map frame |
| `/carla_its_converter/objectList/ego_vehicle` | `pin::ObjectList` | Object list in ego vehicle frame |
| `/carla_its_converter/objectList/map` | `pin::ObjectList` | Object list in map frame |
| `/carla_its_converter/objectList/base_link` | `pin::ObjectList` | Object list in base link frame |

#### Parameters

| Parameter | Type | Description |
| --- | --- | --- |
| publish.carla_map | bool | Whether to publish object list in carla map frame or not. |
| publish.ego_vehicle | bool | Whether to publish object list in ego vehicle frame or not. |
| publish.map | bool | Whether to publish object list in map frame or not. |
| publish.base_link | bool | Whether to publish object list in base link frame or not. |
| publish.egoData | bool | Whether to publish TODO or not. |
| fov_range | float | Maximum field of view range for objects in the base link frame. Only objects within the FOV will be published. |
| center_to_baselink | float | Distance between center of the vehicle and its base link. |

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
ros2 launch carla_its_converter carla_its_converter_ros2.launch
```

### Launch Files

| Package | File | Path | Description |
| --- | --- | --- | --- |
| `carla_its_converter` | `carla_its_converter_ros1.launch` | `launch/` | Launches CarlaItsConverterNode for ROS1. |
| `carla_its_converter` | `carla_its_converter_ros2.launch` | `launch/` | Launches CarlaItsConverterNode for ROS2. |


### Configuration Files

| Package | File | Path | Description |
| --- | --- | --- | --- |
| `carla_its_converter` | `carla_its_converter_params.yaml` | `config/` | ROS parameters for the CarlaItsConverterNode |

### Additional Remarks

\-


## Official Documentation

\-
