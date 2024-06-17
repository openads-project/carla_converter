# carla_its_converter

This package contains the CarlaItsConverterNode - a simple ROS Node that converts incoming messages from the [carla-ros-bridge](https://gitlab.ika.rwth-aachen.de/fb-fi/simulation/carla/carla-ros-bridge) and publishes some of the [fb-fi defined ros messages](https://gitlab.ika.rwth-aachen.de/fb-fi/definitions) for various its-applications.

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
| `/carla/$(role_name)/vehicle_info` | `cm::CarlaEgoVehicleInfo` | Object id of the `role_name` from list `ego_data_actors` (default: `ego_vehicle`) |
| `/carla/$(role_name)/vehicle_status` | `cm::CarlaEgoVehicleStatus` | Steering angle and acceleration of the `role_name` from list `ego_data_actors` (default: `ego_vehicle`) |
| `/carla/$(role_name)/odometry` | `nm::Odometry` | Odometry of the `role_name` from list `ego_data_actors` (default: `ego_vehicle`) |

#### Published Topics

| Topic | Type | Description |
| --- | --- | --- |
| `/carla_its_converter/objects` | `perception_msgs::msg::ObjectList` | Object list in carla map frame |
| `/carla_its_converter/$(role_name)/objects` | `perception_msgs::msg::ObjectList` | Object list in `role_name` frame from list `ego_data_actors` (default: `ego_vehicle`) |
| `/carla_its_converter/$(role_name)/ego_data` | `perception_msgs::msg::EgoState` | Ego State of `role_name` from list `ego_data_actors` (default: `ego_vehicle`)  |

#### Parameters

| Parameter | Type | Description |
| --- | --- | --- |
| ego_data_actors | string | List of strings separated with a comma, lists all ego_data_actors which should be subscribed and published (default: `ego_vehicle`) (example: `ego_data_actors:="hero, hero1"`). |

## Usage of docker-ros Images

### Available Images

| Tag | Description |
| --- | --- |
| ` ` | latest version |

### Default Command

```bash
ros2 launch carla_its_converter carla_its_converter.launch.py
```

### Launch Files

| Package | File | Path | Description |
| --- | --- | --- | --- |
| `carla_its_converter` | `carla_its_converter.launch.py` | `launch/` | Launches CarlaItsConverterNode for ROS2. |



### Additional Remarks

\-


## Official Documentation

\-
