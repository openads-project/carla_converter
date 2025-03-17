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
| `/carla/$(actor_name)/vehicle_info` | `cm::CarlaEgoVehicleInfo` | Object id of the `actor_name` from list `ego_data_actors` (default: `ego_vehicle`) |
| `/carla/$(actor_name)/vehicle_status` | `cm::CarlaEgoVehicleStatus` | Steering angle and acceleration of the `actor_name` from list `ego_data_actors` (default: `ego_vehicle`) |
| `/carla/$(actor_name)/odometry` | `nm::Odometry` | Odometry of the `actor_name` from list `ego_data_actors` (default: `ego_vehicle`) |
| `/carla/.*` | `dom::ObjectArray` | All custom topics of type `dom::ObjectArray` are subscribed and converted automatically |
| `/carla/traffic_lights/info` | `cm::CarlaTrafficLightInfoList` | Global geometric traffic light information |
| `/carla/traffic_lights/status` | `cm::CarlaTrafficLightStatusList` | Global traffic light signal status |

#### Published Topics

| Topic | Type | Description |
| --- | --- | --- |
| `/carla_its_converter/object_list` | `perception_msgs::msg::ObjectList` | Object list in carla map frame |
| `/carla_its_converter/$(actor_name)/object_list` | `perception_msgs::msg::ObjectList` | Object list in `actor_name` frame from list `ego_data_actors` (default: `ego_vehicle`) |
| `/carla_its_converter/.*` | `perception_msgs::msg::ObjectList` | All custom topics of type `dom::ObjectArray` are subscribed and converted automatically |
| `/carla_its_converter/traffic_lights` | `perception_msgs::msg::ObjectList` | Gloal traffic light information of it's pose and current signal status |
| `/carla_its_converter/$(actor_name)/ego_data` | `perception_msgs::msg::EgoState` | Ego State of `actor_name` from list `ego_data_actors` (default: `ego_vehicle`)  |
| `/carla_its_converter/$(actor_name)/etsi_cam` | `etsi_its_cam_msgs::msg::CAM` | Ego State of `actor_name` is converted to CAM if simulation time is current unix time. |

#### Parameters

| Parameter | Type | Description |
| --- | --- | --- |
| ego_data_actors | string | List of actors which should be used to generate ego information. `EgoData` is published for every actor. Additionaly, `CAM` messages are published if simulation time is current unix time. (default: `ego_vehicle`) (example: `ego_data_actors:="hero, hero1"`). |
| object_data_actors | string | List of actors which should be used to generate object information.
The global `ObjectList` is converted and published within the actors frame. Same applies to potential ideal `ObjectList`. (default: `ego_vehicle`) (example: `object_data_actors:="hero, hero1"`). |

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
| `carla_its_converter` | `carla_its_converter.launch.py` | `launch/` | Launches CarlaItsConverterNode |



### Additional Remarks

\-


## Official Documentation

\-
