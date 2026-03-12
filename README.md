# carla_its_converter

This package contains the CarlaItsConverterNode - a ROS 2 Node that converts incoming messages from the [carla-ros-bridge](https://gitlab.ika.rwth-aachen.de/fb-fi/simulation/carla/carla-ros-bridge) and publishes some of the [fb-fi defined ros interfaces](https://github.com/ika-rwth-aachen#interfaces--v2x-communication) for various its-applications.

- [carla\_its\_converter](#carla_its_converter)
  - [Container Images](#container-images)
  - [`carla_its_converter`](#carla_its_converter-1)
    - [Subscribed Topics](#subscribed-topics)
    - [Published Topics](#published-topics)
    - [Parameters](#parameters)


## Container Images

| Description | Image:Tag | Default Command |
| --- | --- | --- |
| ROS 2 Node to convert carla to fb-fi ros messages | `gitlab.ika.rwth-aachen.de:5050/fb-fi/simulation/carla/carla_its_converter:latest` | `ros2 launch carla_its_converter carla_its_converter.launch.py` |

## `carla_its_converter`

### Subscribed Topics

| Topic | Type | Description |
| --- | --- | --- |
| `/carla/objects` | `derived_object_msgs/msg/ObjectArray` | Objects in the carla environment |
| `/carla/<actor_name>/vehicle_info` | `carla_msgs/msg/CarlaEgoVehicleInfo` | Object id of the `actor_name` from list `ego_data_actors` (default: `ego_vehicle`) |
| `/carla/<actor_name>/vehicle_status` | `carla_msgs/msg/CarlaEgoVehicleStatus` | Steering angle of the `actor_name` from list `ego_data_actors` (default: `ego_vehicle`) |
| `/carla/<actor_name>/odometry` | `nav_msgs/msg/Odometry` | Odometry of the `actor_name` from list `ego_data_actors` (default: `ego_vehicle`) |
| `/carla/<actor_name>/gnss` | `sensor_msgs/msg/NavSatFix` | GNSS of the `actor_name` from list `ego_data_actors` (default: `ego_vehicle`) |
| `/carla/<actor_name>/imu` | `sensor_msgs/msg/Imu` | IMU of the `actor_name` from list `ego_data_actors` (default: `ego_vehicle`) |
| `/carla/traffic_lights/info` | `carla_msgs/msg/CarlaTrafficLightInfoList` | Global traffic light information |
| `/carla/traffic_lights/status` | `carla_msgs/msg/CarlaTrafficLightStatusList` | Global traffic light status |
| `/carla/world_info` | `carla_msgs/msg/CarlaWorldInfo` | CARLA world/map info |
| `/carla/.*` | `derived_object_msgs/msg/ObjectArray` | All custom topics of this type are subscribed and converted automatically |

### Published Topics

| Topic | Type | Description |
| --- | --- | --- |
| `/carla_its_converter/object_list` | `perception_msgs/msg/ObjectList` | Object list in carla map frame |
| `/carla_its_converter/<actor_name>/object_list` | `perception_msgs/msg/ObjectList` | Object list in `actor_name` frame, for each actor in `object_data_actors` |
| `/carla_its_converter/<actor_name>/ego_data` | `perception_msgs/msg/EgoData` | Ego state of `actor_name`, for each actor in `ego_data_actors` |
| `/carla_its_converter/<actor_name>/etsi_cam` | `etsi_its_cam_msgs/msg/CAM` | Ego state converted to CAM (requires simulation time to be current unix time) |
| `/carla_its_converter/traffic_lights` | `perception_msgs/msg/ObjectList` | Traffic lights with pose and current signal status |
| `/carla_its_converter/map_info` | `std_msgs/msg/String` | CARLA map name |
| `/carla_its_converter/.*` | `perception_msgs/msg/ObjectList` | Converted output for each auto-subscribed custom topic |

### Parameters

| Parameter | Type | Default | Description |
| --- | --- | --- | --- |
| `ego_data_actors` | `string` | `"ego_vehicle"` | Comma-separated list of actor names to publish ego data for (e.g. `"hero, hero1"`). `EgoData` and `CAM` are published for each actor. |
| `object_data_actors` | `string` | `"ego_vehicle"` | Comma-separated list of actor names to publish object lists for. The global `ObjectList` is transformed into each actor's frame. |
| `pos_variances` | `double` | `0.2` | Position covariance value (`-1.0` = invalid/unset) |
| `vel_variances` | `double` | `-1.0` | Velocity covariance value (`-1.0` = invalid/unset) |
| `acc_variances` | `double` | `-1.0` | Acceleration covariance value (`-1.0` = invalid/unset) |
| `angle_variances` | `double` | `0.1` | Angle covariance value (`-1.0` = invalid/unset) |
| `angle_rate_variances` | `double` | `-1.0` | Angle rate covariance value (`-1.0` = invalid/unset) |
| `traffic_light_frequency` | `double` | `10.0` | Publishing frequency for traffic lights in Hz |
| `carla_fixed_frame_id` | `string` | `"carla_map"` | Fixed frame ID used for the CARLA map |
| `acceleration_filter_alpha` | `double` | `1.0` | Low-pass filter alpha for IMU acceleration: `filtered = alpha * new + (1 - alpha) * previous`. `1.0` disables filtering. |
