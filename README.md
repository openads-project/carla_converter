# carla_its_interface

This package contains the CarlaItsInterfaceNode - a simple ROS Node that converts incoming messages from the [carla-ros-bridge](https://gitlab.ika.rwth-aachen.de/fb-fi/simulation/carla/ros-bridge) and publishes some of the [fb-fi defined ros messages](https://gitlab.ika.rwth-aachen.de/fb-fi/definitions) for various its-applications. Moreover the node is capable to broadcast necessary tf's for different its-applications (e.g. map->base_link).

- [Nodes](#nodes)
  - [carla_its_interface/CarlaItsInterfaceNode](#carla_its_interfacecarlaitsinterfacenode)
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
| `carla_its_interface` | `CarlaItsInterfaceNode` | Converting carla-ros-messages to fb-fi defined its-messages |

### carla_its_interface/CarlaItsInterfaceNode

#### Subscribed Topics

| Topic | Type | Description | 
| --- | --- | --- |

#### Published Topics

| Topic | Type | Description |
| --- | --- | --- |

#### Parameters

| Parameter | Type | Description |
| --- | --- | --- |

## Usage of docker-ros Images

### Available Images

| Tag | Description |
| --- | --- |
| ` ` | latest version |

### Default Command

```bash
roslaunch carla_its_interface start_CarlaItsInterfaceNode.launch 
```

### Launch Files

| Package | File | Path | Description |
| --- | --- | --- | --- |


### Configuration Files

| Package | File | Path | Description |
| --- | --- | --- | --- |


### Additional Remarks

\-


## Official Documentation

\-
