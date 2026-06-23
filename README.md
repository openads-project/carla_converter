# carla_converter

<p align="center">
  <a href="https://github.com/openads-project"><img src="https://img.shields.io/badge/OpenADS-f5ff01"/></a>
  <a href="https://www.ros.org"><img src="https://img.shields.io/badge/ROS 2-jazzy-22314e"/></a>
  <a href="https://github.com/openads-project/carla_converter/releases/latest"><img src="https://img.shields.io/github/v/release/openads-project/carla_converter"/></a>
  <a href="https://github.com/openads-project/carla_converter/blob/main/LICENSE"><img src="https://img.shields.io/github/license/openads-project/carla_converter"/></a>
  <br>
  <a href="https://github.com/openads-project/carla_converter/actions/workflows/docker-ros.yml"><img src="https://github.com/openads-project/carla_converter/actions/workflows/docker-ros.yml/badge.svg"/></a>
  <a href="https://github.com/openads-project/carla_converter/actions/workflows/compose-oci.yml"><img src="https://github.com/openads-project/carla_converter/actions/workflows/compose-oci.yml/badge.svg"/></a>
  <a href="https://openads-project.github.io/carla_converter"><img src="https://github.com/openads-project/carla_converter/actions/workflows/docs.yml/badge.svg"/></a>
  <a href="https://github.com/openads-project/carla_converter/actions/workflows/consistency.yml"><img src="https://github.com/openads-project/carla_converter/actions/workflows/consistency.yml/badge.svg"/></a>
</p>

**Converter for CARLA specific ROS 2 data to OpenADS perception_interfaces**

The [carla_converter](carla_converter/README.md) node bridges CARLA and OpenADS ROS 2 interfaces by converting CARLA actor, object, traffic light, and world information into OpenADS-compatible messages. Key features:

- **Object conversion**: converts CARLA object arrays into OpenADS object list outputs.
- **Per-actor ego outputs**: publishes ego data from odometry, vehicle status, vehicle info, GNSS, and IMU inputs.
- **Traffic light support**: optionally converts CARLA traffic light info and status streams into traffic light object lists.
- **Configurable integration**: exposes launch arguments and parameters for topic remapping, actor selection, covariance values, fixed frames, namespace, node name, and log level.

<p align="center">
  <strong>🚀 <a href="#-quick-start">Quick Start</a></strong> • <strong>💻 <a href="#-development">Development</a></strong> • <strong>📝 <a href="#-documentation">Documentation</a></strong>
</p>


> [!IMPORTANT]
> This repository is part of [***OpenADS***](https://github.com/openads-project), the *Open Automated Driving Systems* project. *OpenADS* and its modules have been initiated and are currently being maintained by the [**Institute for Automotive Engineering (ika) at RWTH Aachen University**](https://www.ika.rwth-aachen.de/de/).


## 🚀 Quick Start

1. Start a container of the pre-built runtime image.
    ```bash
    docker run --rm -it ghcr.io/openads-project/carla_converter:latest bash
    ```
1. Inside the container, launch the pre-built nodes.
    ```bash
    ros2 launch carla_converter carla_converter.launch.py
    ```

<!-- Optional replacement quick start with repo-specific demo (Docker Compose)

1. Launch a container of the pre-built runtime image in the provided demo [Docker Compose](demo/docker-compose.yml) setup.
    ```bash
    cd demo
    xhost +local: # allow GUI forwarding from containers
    docker compose up
    ```
1. Observe ...
1. Stop the demo and clean up.
    > *Ctrl+C*
    ```bash
    docker compose down
    xhost -local: # revoke GUI forwarding permissions
    ```
-->

## 💻 Development

### Set up Development Environment

1. Clone the repository.
    ```bash
    git clone https://github.com/openads-project/carla_converter.git
    ```
1. Initialize the [`.openads-dev-environment`](https://github.com/openads-project/openads-dev-environment) submodule containing development environment configuration.
    ```bash
    cd carla_converter
    git submodule update --init --recursive
    ```
1. Open the repository in [Visual Studio Code](https://code.visualstudio.com).
    ```bash
    code .
    ```
1. Install the recommended VS Code extensions.
    > *Ctrl+Shift+P / Extensions: Show Recommended Extensions / Install Workspace Recommended Extensions (Cloud Download Icon)*
1. Reopen the repository in a [Dev Container](https://code.visualstudio.com/docs/devcontainers/containers).
    > *Ctrl+Shift+P / Dev Containers: Rebuild and Reopen in Container*

### Build

> *Ctrl+Shift+B*

```bash
colcon build
```

### Run Tests

> *Ctrl+Shift+P / Tasks: Run Test Task*

```bash
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=1
colcon test
colcon test-result --verbose
```


## 📝 Documentation

Package and node interfaces are documented in the respective package READMEs listed below. Implementation details are found in the [Source Code Documentation](https://openads-project.github.io/carla_converter).

| Package | Description |
| --- | --- |
| [carla_converter](carla_converter/README.md) | Converter for CARLA specific ROS 2 data to OpenADS perception_interfaces. |

## ⚖️ Licensing

The source code in this repository is licensed under Apache-2.0, see [LICENSE](LICENSE). Container images provided by this repository may contain third-party software shipped with their own license terms.

## 🙏 Acknowledgements

Development and maintenance of this repository are supported by the following projects. We acknowledge the funding of the respective institutions.

| Project | Funding Institution | Grant Number |
| --- | --- | --- |
| [AIGGREGATE](https://aiggregate.eu/) | 🇪🇺 European Union | 101202457 |
| [AIthena](https://aithena.eu/) | 🇪🇺 European Union | 101076754 |
| [autotech.agil](https://www.autotechagil.de/) | 🇩🇪 Federal Ministry for Research, Technology and Space (BMFTR) | 01IS22088A |

<p>
  <img src="https://www.drought.uni-freiburg.de/stressres/images/bmftr-logo/image" height=70>
  <img src="https://ec.europa.eu/regional_policy/images/information-sources/logo-download-center/eu_funded_en.jpg" height=70>
</p>

<sub><sup>Funded by the European Union. Views and opinions expressed are however those of the author(s) only and do not necessarily reflect those of the European Union or the European Climate, Infrastructure and Environment Executive Agency (CINEA). Neither the European Union nor CINEA can be held responsible for them.</sup></sub>
