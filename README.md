# Digiquartz Pressure Sensor Driver

- [Digiquartz Pressure Sensor Driver](#digiquartz-pressure-sensor-driver)
  - [1. Overview](#1-overview)
  - [2. License](#2-license)
  - [3. Installation](#3-installation)
      - [3.1 Building from Source](#31-building-from-source)
      - [3.2 Building from autoproj (recommended)](#32-building-from-autoproj-recommended)
      - [3.3 Protocol Update](#33-protocol-update)
  - [4. How to Use](#4-how-to-use)
  - [Bugs & Feature Requests](#bugs--feature-requests)

---

## 1. Overview

The `drivers-pressure_digiquartz(pressure_pkg)` is a ROS package designed for interfacing with the Digiquartz pressure sensor. It provides functionalities to configure the sensor, read pressure data, and publish it to ROS topics. This package is essential for applications requiring precise pressure measurements and integrates seamlessly with ROS 2.

```
📦
 ┣ 📂 config                      # Config files for robot and controllers description
 ┣ 📂 include                     # Executables headers
 ┣ 📂 launch                      # Launch files for control modes and robots
 ┣ 📂 src                         # Executables
 ┣...
 ```

## 2. License

- The source code is released under an [Apache License 2.0](LICENSE).

**Authors:**

- João Pedro Almeida (<joaop.almeida@fbter.org.br>)

**Affiliation:** SENAI CIMATEC

**Maintainers:**

- João Pedro Almeida (<joaop.almeida@fbter.org.br>)

The `drivers-pressure_digiquartz` package has been tested under ROS 2 Humble and Ubuntu 22.04. This is research code; expect frequent changes and disclaim any fitness for a particular purpose.

| OS | ROS |
| :---: | :---: |
| Ubuntu 22.04 | ROS 2 Humble |

## 3. Installation

### 3.1 Building from Source

* Install [ROS 2 Humble](https://docs.ros.org/en/humble/index.html) on Ubuntu 22.04.

* To build from source, clone the latest version from this repository into your colcon workspace and compile the package using:

   ```sh
   cd YOUR_WORKSPACE/src
   git clone https://github.com/Brazilian-Institute-of-Robotics/drivers-pressure_digiquartz.git
   cd YOUR_WORKSPACE
   source /opt/ros/${ROS_DISTRO}/setup.bash
   colcon build --packages-select pressure_pkg
   ```

### 3.2 Building from autoproj (recommended)

- Follow the [build configuration](https://github.com/Brazilian-Institute-of-Robotics/bir.softrobots-buildconf) based on autoproj.

- After installation and configuration of autoproj, run the following commands:

   ```sh
   cd YOUR_WORKSPACE
   source env.sh
   amake pressure_pkg
   source install/setup.bash
   ```

- Include the `drivers-pressure_digiquartz` in the file `autoproj/manifest` file if it does not exist.

   ```yaml
   package_sets:
     - github: Brazilian-Institute-of-Robotics/bir.flatfish_ros-package_set.git
     private: true

   layout:
     - drivers-pressure_digiquartz
   ```
- _overrides.yaml_
  ```yaml
   overrides:
     - Brazilian-Institute-of-Robotics/drivers-pressure_digiquartz:
       branch: port-to-ros2
  ```
 
- Update and build the autoproj:

   ```bash
   aup
   amake 
   ```

- Run `autoproj doc package` in your terminal to generate Doxygen documentation:

   ```sh
   autoproj doc pressure_pkg
   ```

### 3.3 Protocol Update

To use the Digiquartz pressure sensor with this package, ensure the sensor is properly configured and set up. Please refer to the documentation specific to your sensor model for protocol details.

:warning: **Give permission to your system to access the communication port**

## 4. How to use

   ```sh
   ros2 run pressure_pkg main 
   ```

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/Brazilian-Institute-of-Robotics/drivers-pressure_digiquartz/issues).