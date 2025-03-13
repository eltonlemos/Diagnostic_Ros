# Diagnostic Moog

## Overview

The **Diagnostic Moog** package is a ROS 2-based diagnostic monitoring system for checking the health and status of various topics. It monitors message frequencies, detects potential issues, and publishes diagnostic messages.

This package utilizes the `diagnostic_aggregator` and `diagnostic_msgs` to provide real-time system diagnostics.

## Features

- Monitors specified ROS 2 topics for message frequency.
- Detects abnormal message rates and categorizes issues as **OK**, **Warning**, or **Error**.
- Uses **diagnostic_aggregator** for structured diagnostics.
- Supports configuration via **YAML** files for topics and analyzers.
- Provides a launch file for easy startup.

## Installation

### Prerequisites

Ensure you have ROS 2 installed. This package was developed and tested on **ROS 2 (Foxy, Galactic, Humble, or later).**

### Building the Package

```bash
# Navigate to your ROS 2 workspace
cd ~/ros2_ws/src

# Clone the repository
git clone <repository_url>

# Navigate back to the workspace
cd ~/ros2_ws

# Build the package
colcon build --packages-select diagnostic_moog
```

### Sourcing the Workspace

After building, source the workspace:

```bash
source install/setup.bash
```

## Usage

### Launching the Diagnostic System

Use the provided launch file to start the diagnostics system:

```bash
ros2 launch diagnostic_moog diagnostics.launch.py
```

### Running the Diagnostic Script Manually

```bash
ros2 run diagnostic_moog diagnostic.py --config config/topics.yaml
```

### Configuration

#### `config/topics.yaml`
Defines the topics to be monitored along with their **error** and **warning** thresholds.

```yaml
topics: ["/navsatfix", "/livox/lidar", "/diagnostics"]
error_rates: [100, 10, 1000]
warning_rates: [150, 15, 150]
```

#### `config/analyzers.yaml`
Defines diagnostic analyzers for various sensor topics.

```yaml
analyzers:
  ros__parameters:
    path: Aggregation
    lidar:
      type: diagnostic_aggregator/GenericAnalyzer
      path: Lidar
      contains: [ '/lidar' ]
    ouster:
      type: diagnostic_aggregator/GenericAnalyzer
      path: Ouster
      startswith: [ '/ouster' ]
    topology:
      type: 'diagnostic_aggregator/AnalyzerGroup'
      path: Topology
      analyzers:
        livox:
          type: diagnostic_aggregator/GenericAnalyzer
          path: Livox
          contains: [ '/livox' ]
```

## How It Works

1. **Topic Monitoring**: The `diagnostic.py` script subscribes to the topics listed in `topics.yaml`.
2. **Rate Analysis**: The script calculates the frequency of messages and compares them against predefined thresholds.
3. **Diagnostic Messages**: It publishes the status of each topic to `/diagnostics`, using `diagnostic_msgs.msg.DiagnosticArray`.
4. **Analyzer Aggregation**: The `diagnostic_aggregator` processes the diagnostic messages and provides a structured view of system health.

## Development

### Modifying the Source Code

- **Main script**: [`diagnostic.py`](src/diagnostic_moog/diagnostic.py)
- **Launch file**: [`diagnostics.launch.py`](src/diagnostic_moog/diagnostics.launch.py)
- **Configuration files**:
  - [`topics.yaml`](config/topics.yaml)
  - [`analyzers.yaml`](config/analyzers.yaml)


## License

This project is distributed under the BSD License.

## Contributors

- [EltonLemos](https://github.com/eltonlemos)

