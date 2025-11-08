# Experiments Package Setup - Summary

## What Was Created

I've successfully created the `experiments` package with complete configuration files and launch files for your three experiment types. This package is designed to work with your tracing infrastructure and ROS2 component containers.

## Package Structure

```
packages/src/experiments/
├── CMakeLists.txt                          # Build configuration
├── package.xml                             # Package metadata
├── README.md                               # Complete documentation
├── config/                                 # Configuration files
│   ├── monte_carlo/
│   │   ├── default_server.yaml            # Monte Carlo server config
│   │   └── default_client.yaml            # Monte Carlo client config
│   ├── yolo/
│   │   ├── default_server.yaml            # YOLO server config
│   │   └── default_client.yaml            # YOLO client config
│   └── interference/
│       ├── default_server.yaml            # Monte Carlo server config
│       ├── default_client.yaml            # Monte Carlo client config
│       └── default_interference.yaml      # Interference timer config
└── launch/
    ├── monte_carlo.launch.py              # Monte Carlo experiment launcher
    ├── yolo.launch.py                     # YOLO experiment launcher
    └── interference.launch.py             # Interference experiment launcher
```

## Configuration Files Details

### Monte Carlo Configs
- **default_server.yaml**: Proactive mode, multi-threading enabled, batch_size=1
- **default_client.yaml**: 1000ms goal timer, 250ms cancel timeout

### YOLO Configs
- **default_server.yaml**: Proactive mode, multi-threading enabled, batch_size=1, sync mode
- **default_client.yaml**: video_frames topic, cancel after 12 layers, score-based cancellation disabled

### Interference Configs
- **default_server.yaml**: Same as Monte Carlo server
- **default_client.yaml**: Same as Monte Carlo client
- **default_interference.yaml**: 100ms timer period, 10ms execution time

## Launch Files Features

All launch files support:
- ✅ Component containers (single or multi-threaded executors)
- ✅ Custom configuration file paths
- ✅ Configurable log levels
- ✅ Container naming
- ✅ Clear console output with experiment info

### 1. Monte Carlo Launch (`monte_carlo.launch.py`)
- Launches Monte Carlo server and client in one component container
- Default: multi-threaded executor

### 2. YOLO Launch (`yolo.launch.py`)
- Launches YOLO server and client in one component container
- Launches video_publisher as a separate node
- Supports custom image path
- Default: multi-threaded executor
- Uses intra-process communication for efficiency

### 3. Interference Launch (`interference.launch.py`)
- Launches Monte Carlo server, client, and interference timer in one component container
- Configurable interference timer parameters (period and execution time)
- Default: multi-threaded executor

## Usage Examples

### Quick Start

```bash
# Build the package
cd /home/vscode/workspace/packages
colcon build --packages-select experiments --symlink-install
source install/setup.bash

# Run Monte Carlo experiment
ros2 launch experiments monte_carlo.launch.py

# Run YOLO experiment
ros2 launch experiments yolo.launch.py

# Run Interference experiment
ros2 launch experiments interference.launch.py
```

### With Custom Configurations

```bash
# Monte Carlo with custom batch size
ros2 launch experiments monte_carlo.launch.py \
    server_config:=/path/to/custom_server.yaml

# YOLO with debug logging
ros2 launch experiments yolo.launch.py \
    log_level:=debug

# Interference with custom timing
ros2 launch experiments interference.launch.py \
    timer_period_ms:=50 \
    execution_time_ms:=5
```

### With LTTng Tracing

```bash
# Start tracing
lttng create my_exp
lttng enable-event --userspace 'anytime:*'
lttng start

# Run experiment
ros2 launch experiments monte_carlo.launch.py

# Stop and view trace
lttng stop
babeltrace ~/.lttng-traces/my_exp-*
lttng destroy my_exp
```

## Key Features

1. **Component-based**: All experiments use ROS2 component containers for efficient execution
2. **Tracing-ready**: Works seamlessly with your LTTng tracing infrastructure
3. **Configurable**: All parameters can be overridden via launch arguments or config files
4. **Consistent**: All three experiments follow the same launch file pattern
5. **Default configs**: Sensible defaults that match your previous experiment setups
6. **Documentation**: Complete README with usage examples

## Integration with Evaluation Scripts

These launch files are designed to work with your evaluation scripts:
- `evaluation_monte_carlo.sh`
- `evaluation_yolo.sh`
- `evaluation_yolo_score.sh`

You can modify these scripts to use the new launch files from the experiments package instead of the individual package launch files.

## Next Steps

The experiments package is now ready. You mentioned you'll provide next instructions. Some potential next steps could be:

1. Update evaluation scripts to use the experiments package launch files
2. Create additional configuration variants for different batch sizes
3. Add scripts to automate running multiple configurations
4. Integrate with your plotting scripts
5. Add more experiment types

Let me know how you'd like to proceed!
