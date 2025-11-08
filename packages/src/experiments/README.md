# Experiments Package

This package contains experimental configurations and launch files for testing anytime algorithms with the tracing infrastructure.

## Overview

The experiments package provides three types of experiments:
1. **Monte Carlo** - Tests Monte Carlo algorithm with server and client
2. **YOLO** - Tests YOLO algorithm with server, client, and video publisher
3. **Interference** - Tests Monte Carlo with interference timer node

All experiments use ROS2 component containers for efficient execution and support both single-threaded and multi-threaded executors.

## Structure

```
experiments/
├── config/
│   ├── monte_carlo/
│   │   ├── default_server.yaml
│   │   └── default_client.yaml
│   ├── yolo/
│   │   ├── default_server.yaml
│   │   └── default_client.yaml
│   └── interference/
│       ├── default_server.yaml
│       ├── default_client.yaml
│       └── default_interference.yaml
└── launch/
    ├── monte_carlo.launch.py
    ├── yolo.launch.py
    └── interference.launch.py
```

## Usage

### Building the Package

```bash
cd /home/vscode/workspace/packages
colcon build --packages-select experiments --symlink-install
source install/setup.bash
```

### 1. Monte Carlo Experiment

Runs Monte Carlo server and client in a component container.

**Default configuration:**
```bash
ros2 launch experiments monte_carlo.launch.py
```

**With custom parameters:**
```bash
ros2 launch experiments monte_carlo.launch.py \
    use_multi_threaded:=true \
    server_config:=/path/to/custom_server.yaml \
    client_config:=/path/to/custom_client.yaml \
    log_level:=debug
```

**Available arguments:**
- `use_multi_threaded` (default: `true`) - Use multi-threaded executor
- `server_config` - Path to server configuration YAML
- `client_config` - Path to client configuration YAML
- `container_name` (default: `monte_carlo_experiment_container`)
- `log_level` (default: `info`)

### 2. YOLO Experiment

Runs YOLO server and client in a component container, plus video publisher.

**Default configuration:**
```bash
ros2 launch experiments yolo.launch.py
```

**With custom parameters:**
```bash
ros2 launch experiments yolo.launch.py \
    use_multi_threaded:=true \
    server_config:=/path/to/custom_server.yaml \
    client_config:=/path/to/custom_client.yaml \
    image_path:=/path/to/images \
    log_level:=debug
```

**Available arguments:**
- `use_multi_threaded` (default: `true`) - Use multi-threaded executor
- `server_config` - Path to server configuration YAML
- `client_config` - Path to client configuration YAML
- `image_path` - Path to image directory for video publisher
- `container_name` (default: `yolo_experiment_container`)
- `log_level` (default: `info`)

### 3. Interference Experiment

Runs Monte Carlo server, client, and interference timer in a component container.

**Default configuration:**
```bash
ros2 launch experiments interference.launch.py
```

**With custom parameters:**
```bash
ros2 launch experiments interference.launch.py \
    use_multi_threaded:=true \
    server_config:=/path/to/custom_server.yaml \
    client_config:=/path/to/custom_client.yaml \
    timer_period_ms:=100 \
    execution_time_ms:=10 \
    log_level:=debug
```

**Available arguments:**
- `use_multi_threaded` (default: `true`) - Use multi-threaded executor
- `server_config` - Path to server configuration YAML
- `client_config` - Path to client configuration YAML
- `interference_config` - Path to interference timer configuration YAML
- `timer_period_ms` (default: `100`) - Interference timer period in ms
- `execution_time_ms` (default: `10`) - Interference busy-wait duration in ms
- `container_name` (default: `interference_experiment_container`)
- `log_level` (default: `info`)

## Configuration Files

### Monte Carlo Default Configuration

**Server (`config/monte_carlo/default_server.yaml`):**
- Mode: `proactive`
- Multi-threading: `true`
- Batch size: `1`
- Log level: `info`

**Client (`config/monte_carlo/default_client.yaml`):**
- Goal timer period: `1000 ms`
- Cancel timeout: `250 ms`
- Log level: `info`

### YOLO Default Configuration

**Server (`config/yolo/default_server.yaml`):**
- Mode: `proactive`
- Multi-threading: `true`
- Batch size: `1`
- Sync mode: `sync`
- Log level: `info`

**Client (`config/yolo/default_client.yaml`):**
- Image topic: `video_frames`
- Cancel after layers: `12`
- Cancel layer score: `false`
- Log level: `info`

### Interference Default Configuration

Same as Monte Carlo plus:

**Interference Timer (`config/interference/default_interference.yaml`):**
- Timer period: `100 ms`
- Execution time: `10 ms`
- Log level: `info`

## Using with LTTng Tracing

All experiments support LTTng tracing. To capture traces:

```bash
# Create tracing session
lttng create my_experiment

# Enable anytime tracepoints
lttng enable-event --userspace 'anytime:*'

# Start tracing
lttng start

# Run experiment
ros2 launch experiments monte_carlo.launch.py

# Stop tracing
lttng stop

# View traces
babeltrace ~/.lttng-traces/my_experiment-*

# Destroy session
lttng destroy my_experiment
```

## Creating Custom Configurations

To create custom configurations for your experiments:

1. Copy a default config file:
   ```bash
   cp config/monte_carlo/default_server.yaml config/monte_carlo/my_config.yaml
   ```

2. Edit the parameters in your new config file

3. Launch with your custom config:
   ```bash
   ros2 launch experiments monte_carlo.launch.py \
       server_config:=/path/to/your/my_config.yaml
   ```

## Integration with Evaluation Scripts

These launch files are designed to work with the evaluation scripts (`evaluation_monte_carlo.sh`, `evaluation_yolo.sh`, `evaluation_yolo_score.sh`) which can:
- Generate multiple configuration files
- Run experiments with different parameters
- Capture LTTng traces
- Plot results

See the evaluation scripts in the workspace root for examples.
