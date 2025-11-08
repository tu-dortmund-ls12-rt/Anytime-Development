# Anytime Tracing Package

This package provides custom LTTng tracepoints for the anytime system.

## Overview

The `anytime_tracing` package adds custom instrumentation to anytime system components, allowing detailed performance analysis and debugging using LTTng (Linux Trace Toolkit Next Generation).

## Custom Tracepoints

### `anytime:interference_timer_init`

Triggered when an InterferenceTimerNode is initialized.

**Fields:**
- `node_handle` (void*): Pointer to the node handle
- `timer_period_ms` (int): Timer period in milliseconds  
- `execution_time_ms` (int): Busy-wait execution time in milliseconds
- `version` (string): Package version

**Location:** `interference/src/interference_timer_node.cpp` (constructor)

## Usage

### 1. Build the Package

```bash
cd /home/vscode/workspace/packages
colcon build --packages-select anytime_tracing interference
source install/setup.bash
```

### 2. Enable Tracing

There are two ways to use custom tracepoints:

#### Method A: Manual LTTng Commands

```bash
# Create a tracing session
lttng create my_session

# Enable anytime custom tracepoints
lttng enable-event --userspace 'anytime:*'

# Optionally enable ROS2 tracepoints for comparison
lttng enable-event --userspace 'ros2:rcl_node_init'
lttng enable-event --userspace 'ros2:rcl_timer_init'

# Add context information
lttng add-context --userspace --type=vpid
lttng add-context --userspace --type=vtid
lttng add-context --userspace --type=procname

# Start tracing
lttng start

# Run your application
ros2 run interference interference_timer_node

# Stop tracing
lttng stop

# View results
lttng view | grep anytime

# Cleanup
lttng destroy
```

#### Method B: Using the Test Script

A test script is provided that automates the tracing process:

```bash
cd /home/vscode/workspace
./test_anytime_tracepoint.sh
```

### 3. Viewing Traces

The trace output will show entries like:

```
[timestamp] anytime:interference_timer_init: { 
  node_handle = 0x..., 
  timer_period_ms = 100, 
  execution_time_ms = 10, 
  version = "1.0.0" 
}
```

## Integration with ros2 trace

The `ros2 trace --list` command shows hardcoded ROS2 tracepoints from the `tracetools_trace` package. Custom anytime tracepoints use a separate LTTng provider (`anytime` vs `ros2`) and won't appear in that list by default.

### To show custom tracepoints in ros2 trace --list:

You would need to patch the `tracetools_trace` package in your ROS2 installation:

1. Edit `/home/vscode/ros2_tracing_ws/src/ros2_tracing/tracetools_trace/tracetools_trace/tools/tracepoints.py`
2. Add: `interference_timer_init = 'anytime:interference_timer_init'`
3. Edit `/home/vscode/ros2_tracing_ws/src/ros2_tracing/tracetools_trace/tracetools_trace/tools/names.py`
4. Add the new tracepoint to `DEFAULT_EVENTS_ROS` list
5. Rebuild: `cd /home/vscode/ros2_tracing_ws && colcon build --packages-select tracetools_trace`

However, this is NOT recommended as it modifies core ROS2 packages. Instead, use the manual LTTng commands shown above.

## Verifying the Installation

To verify that the custom tracepoint is working:

```bash
# Run the test script
cd /home/vscode/workspace
./test_anytime_tracepoint.sh
```

You should see output containing:
```
anytime:interference_timer_init: { node_handle = ..., timer_period_ms = 100, execution_time_ms = 10, version = "1.0.0" }
```

## Architecture

The package follows the same structure as `tracetools`:

```
anytime_tracing/
├── CMakeLists.txt
├── package.xml
├── include/anytime_tracing/
│   ├── anytime_tracetools.h      # Public API with tracepoint declarations
│   ├── tp_call.h                  # LTTng tracepoint definitions
│   ├── config.h.in                # Build configuration
│   ├── utils.hpp                  # Utility functions
│   └── visibility_control.hpp     # Symbol visibility macros
└── src/
    ├── anytime_tracetools.c       # Tracepoint implementations
    ├── tp_call.c                  # LTTng tracepoint provider
    └── utils.cpp                  # Utility function implementations
```

## Adding New Tracepoints

To add a new custom tracepoint:

1. **Define in `tp_call.h`:**
   ```c
   TRACEPOINT_EVENT(
     TRACEPOINT_PROVIDER,
     my_new_event,
     TP_ARGS(...),
     TP_FIELDS(...)
   )
   ```

2. **Declare in `anytime_tracetools.h`:**
   ```c
   ANYTIME_DECLARE_TRACEPOINT(
     my_new_event,
     ...parameters...
   )
   ```

3. **Implement in `anytime_tracetools.c`:**
   ```c
   void ANYTIME_TRACEPOINT(my_new_event, ...)
   {
     CONDITIONAL_TP(my_new_event, ...);
   }
   ```

4. **Use in your code:**
   ```cpp
   #include "anytime_tracing/anytime_tracetools.h"
   
   ANYTIME_TRACEPOINT(my_new_event, ...);
   ```

5. **Rebuild:**
   ```bash
   colcon build --packages-select anytime_tracing your_package
   ```

## License

Apache 2.0
