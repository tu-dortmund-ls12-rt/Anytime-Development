#!/bin/bash
# Test script to verify anytime tracepoint is working

# Source the workspace
cd /home/vscode/workspace/packages
source install/setup.bash

# Create a tracing session
lttng create anytime_test_session

# Enable the custom tracepoint
lttng enable-event --userspace 'anytime:*'

# Also enable some ros2 events for comparison
lttng enable-event --userspace 'ros2:rcl_node_init'
lttng enable-event --userspace 'ros2:rcl_timer_init'

# Add context information
lttng add-context --userspace --type=vpid
lttng add-context --userspace --type=vtid
lttng add-context --userspace --type=procname

# Start tracing
lttng start

echo "Tracing started. Running interference timer node for 3 seconds..."

# Run the interference timer node briefly
timeout 3s ros2 run interference interference_timer_node --ros-args -p timer_period_ms:=100 -p execution_time_ms:=10 &
NODE_PID=$!

# Wait for the node to finish
wait $NODE_PID

# Stop tracing
lttng stop

# View the trace
echo ""
echo "Trace results:"
lttng view | grep -E "(anytime|interference)" | head -20

# Destroy the session
lttng destroy

echo ""
echo "Test completed."
