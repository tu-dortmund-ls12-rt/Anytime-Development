# Monte Carlo Experimental Evaluation

This directory contains the complete experimental setup for evaluating Monte Carlo performance with different configurations.

## Overview

The experiments test Monte Carlo with:
- **Batch sizes**: 1, 64, 4096, 16384, 65536, 262144
- **Modes**: reactive, proactive
- **Threading**: single-threaded, multi-threaded
- **Runs per configuration**: 3 (configurable)

**Total configurations**: 24 (6 batch sizes × 2 modes × 2 threading)
**Total runs**: 72 (24 configs × 3 runs)

## Directory Structure

```
monte_carlo/
├── configs/                    # YAML configuration files (24 pairs)
│   ├── batch_1_reactive_single_server.yaml
│   ├── batch_1_reactive_single_client.yaml
│   └── ... (48 files total)
├── traces/                     # LTTng trace data (created during experiments)
│   ├── batch_1_reactive_single_run1/
│   ├── batch_1_reactive_single_run2/
│   └── ... (72 directories after full run)
├── results/                    # Analysis results (created by evaluation)
│   ├── individual_runs.csv
│   ├── aggregated_results.csv
│   ├── aggregated_results.json
│   └── plots/
│       ├── batch_size_vs_iterations.png
│       ├── batch_size_vs_time.png
│       ├── cancellation_delay.png
│       ├── threading_comparison.png
│       └── throughput.png
├── generate_configs.py         # Script to generate configuration files
├── run_monte_carlo_experiments.sh  # Main experiment execution script
├── evaluate_monte_carlo.py     # Analysis and plotting script
├── test_single_config.sh       # Quick test with one configuration
└── README.md                   # This file
```

## Quick Start

### 1. Generate Configuration Files

```bash
cd /home/vscode/workspace/experiments/monte_carlo
python3 generate_configs.py
```

This creates 48 YAML files (24 server + 24 client configs) in the `configs/` directory.

### 2. Test with a Single Configuration

Before running the full experiment suite, test with a single configuration:

```bash
./test_single_config.sh
```

This runs one configuration for 10 seconds to verify everything works.

### 3. Run All Experiments

```bash
./run_monte_carlo_experiments.sh
```

This will:
- Run all 72 experiments (24 configs × 3 runs)
- Each run lasts 30 seconds
- Total time: ~40 minutes (including setup/teardown)
- Automatically call the evaluation script when done

### 4. Run Only Evaluation (if traces already exist)

```bash
python3 evaluate_monte_carlo.py
```

## Configuration Format

### Server Configuration
```yaml
anytime_server:
  ros__parameters:
    is_reactive_proactive: "reactive"  # or "proactive"
    multi_threading: true              # or false
    batch_size: 1                      # 1, 64, 4096, 16384, 65536, 262144
    log_level: "info"
```

### Client Configuration
```yaml
anytime_client:
  ros__parameters:
    goal_timer_period_ms: 1000         # Send goal every 1 second
    cancel_timeout_period_ms: 250      # Cancel after 250ms
    log_level: "info"
```

## Metrics Collected

The evaluation script extracts and analyzes:

### Per-Batch Metrics
- **Iterations per batch**: Number of Monte Carlo iterations in each batch
- **Time per batch**: Duration of each batch computation (ms)
- **Compute time**: Time spent in actual computation
- **Feedback time**: Time spent sending feedback to client
- **Result time**: Time spent calculating final result

### Overall Metrics
- **Total batches completed**: Number of batches finished in 30s
- **Total iterations**: Total Monte Carlo iterations executed
- **Cancellation delay**: Time from cancel request to deactivation (ms)
- **Throughput**: Iterations per second

### Aggregation
- Metrics are averaged across the 3 runs for each configuration
- Standard deviations are computed for variability analysis

## Generated Plots

1. **batch_size_vs_iterations.png**: Shows how many iterations are completed per batch for different batch sizes
2. **batch_size_vs_time.png**: Shows computation time per batch vs. batch size
3. **cancellation_delay.png**: Compares cancellation delays across configurations
4. **threading_comparison.png**: Compares single vs. multi-threaded performance
5. **throughput.png**: Overall throughput (iterations/second) for each configuration

All plots compare:
- Reactive vs. Proactive modes
- Single-threaded vs. Multi-threaded executors

## Output Files

### CSV Files
- `individual_runs.csv`: Raw metrics from each of the 72 runs
- `aggregated_results.csv`: Averaged metrics for each of 24 configurations

### JSON File
- `aggregated_results.json`: Complete results in JSON format for further processing

## Customization

### Modify Experiment Duration
Edit `run_monte_carlo_experiments.sh`:
```bash
RUN_DURATION=30  # Change to desired duration in seconds
```

### Modify Number of Runs
Edit `run_monte_carlo_experiments.sh`:
```bash
NUM_RUNS=3  # Change to desired number of runs per config
```

### Modify Batch Sizes
Edit `generate_configs.py`:
```python
batch_sizes = [1, 64, 4096, 16384, 65536, 262144]  # Add or remove values
```
Then regenerate configs:
```bash
python3 generate_configs.py
```

## Tracepoints Used

The experiments rely on these LTTng tracepoints:
- `anytime:anytime_compute_entry/exit` - Batch boundaries
- `anytime:anytime_compute_iteration` - Individual iterations
- `anytime:monte_carlo_iteration` - Monte Carlo specific iterations
- `anytime:anytime_send_feedback_entry/exit` - Feedback timing
- `anytime:anytime_calculate_result_entry/exit` - Result calculation timing
- `anytime:anytime_server_handle_cancel` - Cancellation requests
- `anytime:anytime_base_deactivate` - Computation deactivation

## Dependencies

- ROS 2 (with experiments package built)
- LTTng tools (`lttng-tools`, `lttng-modules-dkms`, `liblttng-ust-dev`)
- Python 3 with:
  - pandas
  - numpy
  - matplotlib
- babeltrace (for parsing traces)

## Troubleshooting

### No trace data generated
- Verify LTTng is installed: `lttng --version`
- Check if tracepoints are available: `lttng list --userspace | grep anytime`
- Ensure workspace is sourced before running experiments

### babeltrace not found
```bash
sudo apt-get install babeltrace
```

### Python dependencies missing
```bash
pip3 install pandas numpy matplotlib
```

### Experiments fail to launch
- Verify packages are built: `cd /home/vscode/workspace/packages && colcon build`
- Source the workspace: `source /home/vscode/workspace/packages/install/setup.bash`
- Check config files exist: `ls configs/`

## Expected Runtime

- Single configuration test: ~15 seconds
- Full experiment suite (72 runs × 30s): ~40-45 minutes
- Evaluation script: ~1-2 minutes

## Contact

For questions or issues with the experimental setup, please refer to the main project documentation.
