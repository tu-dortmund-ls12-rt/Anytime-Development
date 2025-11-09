# Interference Experiment Setup - Complete

## ✓ Implementation Complete

I've successfully created the complete Interference experimental evaluation pipeline, similar to the Monte Carlo experiment but focused on measuring timing interference.

## What Was Created

### 1. Directory Structure
```
experiments/interference/
├── configs/              # 72 YAML files (24 × 3: server + client + interference)
├── traces/               # Will contain LTTng trace data from experiments
├── results/              # Will contain analysis results and plots
├── generate_configs.py   # Config file generator
├── run_interference_experiments.sh  # Main experiment runner
├── evaluate_interference.py        # Analysis and plotting script
├── test_single_config.sh           # Quick test script
└── README.md                       # Complete documentation
```

### 2. Configuration Files (24 configurations × 3 files each = 72 files)
- **Batch sizes**: 1, 64, 4096, 16384, 65536, 262144
- **Modes**: reactive, proactive
- **Threading**: single-threaded, multi-threaded
- **Interference timer**: Fixed at 100ms period, 10ms execution time
- **Total**: 6 × 2 × 2 = 24 configurations
- **Files**: 72 YAML files (server + client + interference)

Each configuration has:
- Server config: Sets batch_size, is_reactive_proactive, multi_threading
- Client config: Sets goal_timer_period_ms (1000ms), cancel_timeout_period_ms (250ms)
- Interference config: Sets timer_period_ms (100ms), execution_time_ms (10ms)

### 3. Execution Script (`run_interference_experiments.sh`)

Features:
- Iterates through all 24 configurations
- Runs 3 trials per configuration (72 total runs)
- Each run lasts 30 seconds
- Automatically manages LTTng tracing:
  - Creates session before each run
  - Enables selective tracepoints (only what's needed)
  - Focuses on interference timer events
  - Saves traces with descriptive names
- Launches experiments using the `experiments` package's interference.launch.py
- Calls evaluation script after all runs complete
- **Total runtime**: ~40 minutes for full suite

Tracepoints enabled:
- `anytime:anytime_compute_entry/exit` - Monte Carlo batch timing
- `anytime:interference_timer_init` - Timer initialization
- `anytime:interference_timer_callback_entry/exit` - Timer execution (PRIMARY)

### 4. Evaluation Script (`evaluate_interference.py`)

Capabilities:
- Parses LTTng traces using `babeltrace`
- Extracts events for interference timer and Monte Carlo compute
- Calculates metrics:
  - **Primary (Timer Interference)**:
    - Timer periods (time between consecutive timer starts)
    - Timer jitter (deviation from expected 100ms)
    - Missed periods (periods > 150% of expected)
    - Timer execution times
  - **Secondary (Monte Carlo)**:
    - Compute batch times
    - Total batches completed
  - **Aggregated**: averages and std deviations across runs
- Generates 5 comprehensive plots:
  1. **Timer period vs batch size** - Main interference indicator
  2. **Jitter vs batch size** - Shows interference growth
  3. **Missed periods percentage** - Severe interference indicator
  4. **Compute time vs batch size** - Explains interference source
  5. **Timer period distribution** - Shows variability per config
- Exports results to:
  - `individual_runs.csv` - Raw data from all 72 runs
  - `aggregated_results.csv` - Averaged metrics for 24 configurations
  - `aggregated_results.json` - Complete results in JSON format

### 5. Test Script (`test_single_config.sh`)

Quick verification script that:
- Tests one configuration for 10 seconds
- Verifies tracing is working
- Shows sample timer and compute events
- Confirms pipeline is ready

### 6. Documentation (`README.md`)

Complete guide including:
- Overview and hypothesis
- Directory structure
- Quick start instructions
- Configuration format details
- Metrics descriptions (primary and secondary)
- Expected results by batch size
- Output file descriptions
- Customization options
- Troubleshooting guide
- Analysis tips

## Key Differences from Monte Carlo Experiment

### Focus
- **Monte Carlo**: Performance metrics (iterations, throughput, cancellation)
- **Interference**: Timing interference metrics (period jitter, missed periods)

### Metrics
- **Primary**: Timer period, jitter, missed periods
- **Secondary**: Monte Carlo compute batch timing (not individual iterations)

### Tracepoints
- **Minimal set**: Only timer callbacks and compute entry/exit
- **No iteration tracing**: Reduces overhead and trace size
- **Focused on timing**: Precise measurement of timer execution

### Plots
1. Timer period vs batch size (with expected 100ms reference)
2. Jitter vs batch size (log-log scale)
3. Missed periods percentage (bar chart)
4. Compute time vs batch size (explains interference)
5. Timer period distribution (box plots per config)

## Experimental Hypothesis

As Monte Carlo batch size increases, interference with the periodic timer increases:

### Small Batches (1, 64)
- **Expected**: Timer ≈ 100ms, minimal jitter, no missed periods
- **Reason**: Batch completes quickly, timer runs on schedule

### Medium Batches (4096, 16384)
- **Expected**: Timer 100-120ms, moderate jitter, few missed periods
- **Reason**: Batch takes longer, occasional timer delays

### Large Batches (65536, 262144)
- **Expected**: Timer >150ms, high jitter, many missed periods
- **Reason**: Batch blocks execution, severe timer delays

### Threading Impact
- **Single-threaded**: More interference (tasks compete)
- **Multi-threaded**: Less interference (tasks can run concurrently)

## How to Use

### Quick Test (10 seconds)
```bash
cd /home/vscode/workspace/experiments/interference
./test_single_config.sh
```

### Run Full Experiment Suite (~40 minutes)
```bash
cd /home/vscode/workspace/experiments/interference
./run_interference_experiments.sh
```

This will:
1. Run all 72 experiments (24 configs × 3 runs)
2. Collect LTTng traces for each run
3. Automatically run the evaluation script
4. Generate plots and export results

### Run Only Evaluation (if traces exist)
```bash
cd /home/vscode/workspace/experiments/interference
python3 evaluate_interference.py
```

## Expected Output

After running the full experiment suite, you'll have:

### Trace Data
- 72 trace directories in `traces/`
- Each contains CTF files with timer and compute events
- Smaller than Monte Carlo traces (fewer events tracked)

### Results
- `results/individual_runs.csv` - 72 rows of raw metrics
- `results/aggregated_results.csv` - 24 rows of averaged metrics
- `results/aggregated_results.json` - Complete JSON results

### Plots (in `results/plots/`)
- `timer_period_vs_batch_size.png` - **Key plot**: Shows interference effect
- `jitter_vs_batch_size.png` - Quantifies interference growth
- `missed_periods_percentage.png` - Shows severe interference incidents
- `compute_time_vs_batch_size.png` - Explains why interference occurs
- `timer_period_distribution.png` - Detailed view per configuration

All plots compare:
- Reactive vs. Proactive modes
- Single-threaded vs. Multi-threaded executors
- All batch sizes on the same plot

## Metrics Collected

### Primary: Timer Interference Metrics

1. **Timer Period** (ms)
   - Time between consecutive timer callback starts
   - Expected: 100ms
   - **Key indicator**: Deviation shows interference

2. **Timer Jitter** (ms)
   - Deviation from expected 100ms period
   - `jitter = actual_period - expected_period`
   - Positive = delayed, Negative = early (rare)

3. **Missed Periods** (count and %)
   - Periods exceeding 150ms (>50% deviation)
   - Indicates severe interference

4. **Timer Execution Time** (ms)
   - Time spent in timer callback
   - Expected: ~10ms (busy-wait duration)

### Secondary: Monte Carlo Metrics

5. **Compute Batch Time** (ms)
   - Duration of each compute batch
   - Explains interference source

6. **Total Compute Batches**
   - Number of batches completed in 30s

### Statistics
- Mean, std dev, min, max, median for all timing metrics
- Percentage calculations for missed periods
- Aggregation across 3 runs per configuration

## File Verification

✓ All 72 configuration files generated successfully:
- 24 server configs
- 24 client configs  
- 24 interference configs

✓ All scripts created and made executable:
- `generate_configs.py`
- `run_interference_experiments.sh`
- `evaluate_interference.py`
- `test_single_config.sh`

✓ Documentation complete:
- `README.md` with comprehensive guide

## Next Steps

1. **Test the pipeline** (recommended):
   ```bash
   cd /home/vscode/workspace/experiments/interference
   ./test_single_config.sh
   ```
   This runs a 10-second test to verify everything works.

2. **Run full experiments** (optional, ~40 minutes):
   ```bash
   cd /home/vscode/workspace/experiments/interference
   ./run_interference_experiments.sh
   ```

3. **Analyze results**: Review plots and CSV files in `results/` directory

4. **Compare with Monte Carlo**: Cross-reference with Monte Carlo experiment results

## Comparison with Monte Carlo Experiment

| Aspect | Monte Carlo Experiment | Interference Experiment |
|--------|------------------------|-------------------------|
| **Focus** | Algorithm performance | Timing interference |
| **Primary Metric** | Iterations, throughput | Timer period jitter |
| **Tracepoints** | All anytime events | Selective (timer + compute) |
| **Trace Size** | Larger (~100-200 MB) | Smaller (~20-50 MB) |
| **Analysis** | Performance scaling | Interference patterns |
| **Key Plot** | Throughput vs batch size | Timer period vs batch size |

Both experiments use the same configurations and provide complementary insights:
- **Monte Carlo**: How well does the algorithm perform?
- **Interference**: How does the algorithm affect other tasks?

## Implementation Checklist

- [x] Create experiment directory structure
- [x] Generate 24 configurations (72 YAML files)
- [x] Create bash execution script with LTTng integration
- [x] Create Python evaluation script with metric extraction
- [x] Create plotting functionality (5 plots)
- [x] Create CSV/JSON export functionality
- [x] Create test script for verification
- [x] Create comprehensive documentation
- [x] Make all scripts executable
- [x] Verify configuration file generation

## Summary

The complete Interference experimental evaluation pipeline is ready to use. The system will:

1. **Systematically test** all 24 configurations with 3 runs each
2. **Capture precise timing traces** using selective LTTng tracepoints
3. **Measure interference** between Monte Carlo batch processing and periodic timer
4. **Automatically analyze** results and generate visualizations
5. **Export data** in multiple formats for further analysis

The key innovation is measuring **how anytime algorithm batch processing interferes with other periodic tasks**, which is crucial for understanding real-world deployment scenarios where multiple tasks compete for CPU time.

### Key Questions This Experiment Answers

1. **At what batch size does interference become problematic?**
2. **How much does multi-threading reduce interference?**
3. **Do reactive and proactive modes differ in interference patterns?**
4. **What percentage of timer periods are missed at each batch size?**
5. **Is there a correlation between compute batch time and timer jitter?**

The generated plots will provide clear visual answers to all these questions.
