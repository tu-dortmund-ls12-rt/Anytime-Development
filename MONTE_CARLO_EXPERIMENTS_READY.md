# Monte Carlo Experiments - Quick Start Guide

## Overview

Complete experimental evaluation pipeline for Monte Carlo anytime algorithm testing. All setup is complete and tested.

## Quick Commands

From workspace root (`/home/vscode/workspace/`):

```bash
# Quick test (10 seconds)
./run_monte_carlo_experiments.sh test

# Check status
./run_monte_carlo_experiments.sh status

# Run full experiment suite (~40 minutes)
./run_monte_carlo_experiments.sh run

# Evaluate existing traces only
./run_monte_carlo_experiments.sh evaluate

# Clean all data
./run_monte_carlo_experiments.sh clean
```

## What Was Implemented

✅ **24 Configurations**
- 6 batch sizes: 1, 64, 4096, 16384, 65536, 262144
- 2 modes: reactive, proactive
- 2 threading: single, multi
- 48 YAML config files created

✅ **Automated Experiment Runner**
- Runs 3 trials per configuration (72 total runs)
- 30 seconds per run
- Automatic LTTng tracing
- Total time: ~40 minutes

✅ **Evaluation Pipeline**
- Parses LTTng traces with babeltrace
- Extracts performance metrics
- Generates 5 comprehensive plots
- Exports to CSV and JSON

✅ **Test Verified**
- Single config test: ✓ PASSED
- Events captured: 100,205 in 10 seconds
- Tracing verified working correctly

## Experiment Configuration

### What Gets Tested
Each configuration combination is tested with 3 independent runs:
- **Goal period**: 1000ms (new goal every second)
- **Cancel timeout**: 250ms (cancel after 250ms)
- **Run duration**: 30 seconds per run

### What Gets Measured
- Iterations per batch
- Time per batch
- Total throughput (iterations/second)
- Cancellation delay
- Compute/Feedback/Result timing breakdown

## Output Structure

```
experiments/monte_carlo/
├── configs/              # 48 YAML configuration files
├── traces/               # 72 LTTng trace directories (after full run)
│   ├── batch_1_reactive_single_run1/
│   ├── batch_1_reactive_single_run2/
│   └── ... (72 total)
└── results/              # Analysis outputs
    ├── individual_runs.csv       # Raw data (72 rows)
    ├── aggregated_results.csv    # Averaged (24 rows)
    ├── aggregated_results.json   # Complete JSON
    └── plots/
        ├── batch_size_vs_iterations.png
        ├── batch_size_vs_time.png
        ├── cancellation_delay.png
        ├── threading_comparison.png
        └── throughput.png
```

## Tracepoints Captured

All `anytime:*` tracepoints including:
- `anytime_base_init/activate/deactivate/reset`
- `anytime_compute_entry/exit/iteration`
- `anytime_server_handle_goal/cancel/accepted`
- `anytime_client_send_goal/goal_response/feedback/result`
- `monte_carlo_init/iteration/result/reset`
- `anytime_send_feedback_entry/exit`
- `anytime_calculate_result_entry/exit`

## Expected Results

### Trace Data
- ~50-200 MB per trace directory
- ~100,000+ events per 10-second test
- CTF format (Common Trace Format)

### Plots
Each plot compares:
- Reactive vs. Proactive modes
- Single vs. Multi-threaded executors
- All 6 batch sizes

### Metrics
- **Batch size impact**: How batch size affects iterations and timing
- **Mode comparison**: Reactive vs. Proactive performance
- **Threading impact**: Single vs. Multi-threaded execution
- **Cancellation analysis**: Responsiveness to cancellation requests
- **Throughput scaling**: Overall algorithm throughput

## Workflow

### 1. Verify Setup (Already Done ✓)
```bash
./run_monte_carlo_experiments.sh test
```

### 2. Run Full Experiments
```bash
./run_monte_carlo_experiments.sh run
```
This will automatically:
1. Run all 72 experiments with tracing
2. Call evaluation script
3. Generate plots and export data

### 3. Review Results
```bash
# View aggregated results
cat experiments/monte_carlo/results/aggregated_results.csv

# View plots
ls experiments/monte_carlo/results/plots/

# Check status
./run_monte_carlo_experiments.sh status
```

## Customization

### Change Run Duration
Edit `experiments/monte_carlo/run_monte_carlo_experiments.sh`:
```bash
RUN_DURATION=30  # Change to desired seconds
```

### Change Number of Runs
Edit `experiments/monte_carlo/run_monte_carlo_experiments.sh`:
```bash
NUM_RUNS=3  # Change to desired number
```

### Add/Remove Batch Sizes
1. Edit `experiments/monte_carlo/generate_configs.py`
2. Run: `cd experiments/monte_carlo && python3 generate_configs.py`

## Documentation

Detailed documentation available in:
- `experiments/monte_carlo/README.md` - Complete usage guide
- `experiments/monte_carlo/SETUP_COMPLETE.md` - Implementation details
- `EXPERIMENTAL_PLAN.md` - Original plan and specifications

## Test Results

✅ **Single Configuration Test Passed**
- Configuration: batch_1_reactive_multi
- Duration: 10 seconds
- Events captured: 100,205
- Trace size: ~50 MB
- All tracepoints working correctly

## Next Steps

1. **Option A**: Run full experiment suite now
   ```bash
   ./run_monte_carlo_experiments.sh run
   ```

2. **Option B**: Review configuration first
   ```bash
   cat experiments/monte_carlo/configs/batch_1_reactive_multi_server.yaml
   ```

3. **Option C**: Do additional single config tests
   ```bash
   # Modify test_single_config.sh to test different configs
   cd experiments/monte_carlo
   vim test_single_config.sh  # Change TEST_CONFIG variable
   ./test_single_config.sh
   ```

## Troubleshooting

### No traces generated
```bash
# Check LTTng is working
lttng list --userspace | grep anytime
```

### Evaluation fails
```bash
# Check babeltrace is installed
babeltrace --version

# Manually check a trace
babeltrace experiments/monte_carlo/traces/<trace_dir>/ | grep anytime | head
```

### Python dependencies
```bash
pip3 install pandas numpy matplotlib
```

## Summary

The Monte Carlo experimental evaluation is **ready to use**:
- ✅ 24 configurations created
- ✅ Execution script ready
- ✅ Evaluation script ready
- ✅ Test passed (100,205 events captured)
- ✅ All dependencies verified
- ✅ Documentation complete

**Estimated time for full run**: 40 minutes
**Storage required**: ~5-10 GB for all traces
**Output**: 5 plots + CSV + JSON results

Run `./run_monte_carlo_experiments.sh help` for all commands.
