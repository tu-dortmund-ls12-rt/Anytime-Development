# Interference Experiment - Quick Reference

## Overview

The interference experiment measures how Monte Carlo batch processing interferes with a periodic timer task. It complements the Monte Carlo experiment by focusing on **timing interference** rather than algorithm performance.

## Key Metrics

### Primary: Timer Interference
- **Timer Period**: Time between timer callbacks (expected: 100ms)
- **Timer Jitter**: Deviation from expected period
- **Missed Periods**: Periods > 150% of expected (severe interference)

### Secondary: Monte Carlo  
- **Compute Batch Time**: Duration of each batch
- **Total Batches**: Number completed in 30s

## Hypothesis

As batch size increases → More interference:
- **Small batches (1, 64)**: Minimal interference, timer ≈ 100ms
- **Large batches (65536, 262144)**: Severe interference, timer > 150ms

## Quick Start

### 1. Generate Configs
```bash
cd /home/vscode/workspace/experiments/interference
python3 generate_configs.py
```

### 2. Test (10 seconds)
```bash
./test_single_config.sh
```

### 3. Run All Experiments (~40 minutes)
```bash
./run_interference_experiments.sh
```

### 4. View Results
```bash
ls results/plots/
cat results/aggregated_results.csv
```

## Key Files

- **generate_configs.py**: Creates 72 YAML configs (24 configs × 3 files)
- **run_interference_experiments.sh**: Runs 72 experiments with LTTng tracing
- **evaluate_interference.py**: Analyzes traces and generates 5 plots
- **test_single_config.sh**: Quick 10-second verification test
- **README.md**: Complete documentation

## Configuration

- **Batch sizes**: 1, 64, 4096, 16384, 65536, 262144
- **Modes**: reactive, proactive
- **Threading**: single, multi
- **Timer**: 100ms period, 10ms execution (fixed)
- **Runs per config**: 3
- **Total**: 72 runs (24 configs × 3)

## Plots Generated

1. `timer_period_vs_batch_size.png` - **Main plot**: Shows interference effect
2. `jitter_vs_batch_size.png` - Quantifies interference growth  
3. `missed_periods_percentage.png` - Shows severe interference incidents
4. `compute_time_vs_batch_size.png` - Explains why interference occurs
5. `timer_period_distribution.png` - Detailed view per configuration

## Tracepoints Used

**Selective tracing** (minimal overhead):
- `anytime:interference_timer_callback_entry/exit` - Timer execution
- `anytime:anytime_compute_entry/exit` - Compute batch timing

## Expected Results

| Batch Size | Timer Period | Jitter | Missed Periods |
|------------|--------------|--------|----------------|
| 1, 64      | ≈100ms       | <5ms   | 0%             |
| 4096, 16384| 100-120ms    | 5-20ms | <10%           |
| 65536+     | >150ms       | >50ms  | >30%           |

**Threading**: Multi-threaded shows less interference than single-threaded.

## For More Details

See `README.md` and `SETUP_COMPLETE.md` in this directory.

## Relationship to Monte Carlo Experiment

| Aspect | Monte Carlo | Interference |
|--------|-------------|--------------|
| Focus | Performance | Timing interference |
| Primary Metric | Throughput | Timer jitter |
| Answers | "How fast?" | "How disruptive?" |

Both use the same configurations and provide complementary insights into anytime algorithm behavior.
