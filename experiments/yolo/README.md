# YOLO Experiments

This directory contains experiments for analyzing the Anytime YOLO performance as described in the experimental plan.

## Experiment Overview

The YOLO experiments are divided into phases:

### Phase 1: Baseline Run
- **Purpose**: Collect layer-wise detection data to analyze quality progression
- **Configuration**: batch_size=1, proactive mode
- **Script**: `run_phase1_baseline.sh`
- **Output**: Layer-by-layer detection quality data for quality analysis

### Phase 2: Quality Analysis
- **Purpose**: Determine optimal cancellation points based on Phase 1 data
- **Script**: `analyze_quality.py` (to be implemented)
- **Output**: Recommended cancellation thresholds

### Phase 3: Maximum Throughput
- **Purpose**: Measure maximum throughput with all layers processed at once
- **Configuration**: batch_size=25, proactive mode
- **Script**: `run_phase3_max_throughput.sh`
- **Output**: Throughput and performance metrics

### Phase 4: Full Configuration Sweep
- **Purpose**: Test various configurations with optimized cancellation
- **Script**: `run_full_sweep.sh` (to be implemented)
- **Configurations**: Multiple batch sizes, modes, and threading options

## Directory Structure

```
experiments/yolo/
├── configs/                    # Experiment configuration files
│   ├── phase1_baseline.yaml
│   └── phase3_max_throughput.yaml
├── traces/                     # LTTng trace data
│   ├── phase1_baseline_trial1/
│   ├── phase1_baseline_trial2/
│   ├── phase3_max_throughput_trial1/
│   └── ...
├── results/                    # Analysis results
│   ├── plots/                  # Generated plots
│   ├── yolo_summary.csv        # Summary metrics
│   └── yolo_detailed.json      # Detailed results
├── run_phase1_baseline.sh      # Run Phase 1 experiments
├── run_phase3_max_throughput.sh # Run Phase 3 experiments
├── evaluate_yolo.py            # Evaluate experiment results
├── analyze_quality.py          # Analyze detection quality (Phase 2)
└── README.md                   # This file
```

## Prerequisites

1. **Build the workspace**:
   ```bash
   cd /home/vscode/workspace/packages
   colcon build --symlink-install
   ```

2. **Download YOLO weights**:
   ```bash
   # Run the download task or use:
   cd /home/vscode/workspace/packages/src/anytime_yolo
   wget https://tu-dortmund.sciebo.de/s/W86QE9hUscsUPeM/download -O weights.zip
   unzip -o weights.zip -d .
   rm weights.zip
   ```

3. **Download test images**:
   ```bash
   # Run the download task or use:
   mkdir -p /home/vscode/workspace/packages/src/video_publisher/images
   cd /home/vscode/workspace/packages/src/video_publisher/images
   wget https://tu-dortmund.sciebo.de/s/aA9MDhgN2lBmeZk/download -O images.zip
   unzip -o images.zip -d .
   rm images.zip
   ```

4. **Install LTTng** (should already be installed in dev container):
   ```bash
   sudo apt-get update
   sudo apt-get install lttng-tools lttng-modules-dkms liblttng-ust-dev
   ```

5. **Install Python dependencies**:
   ```bash
   pip3 install pandas numpy matplotlib
   ```

## Running Experiments

### Phase 1: Baseline

Collect layer-wise detection data:

```bash
cd /home/vscode/workspace/experiments/yolo
./run_phase1_baseline.sh
```

This will:
- Run 3 trials of the baseline configuration
- Process all images with batch_size=1
- Collect traces for each trial
- Save traces to `traces/phase1_baseline_trial1/`, `trial2/`, `trial3/`

### Phase 3: Maximum Throughput

Measure maximum throughput:

```bash
cd /home/vscode/workspace/experiments/yolo
./run_phase3_max_throughput.sh
```

This will:
- Run 3 trials of the max throughput configuration
- Process all images with batch_size=25
- Collect traces for each trial
- Save traces to `traces/phase3_max_throughput_trial1/`, `trial2/`, `trial3/`

## Analyzing Results

After running experiments, evaluate the results:

```bash
cd /home/vscode/workspace/experiments/yolo
python3 evaluate_yolo.py
```

This will:
- Parse all trace directories
- Calculate metrics (runtime, throughput, layer times)
- Generate plots in `results/plots/`
- Export summary CSV and detailed JSON

## Configuration Files

Configuration files are in YAML format and specify:
- **experiment**: name, phase, description
- **server_params**: batch_size, mode (reactive/proactive), sync/async
- **client_params**: cancellation settings
- **video_publisher**: image path
- **trace**: LTTng session configuration

## Metrics Collected

### Per-Goal Metrics
- Total runtime (ms)
- Number of detections
- Layers processed

### Layer-wise Metrics
- Processing time per layer (ms)
- Detection count after each layer
- Exit calculation time

### Summary Metrics
- Average runtime
- Throughput (images/second)
- Average detections per image
- Layer timing statistics

## Tracepoints Used

The following LTTng tracepoints are collected:
- `anytime:yolo_init` - Server initialization
- `anytime:yolo_layer_start` - Layer computation start
- `anytime:yolo_layer_end` - Layer computation end
- `anytime:yolo_exit_calculation_start` - Exit calculation start
- `anytime:yolo_exit_calculation_end` - Exit calculation end (with detection count)
- `anytime:yolo_result` - Final result with total detections
- `anytime:anytime_base_activate` - Goal start
- `anytime:anytime_base_reset` - Goal reset

## Troubleshooting

### LTTng Session Already Exists
```bash
lttng destroy yolo_phase1_baseline
# or
lttng destroy yolo_phase3_max_throughput
```

### No Images Found
Check that images are downloaded to:
```
/home/vscode/workspace/packages/src/video_publisher/images/
```

### YOLO Weights Not Found
Check that weights are downloaded to:
```
/home/vscode/workspace/packages/src/anytime_yolo/weights_32/
```

### Permission Denied on Scripts
```bash
chmod +x /home/vscode/workspace/experiments/yolo/*.sh
```

## Next Steps

After completing Phase 1 and Phase 3:

1. **Implement Phase 2** (Quality Analysis):
   - Create `analyze_quality.py` to analyze layer-wise detection quality
   - Determine optimal cancellation points
   - Generate quality progression plots

2. **Implement Phase 4** (Full Sweep):
   - Create configurations for various batch sizes
   - Test reactive vs proactive modes
   - Test single vs multi-threaded executors
   - Create `run_full_sweep.sh` script

3. **Write Paper**:
   - Use generated plots and metrics
   - Compare baseline vs maximum throughput
   - Analyze quality-performance tradeoffs
   - Discuss optimal cancellation strategies

## Expected Results

### Phase 1 (Baseline)
- Layer-by-layer processing times
- Detection quality progression across layers
- Identify at which layer detection quality plateaus

### Phase 3 (Maximum Throughput)
- Maximum achievable throughput
- Comparison with baseline runtime
- Understanding of batch processing benefits

## Contact

For questions or issues, please refer to the main project README or the experimental plan document.
