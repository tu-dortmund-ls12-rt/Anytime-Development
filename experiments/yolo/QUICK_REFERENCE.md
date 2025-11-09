# YOLO Experiments - Quick Reference

## Quick Start

### 1. Test Setup (Recommended First)
```bash
cd /home/vscode/workspace/experiments/yolo
./test_single_config.sh
```
This runs a quick test to verify everything is working. Press Ctrl+C after you see some processing.

### 2. Run Phase 1 (Baseline)
```bash
cd /home/vscode/workspace/experiments/yolo
./run_phase1_baseline.sh
```
Takes ~10-20 minutes for 3 trials (depends on number of images).

### 3. Run Phase 3 (Max Throughput)
```bash
cd /home/vscode/workspace/experiments/yolo
./run_phase3_max_throughput.sh
```
Takes ~10-20 minutes for 3 trials.

### 4. Evaluate Results
```bash
cd /home/vscode/workspace/experiments/yolo
python3 evaluate_yolo.py
```
Generates plots and CSV/JSON results in `results/` directory.

## File Locations

### Configuration Files
```
experiments/yolo/configs/
├── phase1_baseline.yaml        # Batch size 1, proactive
└── phase3_max_throughput.yaml  # Batch size 25, proactive
```

### Trace Output
```
experiments/yolo/traces/
├── phase1_baseline_trial1/     # Phase 1, trial 1
├── phase1_baseline_trial2/     # Phase 1, trial 2
├── phase1_baseline_trial3/     # Phase 1, trial 3
├── phase3_max_throughput_trial1/
├── phase3_max_throughput_trial2/
└── phase3_max_throughput_trial3/
```

### Results
```
experiments/yolo/results/
├── plots/
│   ├── layer_times.png             # Layer processing times
│   ├── throughput_comparison.png   # Throughput across configs
│   └── runtime_vs_batch_size.png   # Runtime scaling
├── yolo_summary.csv                # Summary metrics (good for paper)
└── yolo_detailed.json              # Full detailed data
```

## Scripts

| Script | Purpose | Duration |
|--------|---------|----------|
| `test_single_config.sh` | Quick test (manual stop) | ~30s |
| `run_phase1_baseline.sh` | Phase 1 - Baseline | ~10-20 min |
| `run_phase3_max_throughput.sh` | Phase 3 - Max throughput | ~10-20 min |
| `evaluate_yolo.py` | Analyze all traces | ~1-2 min |

## Configuration Parameters

### Server Parameters
- `batch_size`: Number of layers to process in one batch (1-25)
- `is_reactive_proactive`: "reactive" or "proactive"
- `is_sync_async`: "sync" or "async"
- `weights_path`: Path to YOLO model weights

### Client Parameters
- `send_cancel`: true/false - whether to send cancel request
- `cancel_delay_ms`: delay before canceling (if send_cancel=true)

## Common Commands

### Clean up old LTTng session
```bash
lttng destroy yolo_phase1_baseline
lttng destroy yolo_phase3_max_throughput
lttng destroy yolo_test
```

### View trace data
```bash
# View all events
babeltrace2 experiments/yolo/traces/phase1_baseline_trial1/

# View only YOLO events
babeltrace2 experiments/yolo/traces/phase1_baseline_trial1/ | grep 'anytime:yolo'

# Count events
babeltrace2 experiments/yolo/traces/phase1_baseline_trial1/ | grep 'anytime:yolo' | wc -l
```

### Manual execution (for debugging)
```bash
# Terminal 1: Video Publisher
source packages/install/setup.bash
ros2 launch video_publisher video_publisher.launch.py \
    image_path:=/home/vscode/workspace/packages/src/video_publisher/images

# Terminal 2: YOLO Client
source packages/install/setup.bash
ros2 launch anytime_yolo action_client.launch.py \
    send_cancel:=false

# Terminal 3: YOLO Server
source packages/install/setup.bash
ros2 launch anytime_yolo action_server.launch.py \
    is_reactive_proactive:=proactive \
    is_sync_async:=sync \
    batch_size:=1 \
    weights_path:=/home/vscode/workspace/packages/src/anytime_yolo/weights_32
```

## Tracepoints

Key YOLO tracepoints collected:
- `yolo_init` - Server initialization with config
- `yolo_layer_start` - Layer computation start (layer_num)
- `yolo_layer_end` - Layer computation end (layer_num)
- `yolo_exit_calculation_start` - Exit calculation start (layer_num)
- `yolo_exit_calculation_end` - Exit calculation end (layer_num, num_detections)
- `yolo_result` - Final result (processed_layers, result_processed_layers, total_detections)
- `yolo_image_processed` - Image received and preprocessed (width, height)
- `anytime_base_activate` - Goal accepted and processing started
- `anytime_base_reset` - Goal reset and domain state reset

## Expected Metrics

### Phase 1 (Baseline - Batch Size 1)
- Runtime per image: Higher (layer-by-layer)
- Throughput: Lower (0.5-2 images/sec)
- Layer times: Individual layer processing times
- Purpose: Understand layer-wise quality

### Phase 3 (Max Throughput - Batch Size 25)
- Runtime per image: Lower (all layers at once)
- Throughput: Higher (3-10 images/sec)
- Layer times: Batch processing times
- Purpose: Measure maximum performance

## Troubleshooting

### Error: "No trace directories found"
- Make sure you ran the experiment scripts first
- Check that traces exist in `experiments/yolo/traces/`

### Error: "babeltrace not found"
```bash
sudo apt-get update
sudo apt-get install lttng-tools babeltrace
```

### Error: "No valid image files found"
- Download images using the ROS2-Download-Images task
- Or manually download to `packages/src/video_publisher/images/`

### Error: "Failed to load YOLO weights"
- Download weights using the ROS2-Download-Weights task
- Or manually download to `packages/src/anytime_yolo/weights_32/`

### Processes not stopping cleanly
```bash
# Kill all ROS2 nodes
killall -9 ros2

# Or find specific processes
ps aux | grep ros2
kill -9 <PID>
```

## Next Steps

After completing Phase 1 and Phase 3:

1. **Analyze quality** (Phase 2):
   - Implement `analyze_quality.py`
   - Determine optimal cancellation points
   - Find layer where quality plateaus

2. **Full sweep** (Phase 4):
   - Create configs for batch sizes: 1, 3, 5, 10, 15, 20, 25
   - Test reactive vs proactive
   - Test single vs multi-threaded

3. **Paper writing**:
   - Use plots from `results/plots/`
   - Use metrics from `yolo_summary.csv`
   - Compare baseline vs max throughput
   - Discuss quality-performance tradeoffs

## Tips

- **Parallel execution**: Don't run multiple experiments simultaneously (LTTng session conflicts)
- **Disk space**: Each trial creates ~100-500MB of trace data
- **Memory**: YOLO requires significant GPU memory (4GB+)
- **Time estimate**: Full Phase 1 + Phase 3 = ~30-40 minutes
- **Multiple runs**: More trials = better statistics (recommend 3-5)

## Contact

See main README or experimental plan for more information.
