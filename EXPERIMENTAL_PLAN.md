# Experimental Plan for Scientific Paper

## Overview
This document outlines the complete plan for implementing three experiments:
1. **Monte Carlo Batch Size Scaling**
2. **Anytime YOLO Performance Analysis**
3. **Interference Experiment**

---

## Experiment 1: Monte Carlo Batch Size Scaling

### Objective
Test Monte Carlo with varying batch sizes across all configurations (reactive/proactive, single/multi-threaded) and measure:
- Number of iterations
- Number of batches
- Time per batch
- Cancellation delay

### Implementation Steps

#### 1.1 Add Tracepoints to Monte Carlo
**File**: `packages/src/anytime_monte_carlo/src/anytime_management.cpp`
- [ ] Add tracepoint for batch start
- [ ] Add tracepoint for batch completion
- [ ] Add tracepoint for iteration count
- [ ] Add tracepoint for cancellation events
- [ ] Track batch number and batch size

**File**: `packages/src/anytime_core/include/anytime_core/tracing.hpp`
- [ ] Define new tracepoint macros for Monte Carlo metrics:
  - `TRACE_MONTE_CARLO_BATCH_START(batch_num, batch_size)`
  - `TRACE_MONTE_CARLO_BATCH_END(batch_num, iterations_in_batch)`
  - `TRACE_MONTE_CARLO_ITERATION(iteration_num)`
  - `TRACE_MONTE_CARLO_CANCEL(delay_ms)`

#### 1.2 Create Experiment Configuration
**Directory**: `experiments/monte_carlo/`
- [ ] Create `experiments/monte_carlo/configs/` directory
- [ ] Create YAML config files for each test configuration
  - Configs for batch sizes: [1, 64, 4096, 16384, 65536, 262144]
  - Configs for reactive/proactive modes
  - Configs for single/multi-threading

#### 1.3 Create Evaluation Script
**File**: `experiments/monte_carlo/evaluate_monte_carlo.py`
- [ ] Parse tracepoint data from LTTng traces
- [ ] Calculate metrics:
  - Average iterations per batch
  - Total batches completed
  - Average time per batch
  - Cancellation delay statistics
- [ ] Generate plots:
  - Batch size vs. iterations plot
  - Batch size vs. time per batch
  - Cancellation delay comparison (reactive vs. proactive)
  - Threading comparison plots
- [ ] Export results to CSV/JSON

#### 1.4 Create Bash Execution Script
**File**: `experiments/monte_carlo/run_monte_carlo_experiments.sh`
- [ ] Start LTTng tracing session before each run
- [ ] Iterate through all configurations:
  - Batch sizes: [1, 64, 4096, 16384, 65536, 262144]
  - Reactive/Proactive: [reactive, proactive]
  - Threading: [single, multi]
- [ ] Run multiple trials (e.g., 3-5 runs per config)
- [ ] Stop LTTng session and save trace data
- [ ] Name traces with configuration details
- [ ] Call evaluation script after all runs complete

---

## Experiment 2: Anytime YOLO Performance Analysis

### Objective
Analyze YOLO performance with different configurations:
1. **Phase 1**: Baseline run (batch_size=1, proactive) to collect layer-wise results
2. **Phase 2**: Quality analysis to determine optimal cancellation point
3. **Phase 3**: Maximum throughput test (batch_size=25)
4. **Phase 4**: Full configuration sweep with optimized cancellation

### Implementation Steps

#### 2.1 Add Tracepoints to Anytime YOLO
**File**: `packages/src/anytime_yolo/src/anytime_management.cpp`
- [ ] Add tracepoint for layer computation start/end
- [ ] Add tracepoint for exit calculation start/end
- [ ] Add tracepoint for detection results after each layer
  - Include: layer_num, num_detections, confidence_scores, bounding_boxes
- [ ] Add tracepoint for total algorithm runtime
- [ ] Add tracepoint for batch processing

**File**: `packages/src/anytime_core/include/anytime_core/tracing.hpp`
- [ ] Define new tracepoint macros for YOLO metrics:
  - `TRACE_YOLO_LAYER_START(layer_num, batch_size)`
  - `TRACE_YOLO_LAYER_END(layer_num, duration_ms)`
  - `TRACE_YOLO_EXIT_CALC_START(layer_num)`
  - `TRACE_YOLO_EXIT_CALC_END(layer_num, duration_ms)`
  - `TRACE_YOLO_DETECTION_RESULT(layer_num, num_detections, avg_confidence)`
  - `TRACE_YOLO_TOTAL_RUNTIME(total_ms, batch_size)`

#### 2.2 Enhance Video Publisher for Controlled Playback
**File**: `packages/src/video_publisher/scripts/video_publisher.py`
- [ ] Ensure dynamic waiting based on YOLO runtime
- [ ] Add completion signal when all images processed
- [ ] Add tracepoint for image publication timing

#### 2.3 Create Experiment Configuration
**Directory**: `experiments/yolo/`
- [ ] Create `experiments/yolo/configs/` directory
- [ ] Phase 1 config: batch_size=1, proactive
- [ ] Phase 3 config: batch_size=25 (max), proactive
- [ ] Phase 4 configs: Full sweep
  - Batch sizes: [1, 3, 5, 10, 15, 20, 25]
  - Reactive/Proactive: [reactive, proactive]
  - Sync/Async: [sync, async]
  - Threading: [single, multi]

#### 2.4 Create Quality Analysis Script (Phase 2)
**File**: `experiments/yolo/analyze_quality.py`
- [ ] Load Phase 1 tracepoint data (layer-wise results)
- [ ] For each image, analyze detection quality after each layer:
  - Number of detections
  - Confidence scores
  - Bounding box quality (if ground truth available)
- [ ] Generate distribution plots:
  - "After how many layers can we cancel?" histogram
  - Quality vs. layer number plots
  - Confidence progression across layers
- [ ] Determine cancellation thresholds:
  - Find layer number for 90%, 95%, 99% quality
  - Output recommended cancellation points

#### 2.5 Create Evaluation Script (Phase 4)
**File**: `experiments/yolo/evaluate_yolo.py`
- [ ] Parse tracepoint data for all configurations
- [ ] Calculate metrics:
  - Total runtime per configuration
  - Layer computation times
  - Exit calculation overhead
  - Number of early cancellations
  - Resource usage (from batch size analysis)
  - Throughput (images/second)
- [ ] Generate plots:
  - Runtime vs. batch size
  - Quality vs. runtime tradeoff
  - Reactive vs. Proactive comparison
  - Sync vs. Async comparison
  - Threading impact analysis
  - Early cancellation benefits
- [ ] Export results to CSV/JSON

#### 2.6 Create Bash Execution Script
**File**: `experiments/yolo/run_yolo_experiments.sh`
- [ ] **Phase 1**: Run baseline (batch_size=1, proactive)
  - Start LTTng tracing
  - Launch video_publisher, action_server, action_client
  - Wait for completion
  - Stop tracing, save trace as "phase1_baseline"
  
- [ ] **Phase 2**: Run quality analysis
  - Call `analyze_quality.py` on Phase 1 data
  - Output cancellation thresholds to config file
  
- [ ] **Phase 3**: Run max throughput test (batch_size=25)
  - Start LTTng tracing
  - Run with maximum batch size
  - Stop tracing, save trace as "phase3_max_throughput"
  
- [ ] **Phase 4**: Full configuration sweep
  - For each configuration:
    - Start LTTng tracing
    - Launch all components
    - Wait for video_publisher completion
    - Stop tracing
    - Save trace with config details in name
  - Run multiple trials per configuration
  
- [ ] Call `evaluate_yolo.py` after all runs complete

---

## Experiment 3: Interference Experiment

### Objective
Demonstrate how batch size affects concurrent task execution by running an interference timer task alongside the anytime action to show scheduling patterns.

### Implementation Steps

#### 3.1 Create Combined Main for Monte Carlo Interference
**File**: `packages/src/interference_experiment/src/monte_carlo_interference_main.cpp` (NEW)

**Tasks**:
- [ ] Create main executable that instantiates:
  - AnytimeActionServer (Monte Carlo) node
  - AnytimeActionClient (Monte Carlo) node
  - InterferenceTimer node (OPTIONAL - only when enable_interference=true)
- [ ] Add command-line arguments or ROS parameters for:
  - executor_type: [single_threaded, multi_threaded]
  - batch_size for Monte Carlo
  - timer_period for interference timer
  - is_reactive_proactive
  - enable_interference: [true, false] - whether to include the interference timer
- [ ] Manually create executor based on executor_type parameter:
  - `rclcpp::executors::SingleThreadedExecutor` for single-threaded
  - `rclcpp::executors::MultiThreadedExecutor` for multi-threaded
- [ ] Add server and client nodes to the executor
- [ ] Conditionally add InterferenceTimer node only if enable_interference=true
- [ ] Spin the executor
- [ ] Handle graceful shutdown

#### 3.2 Create Combined Main for YOLO Interference
**File**: `packages/src/interference_experiment/src/yolo_interference_main.cpp` (NEW)

**Tasks**:
- [ ] Create main executable that instantiates:
  - AnytimeActionServer (YOLO) node
  - AnytimeActionClient (YOLO) node
  - InterferenceTimer node (OPTIONAL - only when enable_interference=true)
- [ ] Add command-line arguments or ROS parameters for:
  - executor_type: [single_threaded, multi_threaded]
  - batch_size for YOLO
  - timer_period for interference timer
  - is_reactive_proactive, is_sync_async
  - enable_interference: [true, false] - whether to include the interference timer
- [ ] Manually create executor based on executor_type parameter:
  - `rclcpp::executors::SingleThreadedExecutor` for single-threaded
  - `rclcpp::executors::MultiThreadedExecutor` for multi-threaded
- [ ] Add server and client nodes to the executor
- [ ] Conditionally add InterferenceTimer node only if enable_interference=true
- [ ] Spin the executor
- [ ] Handle graceful shutdown

#### 3.3 Create/Enhance Interference Timer Node
**File**: `packages/src/interference_experiment/src/interference_timer_node.cpp` (NEW)
**File**: `packages/src/interference_experiment/include/interference_experiment/interference_timer_node.hpp` (NEW)

**Tasks**:
- [ ] Create standalone interference timer node (not a component)
- [ ] Add tracepoints for timer execution:
  - Timer callback start
  - Timer callback end
  - Execution duration
  - Expected vs. actual period
- [ ] Add configurable timer period parameter
- [ ] Implement timer callback with busy-wait or computation to simulate interference

**File**: `packages/src/anytime_core/include/anytime_core/tracing.hpp`
- [ ] Define tracepoint macros for interference timer:
  - `TRACE_INTERFERENCE_TIMER_START(timer_id, expected_period_ms)`
  - `TRACE_INTERFERENCE_TIMER_END(timer_id, actual_delay_ms)`

#### 3.4 Update CMakeLists for Interference Experiment
**File**: `packages/src/interference_experiment/CMakeLists.txt`
- [ ] Add executable for `monte_carlo_interference_main`
- [ ] Add executable for `yolo_interference_main`
- [ ] Link against required dependencies:
  - anytime_monte_carlo library
  - anytime_yolo library
  - rclcpp
  - anytime_core
- [ ] Install executables

#### 3.5 Create Interference Launch Files
**File**: `packages/src/interference_experiment/launch/monte_carlo_interference.launch.py` (NEW)
- [ ] Create launch file that starts:
  - `monte_carlo_interference_main` executable with appropriate parameters
- [ ] Support parameters:
  - executor_type: [single_threaded, multi_threaded]
  - batch_size for Monte Carlo
  - timer_period for interference timer
  - is_reactive_proactive
  - enable_interference: [true, false] - controls whether interference timer is active

**File**: `packages/src/interference_experiment/launch/yolo_interference.launch.py` (NEW)
- [ ] Create launch file that starts:
  - video_publisher node (separate process)
  - `yolo_interference_main` executable with appropriate parameters
- [ ] Support parameters:
  - executor_type: [single_threaded, multi_threaded]
  - batch_size for YOLO
  - timer_period for interference timer
  - is_reactive_proactive, is_sync_async
  - enable_interference: [true, false] - controls whether interference timer is active

#### 3.5 Create Interference Evaluation Script
**File**: `experiments/interference/evaluate_interference.py`
- [ ] Parse tracepoint data from interference timer and anytime tasks
- [ ] Analyze execution patterns:
  - Timer execution frequency
  - Actual timer period vs. expected period
  - Gaps between timer executions
  - Correlation with batch size
  - Anytime task execution blocking time
- [ ] Calculate metrics:
  - Timer deadline miss rate
  - Average timer delay
  - Timer execution regularity (jitter)
  - Percentage of time interference timer gets scheduled
- [ ] Generate plots:
  - Execution timeline (gantt-style chart)
  - Timer period distribution for different batch sizes
  - Deadline miss rate vs. batch size
  - Side-by-side comparison: high vs. low batch size

#### 3.6 Create Experiment Configuration
**Directory**: `experiments/interference/`
- [ ] Create `experiments/interference/configs/` directory
- [ ] Create configs for Monte Carlo interference:
  - Batch sizes: [1, 64, 4096, 65536] (show extreme differences)
  - Executor: [single_threaded, multi_threaded]
  - Timer period: [10ms, 50ms, 100ms]
  - enable_interference: true (for Experiment 3 only)
- [ ] Create configs for YOLO interference:
  - Batch sizes: [1, 5, 15, 25]
  - Executor: [single_threaded, multi_threaded]
  - Timer period: [50ms, 100ms]
  - enable_interference: true (for Experiment 3 only)
- [ ] **Note**: For Experiments 1 and 2, the same executables can be used with enable_interference=false to run without the interference timer

#### 3.7 Create Bash Execution Script
**File**: `experiments/interference/run_interference_experiments.sh`
- [ ] **Monte Carlo Interference Tests** (Experiment 3 only):
  - For each configuration:
    - Start LTTng tracing
    - Launch `monte_carlo_interference.launch.py` with enable_interference=true
    - Run for fixed duration (e.g., 60 seconds)
    - Stop tracing
    - Save trace with config details
  - Multiple trials per configuration
  
- [ ] **YOLO Interference Tests** (Experiment 3 only):
  - For each configuration:
    - Start LTTng tracing
    - Launch `yolo_interference.launch.py` with enable_interference=true (includes video_publisher)
    - Wait for video_publisher completion
    - Stop tracing
    - Save trace with config details
  - Multiple trials per configuration
  
- [ ] Call `evaluate_interference.py` after all runs complete

**Note**: For Experiments 1 and 2, the same executables can be launched with enable_interference=false, eliminating the interference timer entirely and allowing the server/client to run in isolation for baseline measurements.

---

## Infrastructure Setup

### Directory Structure
```
experiments/
├── monte_carlo/
│   ├── configs/
│   │   ├── batch_1_reactive_single.yaml
│   │   ├── batch_1_reactive_multi.yaml
│   │   ├── batch_1_proactive_single.yaml
│   │   ├── batch_1_proactive_multi.yaml
│   │   ├── ... (all combinations)
│   ├── run_monte_carlo_experiments.sh
│   ├── evaluate_monte_carlo.py
│   └── README.md
├── yolo/
│   ├── configs/
│   │   ├── phase1_baseline.yaml
│   │   ├── phase3_max_throughput.yaml
│   │   ├── ... (phase 4 configs)
│   ├── run_yolo_experiments.sh
│   ├── analyze_quality.py
│   ├── evaluate_yolo.py
│   └── README.md
├── interference/
│   ├── configs/
│   │   ├── monte_carlo/
│   │   │   ├── batch_1_single_10ms.yaml
│   │   │   ├── ... (all combinations)
│   │   ├── yolo/
│   │   │   ├── batch_1_single_50ms.yaml
│   │   │   ├── ... (all combinations)
│   ├── run_interference_experiments.sh
│   ├── evaluate_interference.py
│   └── README.md
├── common/
│   ├── tracing_utils.py          # LTTng helper functions
│   ├── plotting_utils.py         # Common plotting functions
│   ├── metrics_utils.py          # Common metric calculations
│   └── README.md
└── README.md                      # Master experiment documentation
```

### Common Utilities

#### File: `experiments/common/tracing_utils.py`
- [ ] Functions to start/stop LTTng sessions
- [ ] Functions to parse trace data
- [ ] Functions to extract specific tracepoint events
- [ ] Timestamp conversion utilities

#### File: `experiments/common/plotting_utils.py`
- [ ] Standard plot styles for paper
- [ ] Common plot types (bar charts, line plots, heatmaps)
- [ ] Export functions (PNG, PDF, SVG)

#### File: `experiments/common/metrics_utils.py`
- [ ] Statistical analysis functions
- [ ] Aggregation across multiple trials
- [ ] Confidence interval calculations

---

## Master Execution Script

### File: `experiments/run_all_experiments.sh`
- [ ] Set up environment variables
- [ ] Ensure ROS2 workspace is sourced
- [ ] Create results directory structure
- [ ] Run Experiment 1 (Monte Carlo)
- [ ] Run Experiment 2 (YOLO)
- [ ] Run Experiment 3 (Interference)
- [ ] Generate summary report
- [ ] Package results for archival

---

## Step-by-Step Execution Order

### Phase A: Tracepoint Infrastructure
1. Update `anytime_core/include/anytime_core/tracing.hpp` with all new tracepoint macros
2. Add tracepoints to `anytime_monte_carlo/src/anytime_management.cpp`
3. Add tracepoints to `anytime_yolo/src/anytime_management.cpp`
4. Build and test tracepoints work

### Phase B: Interference Main Executables
5. Create `monte_carlo_interference_main.cpp`
6. Create `yolo_interference_main.cpp`
7. Create/update interference timer node
8. Update `interference_experiment/CMakeLists.txt`
9. Build and test interference executables work

### Phase C: Launch Files for Interference
10. Create `monte_carlo_interference.launch.py`
11. Create `yolo_interference.launch.py`
12. Test launch files with executables

### Phase D: Experiment 1 Setup
13. Create `experiments/monte_carlo/` directory structure
14. Create Monte Carlo config files
15. Create `run_monte_carlo_experiments.sh`
16. Create `evaluate_monte_carlo.py`

### Phase E: Experiment 2 Setup
17. Create `experiments/yolo/` directory structure
18. Create YOLO config files
19. Create `run_yolo_experiments.sh`
20. Create `analyze_quality.py`
21. Create `evaluate_yolo.py`

### Phase F: Experiment 3 Setup
22. Create `experiments/interference/` directory structure
23. Create interference config files
24. Create `run_interference_experiments.sh`
25. Create `evaluate_interference.py`

### Phase G: Common Utilities
26. Create `experiments/common/tracing_utils.py`
27. Create `experiments/common/plotting_utils.py`
28. Create `experiments/common/metrics_utils.py`

### Phase H: Master Scripts and Documentation
29. Create `experiments/run_all_experiments.sh`
30. Create all README.md files
31. Create master `experiments/README.md`

### Phase I: Testing and Validation
32. Run dry-run of Experiment 1
33. Run dry-run of Experiment 2
34. Run dry-run of Experiment 3
35. Validate all plots and metrics
36. Document any issues and fixes

### Phase J: Production Runs
37. Run full Experiment 1 (multiple trials)
38. Run full Experiment 2 (all phases)
39. Run full Experiment 3 (all configurations)
40. Generate final plots and tables for paper
41. Archive results

---

## Notes and Considerations

### Tracepoint Design
- Use LTTng for minimal overhead
- Ensure all tracepoints include timestamp
- Include unique identifiers (batch_num, layer_num, etc.)
- Keep tracepoint data compact but informative

### Configuration Management
- Use YAML files for all configurations
- Ensure naming convention is consistent
- Include metadata (date, description) in configs

### Data Management
- Store raw trace data separately from processed data
- Use consistent naming: `{experiment}_{config}_{trial}.trace`
- Export processed data to CSV/JSON for easier analysis

### Plotting Style
- Use consistent colors across all plots
- Include error bars (std dev or confidence intervals)
- Label axes clearly with units
- Use appropriate scales (log scale where needed)

### Reproducibility
- Document all dependencies and versions
- Include random seeds where applicable
- Save configuration alongside results
- Create checksums for result files

---

## Expected Deliverables

### For Paper
1. **Experiment 1 Plots**:
   - Batch size impact on iterations
   - Batch size impact on latency
   - Reactive vs. Proactive comparison
   - Threading impact

2. **Experiment 2 Plots**:
   - Quality distribution across layers
   - Runtime vs. batch size
   - Early cancellation benefits
   - Configuration comparison matrix

3. **Experiment 3 Plots**:
   - Execution timeline visualization
   - Deadline miss rate vs. batch size
   - Interference reduction with smaller batches

### For Repository
- Complete experiment framework
- All configuration files
- Automated execution scripts
- Evaluation and plotting scripts
- Documentation and README files
- Archived results with metadata

---

## Timeline Estimate

- **Phase A** (Tracepoints): 2-3 hours
- **Phase B** (Interference Mains): 3-4 hours
- **Phase C** (Launch files): 2 hours
- **Phase D** (Exp 1 Setup): 3-4 hours
- **Phase E** (Exp 2 Setup): 4-5 hours
- **Phase F** (Exp 3 Setup): 3-4 hours
- **Phase G** (Common Utils): 2-3 hours
- **Phase H** (Master Scripts): 2 hours
- **Phase I** (Testing): 3-4 hours
- **Phase J** (Production): Varies by experiment duration

**Total Estimated Setup Time**: 24-33 hours
**Production Run Time**: Depends on trial count and duration

---

## End of Plan
