# Anytime ROS 2

ROS 2 implementation of anytime algorithms for timely task completion in non-preemptive robotic systems.

> **Artifact Evaluation:** See [`ARTIFACT_EVALUATION.md`](ARTIFACT_EVALUATION.md) for step-by-step instructions to reproduce all paper figures and tables. The accepted paper is included as [`main.pdf`](main.pdf).

## Quick Start

### Prerequisites

- [Docker Engine](https://docs.docker.com/engine/install/) (24+) with Compose V2
- For GPU support: [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)

### Build and Run

```bash
# CPU-only
docker compose build anytime-cpu
docker compose run --rm anytime-cpu bash

# OR with GPU support
docker compose build anytime-gpu
docker compose run --rm anytime-gpu bash
```

Inside the container:

```bash
cd packages && colcon build --symlink-install && source install/setup.bash
cd ..

# Run smoke test (< 2 minutes)
./scripts/smoke_test.sh

# Run all CPU experiments with quick settings (~10 minutes)
./scripts/run_all.sh --quick --cpu-only

# Reproduce a specific paper figure
./scripts/reproduce_figure.sh 5a --quick
```

### VS Code Dev Container

Alternatively, open this folder in VS Code with the Dev Containers extension. Three configurations are available:

- `linux/` — Full environment with CUDA + TensorRT (GPU required)
- `linux-no-hardware/` — CPU-only environment
- `jetson/` — NVIDIA Jetson (ARM64)

## Experiments

Three experimental evaluations reproduce the paper results:

### Monte Carlo (Figures 5a, 5b)

Evaluates batch size scaling, mode comparison (reactive vs proactive), and threading impact on anytime Monte Carlo pi estimation.

```bash
./scripts/reproduce_figure.sh 5a        # Full run (~40 min)
./scripts/reproduce_figure.sh 5a --quick # Quick run (~5 min)
```

### Interference (Figure 6, Table I)

Measures timing interference between anytime batch processing and periodic timer tasks.

```bash
./scripts/reproduce_figure.sh 6          # Full run (~40 min)
./scripts/reproduce_figure.sh 6 --quick  # Quick run (~3 min)
```

### YOLO (Figures 7a, 7b) — GPU Required

Evaluates anytime YOLO object detection with layer-wise cancellation on GPU.

```bash
./scripts/reproduce_figure.sh 7a  # Quality progression (~30 min)
./scripts/reproduce_figure.sh 7b  # Runtime comparison (~1 hr)
```

See [`ARTIFACT_EVALUATION.md`](ARTIFACT_EVALUATION.md) for YOLO prerequisites (weights and test images).

## Tracing

All experiments use LTTng for low-overhead tracing. Custom tracepoints are defined in `packages/src/anytime_tracing/`.

Key tracepoints:

- `anytime:anytime_compute_entry/exit` — Batch computation timing
- `anytime:client_send_goal` — Client request tracking
- `anytime:yolo_layer_start/end` — YOLO layer processing
- `anytime:interference_timer_callback` — Timer interference

View traces:

```bash
babeltrace ~/.lttng-traces/session_name/
```

## Project Structure

```
Anytime-Development/
├── ARTIFACT_EVALUATION.md      # Artifact evaluation guide
├── main.pdf                    # Accepted paper
├── docker-compose.yml          # Container management (GPU + CPU)
├── LICENSE                     # Apache 2.0
├── requirements.txt            # Python dependencies
├── packages/src/
│   ├── anytime_core/           # Base anytime computation framework
│   ├── anytime_interfaces/     # ROS 2 action type definitions
│   ├── anytime_monte_carlo/    # Monte Carlo pi estimation (CPU)
│   ├── anytime_yolo/           # Anytime YOLO detection (GPU)
│   ├── anytime_tracing/        # LTTng tracepoint definitions
│   ├── experiments/            # Launch files and default configs
│   ├── interference/           # Timer interference test node
│   └── video_publisher/        # Video frame publisher for YOLO
├── experiments/
│   ├── monte_carlo/            # MC experiment scripts and evaluation
│   ├── interference/           # Interference experiment scripts and evaluation
│   └── yolo/                   # YOLO pipeline (9 steps)
└── scripts/
    ├── smoke_test.sh           # Quick validation (< 2 min)
    ├── run_all.sh              # End-to-end runner (--quick/--full/--cpu-only)
    └── reproduce_figure.sh     # Per-figure reproduction
```

## Citation

```bibtex
@inproceedings{teper2026anytime,
  title     = {Anytime {ROS} 2: Timely Task Completion in Non-Preemptive Robotic Systems},
  author    = {Teper, Harun},
  booktitle = {IEEE Real-Time and Embedded Technology and Applications Symposium (RTAS)},
  year      = {2026}
}
```

## License

This project is licensed under the Apache License 2.0. See [`LICENSE`](LICENSE) for details.
