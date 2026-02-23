# Step 9: Documentation

**Status:** Pending
**Day:** 3 (Thu Feb 26)
**Priority:** HIGH
**Time:** ~3h
**Dependencies:** Step 2

## Why

Criterion 2 (score 5) requires explaining "all major components/modules, important design decisions, how to modify/extend." Criterion 3 (score 3+) requires file/class documentation.

**AE Impact:** Criteria 2 and 3 - the difference between scores of 3 and 5.

## New Files

### `docs/ARCHITECTURE.md`

- System overview diagram
- Package dependency graph
- Key design patterns (template method, guard conditions, cooperative notification)
- Execution flows (reactive, proactive)
- GPU async synchronization (updated for Step 2 fix)
- Tracing architecture

### `docs/EXPERIMENTS.md`

- Experiment matrix with durations
- **Explicit mapping to paper figures/tables** (which command produces which figure)
- Parameters and metrics for each experiment
- How to modify experiments (for Criterion 2 score 5: "how to modify/extend")
- What would break experiments (for Criterion 2 score 5: "what would break")

### `docs/COMPONENTS.md`

Documents each package's purpose, key classes, and interfaces:

- `anytime_core` - Template parameters, virtual interface, AnytimeWaitable, AnytimeServer, AnytimeClientBase
- `anytime_yolo` - yolo.hpp wrapper, anytime_management.hpp, GPU sync, early exit, weights
- `anytime_monte_carlo` - Stochastic computation, iteration logic, reproducible seeds
- `anytime_tracing` - LTTng tracepoint definitions, conditional compilation
- `anytime_interfaces` - Action definitions
- `interference` - Timer node, jitter measurement
- `video_publisher` - Frame publishing
- `experiments` - Launch files, unified configs
