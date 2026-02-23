# Step 4: Unit Tests

**Status:** Pending
**Day:** 2 (Wed Feb 25)
**Priority:** HIGH
**Time:** ~3h
**Dependencies:** Steps 2, 3

## Why

Tests validate correctness and demonstrate software quality.

**AE Impact:** Criterion 3 (Quality) - score of 4 requires "test cases with known solutions." Score of 5 requires unit tests + system-level tests.

## New Files

### 1. `packages/src/anytime_core/test/test_anytime_waitable.cpp`

5 tests for guard condition/waitable:

- `trigger_makes_ready`
- `execute_calls_callback`
- `multiple_triggers`
- `not_ready_initially`
- `null_callback_no_crash`

### 2. `packages/src/anytime_core/test/test_anytime_base.cpp`

6 tests with MockAnytimeBase:

- `compute_calls_iteration_batch_times`
- `batch_iterations_default`
- `reset_clears_domain_state`
- `activate_sets_running`
- `deactivate_clears_running`
- `process_gpu_completions_default_noop`

### 3. `packages/src/anytime_monte_carlo/test/test_monte_carlo_management.cpp`

7 tests including seeded reproducibility:

- `single_iteration_increments_counters`
- `should_finish_at_goal`
- `should_not_finish_before_goal`
- `reset_clears_state`
- `populate_result_computes_pi`
- `populate_feedback_returns_count`
- `seeded_reproducibility`

### 4-5. CMakeLists.txt modifications

Add gtest targets to:

- `packages/src/anytime_core/CMakeLists.txt`
- `packages/src/anytime_monte_carlo/CMakeLists.txt`

## Verification

```bash
colcon build --packages-select anytime_core anytime_monte_carlo
colcon test --packages-select anytime_core anytime_monte_carlo
colcon test-result --verbose
```
