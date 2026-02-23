# Step 2: GPU Sync Fix

**Status:** Pending
**Day:** 1 (Tue Feb 24)
**Priority:** CRITICAL
**Time:** ~3h
**Dependencies:** None

## Why

The async GPU mode has race conditions that can cause crashes or incorrect results during YOLO experiments.

**AE Impact:** Criterion 1 (Repeatability) - YOLO experiments may fail without this.

## Approach

Use the simpler counter-only approach (not the full event-queue architecture from the old plan). This is sufficient for correctness and saves time.

## Changes

### File: `packages/src/anytime_yolo/include/anytime_yolo/anytime_management.hpp`

1. **Make `processed_layers_` atomic:**

   ```cpp
   std::atomic<int> processed_layers_{0};
   ```

2. **Add completion signal counter:**

   ```cpp
   std::atomic<int> completion_signals_{0};
   ```

3. **Rewrite `forward_finished_callback()` to be minimal and thread-safe:**

   ```cpp
   static void CUDART_CB forward_finished_callback(void* userData) {
     auto* self = static_cast<AnytimeManagement*>(userData);
     self->completion_signals_.fetch_add(1, std::memory_order_release);
     self->notify_waitable();
     // NO other state access - no goal_handle_, no logging
   }
   ```

4. **Add `process_gpu_completions()` override** - runs on executor thread only, drains completion signals and updates state safely.

5. **Update `reset_domain_state()`** - reset atomic counters.

### File: `packages/src/anytime_yolo/include/anytime_yolo/yolo.hpp`

6. **Guard `cudaLaunchHostFunc` with null check** (~line 1067):

   ```cpp
   if (callback != nullptr) {
     cudaLaunchHostFunc(this->stream, callback, userData);
   }
   ```

### File: `packages/src/anytime_core/include/anytime_core/anytime_base.hpp`

7. **Add virtual `process_gpu_completions()` hook** (default no-op).
8. **Call it in `reactive_anytime_function()` and `proactive_anytime_function()`** before compute.

## Verification

```bash
colcon build --packages-select anytime_yolo
# Test sync mode (regression)
# Test async mode - verify no crashes, correct layer counts
```
