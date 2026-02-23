# Step 3: C++ Correctness Bugs

**Status:** Pending
**Day:** 1 (Tue Feb 24)
**Priority:** HIGH
**Time:** ~1.5h
**Dependencies:** None (but Step 2 fixes C5 already)

## Why

Null pointer dereferences and bounds errors would crash during evaluation.

**AE Impact:** Criterion 1 (Repeatability) - crashes = score of 1.

## Bugs to Fix

| ID | File | Issue | Fix |
| -- | ---- | ----- | --- |
| C1 | `anytime_core/.../anytime_client_base.hpp:68-69,129` | Null `goal_handle_` dereference in `log_result()` | Add null guard |
| C2 | `anytime_yolo/.../anytime_management.hpp:120-135` | Bounds access before check in `populate_result()` | Move bounds check to top of loop |
| C2b | `anytime_yolo/.../anytime_management.hpp:147` | Null `goal_handle_` dereference in `populate_result()` | Add null guard |
| C3 | `anytime_yolo/.../yolo.hpp:171-177` | `cudaMemset` after failed `cudaMalloc` | Reorder: check first, memset after |
| C4 | `anytime_yolo/.../yolo.hpp:1066` | Variable shadowing (`auto stream = this->stream`) | Remove redundant local |
| C5 | `anytime_yolo/.../yolo.hpp:1066-1067` | `cudaLaunchHostFunc` called with nullptr | Already fixed in Step 2 |

## Also Fix

- `anytime_yolo/.../anytime_management.hpp:254-261` - Remove `get_iteration_callback()` (strict aliasing violation via `reinterpret_cast`). After Step 2, callback is passed directly.
- Remove from `anytime_base.hpp:34` as well if the virtual method is declared there.

## Verification

```bash
colcon build --symlink-install
# Build should have no new warnings with -Wall -Wextra
```
