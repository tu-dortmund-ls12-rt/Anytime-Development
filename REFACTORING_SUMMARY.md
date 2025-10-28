# Anytime Packages Refactoring Summary

## Overview
Consolidated common anytime algorithm functionality from `anytime_monte_carlo` and `anytime_yolo` packages into a new shared `anytime_core` package to eliminate code duplication and improve maintainability.

**Phase 1**: Consolidated server implementations (handle_goal, handle_cancel, handle_accepted)
**Phase 2**: Consolidated anytime_management common patterns (reactive/proactive functions, timing, waitable setup)
**Phase 3**: Consolidated constructor initialization (callback groups, waitables, node setup)

## Changes Made

### 1. Created New `anytime_core` Package

#### Package Structure
```
packages/src/anytime_core/
├── package.xml
├── CMakeLists.txt
└── include/anytime_core/
    ├── anytime_base.hpp
    ├── anytime_waitable.hpp
    └── anytime_server.hpp
```

#### New Files

**`anytime_base.hpp`**
- Template class `AnytimeBase<InterfaceType, GoalHandleType>`
- **NEW**: Provides default implementations for common anytime patterns:
  - `reactive_function()` - Calls compute(), optionally send_feedback(), notify_result()
  - `reactive_result_function()` - Check finish condition, calculate result, notify
  - `check_cancel_and_finish_reactive()` - Complete reactive execution flow
  - `proactive_function()` - Calls compute(), notify_result()
  - `proactive_result_function()` - Send feedback, calculate result, notify  
  - `check_cancel_and_finish_proactive()` - Complete proactive execution flow
  - `start()` - Common start logic
  - `compute()` - Timing wrapper that calls `compute_iteration()`
  - `send_feedback()` - Calls `populate_feedback()` and publishes
  - `calculate_result()` - Calls `populate_result()` and adds timestamps
  - `reset()` - Common reset logic + calls `reset_domain_state()`
- **Pure virtual methods** that derived classes must implement:
  - `compute_iteration()` - The actual domain-specific computation work
  - `populate_feedback(feedback)` - Fill in feedback message fields
  - `populate_result(result)` - Fill in result message fields  
  - `reset_domain_state()` - Reset domain-specific state variables
  - `should_finish()` - Return true when computation should complete
  - Optional override for async/GPU callback support
- **Node pointer** (`node_`) for logging (set by base class)
- **Protected helper method**:
  - `initialize_anytime_base<isReactiveProactive>(node, batch_size)` - **Phase 3 addition**
    - Creates callback group
    - Creates and configures waitables based on reactive/proactive mode
    - Registers waitables with node
    - Sets node pointer and batch size
    - Eliminates ~40 lines of duplicate constructor code from each package
  - Virtual methods for reactive/proactive patterns (NOW WITH DEFAULT IMPLEMENTATIONS)
  - Goal handle management
  - Timestamp tracking (receive, accept, start, cancel, result)
  - Waitable notification system
  - Batch size and performance metrics (timing in base class)
  - Thread-safe state management (is_running, is_active)

**`anytime_waitable.hpp`**
- Thread-safe waitable class for ROS2 event synchronization
- Uses atomic `is_triggered_` flag for lock-free notification
- Implements ROS2 waitable interface for integration with executors
- Improved version based on YOLO implementation

**`anytime_server.hpp`**
- Template class `AnytimeActionServerBase<ActionType>`
- Provides common action server functionality
- Implements standard action server callbacks:
  - `handle_goal()` - Validates server state, tracks timestamps, accepts/rejects goals
  - `handle_cancel()` - Notifies anytime management of cancellation
  - `handle_accepted()` - Sets goal handle, resets, activates, and starts processing
- Derived classes only need to implement:
  - Constructor with parameter parsing
  - `create_anytime_management()` factory method

### 2. Updated `anytime_monte_carlo` Package

#### Modified Files

**`package.xml`**
- Added dependency: `<depend>anytime_core</depend>`

**`CMakeLists.txt`**
- Added dependency: `anytime_core`
- Updated include directories to include anytime_core

**`include/anytime_monte_carlo/anytime_management.hpp`**
- **REFACTORED (Phase 2)**: Dramatically simplified by using base class implementations
- **REFACTORED (Phase 3)**: Constructor now just calls `initialize_anytime_base<isReactiveProactive>()`
- Changed base class: `anytime_core::AnytimeBase<Anytime, AnytimeGoalHandle>`
- Removed duplicate base class functionality
- **Constructor reduced from ~45 lines to 3 lines**:
  ```cpp
  AnytimeManagement(rclcpp::Node * node, int batch_size = 1) {
    this->template initialize_anytime_base<isReactiveProactive>(node, batch_size);
  }
  ```
- **Removed all reactive/proactive function implementations** (now in base class):
  - `reactive_function()` - deleted (~200 lines across both packages)
  - `reactive_result_function()` - deleted
  - `check_cancel_and_finish_reactive()` - deleted
  - `proactive_function()` - deleted
  - `proactive_result_function()` - deleted
  - `check_cancel_and_finish_proactive()` - deleted
  - `start()` - deleted (common implementation now in base)
  - `compute()` - deleted (timing logic now in base)
  - `send_feedback()` - deleted (publishing logic now in base)
  - `calculate_result()` - deleted (timestamp logic now in base)
  - `reset()` - deleted (common reset logic now in base)
- **New domain-specific implementations** (only Monte Carlo specifics):
  - `compute_iteration()` - Just the Pi approximation loop
  - `populate_feedback(feedback)` - Set feedback->feedback = count_total_
  - `populate_result(result)` - Calculate Pi, set iterations, batch_time
  - `reset_domain_state()` - Reset count variables
  - `should_finish()` - Check if loop_count_ >= goal
- Constructor: Simplified, sets node_ pointer, creates waitables (similar structure to before)
- Kept domain-specific variables: count_total_, count_inside_, loop_count_, x, y

**`include/anytime_monte_carlo/anytime_server.hpp`**
- **REFACTORED**: Now inherits from `anytime_core::AnytimeActionServerBase<anytime_interfaces::action::MonteCarlo>`
- Removed duplicate method declarations:
  - `handle_goal()`
  - `handle_cancel()`
  - `handle_accepted()`
  - Member variable `action_server_` (now in base)
  - Member variable `anytime_management_` (now in base)
- Kept package-specific code:
  - Constructor declaration
  - `create_anytime_management()` factory method

**`src/anytime_server.cpp`**
- **REFACTORED**: Updated to use base class
- Constructor now calls `AnytimeActionServerBase` constructor
- Removed duplicate implementations of:
  - `handle_goal()` (91 lines removed)
  - `handle_cancel()` 
  - `handle_accepted()`
- Kept package-specific code:
  - Parameter parsing (reactive/proactive mode, batch_size)
  - `create_anytime_management()` factory (creates appropriate template instance)

**Removed Files**
- `include/anytime_monte_carlo/anytime_base.hpp` (backward compatibility wrapper - no longer needed)
- `include/anytime_monte_carlo/anytime_waitable.hpp` (backward compatibility wrapper - no longer needed)
- `src/monte_carlo.hpp` (empty file)

### 3. Updated `anytime_yolo` Package

#### Modified Files

**`package.xml`**
- Added dependency: `<depend>anytime_core</depend>`

**`CMakeLists.txt`**
- Added dependency: `anytime_core`
- Updated include directories to include anytime_core

**`include/anytime_yolo/anytime_management.hpp`**
- **REFACTORED (Phase 2)**: Dramatically simplified by using base class implementations  
- **REFACTORED (Phase 3)**: Constructor initialization simplified using `initialize_anytime_base<isReactiveProactive>()`
- Changed base class: `anytime_core::AnytimeBase<Anytime, AnytimeGoalHandle>`
- Uses `anytime_core::AnytimeWaitable` for waitables
- Removed duplicate base class functionality
- **Constructor body reduced from ~45 lines to 1 line**:
  ```cpp
  // After YOLO-specific member initializers in constructor initialization list:
  {
    this->template initialize_anytime_base<isReactiveProactive>(node, batch_size);
  }
  ```
- **Removed all reactive/proactive function implementations** (same as Monte Carlo, now in base)
- **New domain-specific implementations** (only YOLO specifics):
  - `compute_iteration()` - YOLO inferStep loop with GPU callback support
  - `populate_feedback(feedback)` - Set processed_layers and detections
  - `populate_result(result)` - Calculate YOLO detections, bounding boxes, scaling
  - `reset_domain_state()` - Process image, create blob, reset YOLO state
  - `should_finish()` - Check if yolo_state_->isCompleted()
  - `get_iteration_callback()` - Return GPU callback for async, nullptr for sync
- **GPU Callback Support**:
  - `forward_finished_callback()` - CUDA callback for async mode
  - Properly integrated with base class through `get_iteration_callback()`
- Kept YOLO-specific extensions:
  - GPU callback support (`forward_finished_callback`)
  - CUDA integration  
  - TensorRT-specific functionality
  - Image processing (OpenCV, cv_bridge)
  - Detection message creation
- Kept domain-specific variables: yolo_, yolo_state_, input_cuda_buffer_, processed_layers_, etc.

**`include/anytime_yolo/anytime_server.hpp`**
- **REFACTORED**: Now inherits from `anytime_core::AnytimeActionServerBase<anytime_interfaces::action::Yolo>`
- Removed duplicate method declarations:
  - `handle_goal()`
  - `handle_cancel()`
  - `handle_accepted()`
  - Member variable `action_server_` (now in base)
  - Member variable `anytime_management_` (now in base)
- Kept package-specific code:
  - Constructor declaration
  - `create_anytime_management()` factory method with 5 parameters

**`src/anytime_server.cpp`**
- **REFACTORED**: Updated to use base class
- Constructor now calls `AnytimeActionServerBase` constructor with `use_intra_process_comms(true)`
- Removed duplicate implementations of:
  - `handle_goal()` (146 lines removed)
  - `handle_cancel()`
  - `handle_accepted()`
- Kept package-specific code:
  - Parameter parsing (5 parameters: reactive/proactive, passive/cooperative, sync/async, batch_size, weights_path)
  - 8-way `create_anytime_management()` factory (creates appropriate template instance based on 3 boolean parameters)

**`include/anytime_yolo/anytime_client.hpp`**
- Updated to use `anytime_core::AnytimeWaitable` directly

**Removed Files**
- `include/anytime_yolo/anytime_base.hpp` (backward compatibility wrapper - no longer needed)
- `include/anytime_yolo/anytime_waitable.hpp` (backward compatibility wrapper - no longer needed)

## Architecture Improvements

### Before Refactoring
```
anytime_monte_carlo/                 anytime_yolo/
├── anytime_base.hpp (duplicate)     ├── anytime_base.hpp (duplicate)
├── anytime_waitable.hpp (dup)       ├── anytime_waitable.hpp (dup)
├── anytime_server.hpp               ├── anytime_server.hpp
│   ├── inherits: rclcpp::Node       │   ├── inherits: rclcpp::Node
│   ├── handle_goal() (duplicate)    │   ├── handle_goal() (duplicate)
│   ├── handle_cancel() (duplicate)  │   ├── handle_cancel() (duplicate)
│   └── handle_accepted() (duplicate)│   └── handle_accepted() (duplicate)
└── anytime_management.hpp           └── anytime_management.hpp
```

### After Refactoring
```
anytime_core/
├── anytime_base.hpp (shared)
├── anytime_waitable.hpp (shared)
└── anytime_server.hpp (shared)
    ├── template: AnytimeActionServerBase<ActionType>
    ├── handle_goal() (single implementation)
    ├── handle_cancel() (single implementation)
    └── handle_accepted() (single implementation)

anytime_monte_carlo/                 anytime_yolo/
├── anytime_server.hpp               ├── anytime_server.hpp
│   ├── inherits: AnytimeActionServerBase<MonteCarlo>
│   └── create_anytime_management()  │   └── create_anytime_management()
└── anytime_management.hpp           └── anytime_management.hpp
    └── inherits: AnytimeBase            └── inherits: AnytimeBase
```

## Key Benefits

1. **Eliminated Massive Code Duplication**
   - **Phase 1**: Removed ~250+ lines of duplicated server implementation code
   - **Phase 2**: Removed ~400+ lines of duplicated management implementation code
   - **Phase 3**: Removed ~80+ lines of duplicated constructor initialization code
   - **Total**: Eliminated ~730+ lines of duplicate code across both packages
   - Single source of truth for:
     - Action server logic (handle_goal, handle_cancel, handle_accepted)
     - Reactive/proactive patterns (reactive_function, proactive_function, etc.)
     - Timing and performance metrics
     - Feedback and result publishing
     - State management and reset logic
   - Consistent behavior across all anytime algorithms

2. **Improved Maintainability**
   - Changes to core logic only need to be made once
   - Easier to add new anytime algorithm implementations (only implement 5 domain-specific methods + simple constructor)
   - Clear separation between common and package-specific code
   - Domain-specific classes are now ~75% smaller and much more focused
   - Constructor setup is completely unified - no room for inconsistency

3. **Better Abstraction**
   - Template-based design maintains compile-time type checking
   - No runtime overhead from abstraction  
   - Pure virtual methods clearly define the "contract" for new algorithms
   - Optional override for async/GPU callback support

4. **Cleaner Package Structure**
   - Each package only contains domain-specific code
   - Clear dependencies: monte_carlo/yolo → core → rclcpp
   - Removed backward compatibility wrappers

5. **Easier Testing**
   - Core functionality can be tested independently
   - Package-specific tests can focus on domain logic

## Remaining Package-Specific Code

### Monte Carlo
- Parameter parsing: `is_reactive_proactive`, `batch_size`
- Factory with 2 template instantiations (reactive/proactive)
- Pi approximation compute logic

### YOLO
- Parameter parsing: `is_reactive_proactive`, `is_passive_cooperative`, `is_sync_async`, `batch_size`, `weights_path`
- Factory with 8 template instantiations (2^3 combinations)
- TensorRT/CUDA integration
- GPU callback support
- Image processing logic

## Notes on Client Implementations

The client implementations in both packages (`anytime_client.hpp/cpp`) were analyzed but NOT consolidated because:

1. **Different Domain Requirements**
   - Monte Carlo: Timer-based goal sending
   - YOLO: Image subscription-based goal sending with publishers for results

2. **Different Metrics**
   - Monte Carlo: tracks `iterations` and `batch_size`
   - YOLO: tracks `processed_layers`, `result_processed_layers`, `average_batch_time`

3. **Similar Patterns, Different Data**
   - Both have similar timestamp tracking and `print_time_differences()` methods
   - However, the CSV output schemas differ based on domain-specific metrics
   - Abstraction would be complex and provide minimal benefit

The core server duplication was the primary concern and has been successfully eliminated.

## Build Instructions

1. Build `anytime_core` first (it has no custom dependencies beyond ROS2)
2. Build `anytime_monte_carlo` and `anytime_yolo` (they now depend on `anytime_core`)

```bash
cd packages
colcon build --packages-select anytime_core
colcon build --packages-select anytime_monte_carlo anytime_yolo
```

Or build everything:
```bash
cd packages
colcon build --symlink-install
```

## Migration Guide for Future Anytime Algorithms

To create a new anytime algorithm package:

1. **Create Package with Dependencies**
   ```xml
   <depend>anytime_core</depend>
   <depend>anytime_interfaces</depend>
   <depend>rclcpp</depend>
   <depend>rclcpp_action</depend>
   ```

2. **Implement Management Class**
   ```cpp
   template<bool IsReactiveProactive, /* other params */>
   class AnytimeManagement : public anytime_core::AnytimeBase<YourAction, GoalHandleType> {
     // Implement compute() and other domain-specific methods
   };
   ```

3. **Implement Server Class**
   ```cpp
   class AnytimeActionServer 
   : public anytime_core::AnytimeActionServerBase<your_interfaces::action::YourAction> {
   public:
     AnytimeActionServer(rclcpp::NodeOptions options);
   private:
     std::shared_ptr<anytime_core::AnytimeBase<Anytime, GoalHandleType>> 
       create_anytime_management(/* params */);
   };
   ```

4. **Implement Constructor and Factory**
   - Parse ROS2 parameters
   - Call base constructor with node name
   - Create action server
   - Instantiate appropriate management template

That's it! All the action server callbacks, timing, and state management are handled by the base class.
