# C++ Trajectory Tools

C++ Trajectory Tools is a C++17 library that validates multi-joint robot trajectories.

## Validation checks

- trajectory structure consistency
- finite joint-position values
- configured joint limits
- finite, strictly increasing timestamps

Row indices in validation reports are zero-based.

## Build

```powershell
cmake -S . -B build -G "MinGW Makefiles"
cmake --build build
```

## Run tests

```powershell
ctest --test-dir build --output-on-failure
```

## Run demo

```powershell
.\build\trajectory_demo.exe
```

## Usage

```cpp
#include "trajectory_tools/trajectory.hpp"

trajectory_tools::Trajectory trajectory{
    {0.0, 0.1},
    {{0.0, 0.0}, {0.5, -0.5}}};
trajectory_tools::JointLimits limits{{-1.0, -1.0}, {1.0, 1.0}};

const auto report = trajectory_tools::validate_trajectory(trajectory, limits);
if (report.is_valid()) {
    // The trajectory passed all current validation checks.
}
```

## Current scope

- in-memory trajectory validation only
- no trajectory file I/O yet
