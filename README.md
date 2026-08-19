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

## CSV input format

```csv
timestamp,joint_0,joint_1
0.0,0.0,0.0
0.1,0.2,-0.1
```

- The first column is `timestamp`.
- Joint columns are named `joint_0`, `joint_1`, and so on.
- Every data row must have the same number of columns as the header.
- Malformed files raise `TrajectoryFileError`.
- CSV loading does not decide whether values are finite, timestamps increase, or joints are within limits; validation does that.

## Run the demo

```powershell
.\build\trajectory_demo.exe examples\invalid_trajectory.csv
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

- supports in-memory validation and CSV trajectory loading
- does not yet support JSON or binary trajectory formats
