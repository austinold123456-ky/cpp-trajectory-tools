# C++ Trajectory Tools

C++ Trajectory Tools is a C++17 library that validates multi-joint robot trajectories.

## Validation checks

- trajectory structure consistency
- finite joint-position values
- configured joint limits
- finite, strictly increasing timestamps

Row indices in validation reports are zero-based.

## Build and test

```powershell
cmake -S . -B build -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Debug
cmake --build build
ctest --test-dir build --output-on-failure
```

## CSV input format

```csv
timestamp,joint_0,joint_1,...
0.0,0.0,0.0
0.1,0.2,-0.1
```

- The first column is `timestamp`.
- Joint columns are named `joint_0`, `joint_1`, and so on; at least one joint column is required.
- Data rows contain unquoted, comma-separated numeric fields; at least one data row is required.
- Every data row must have the same number of fields as the header, with no leading or trailing field whitespace.
- Malformed files raise `TrajectoryFileError`.
- `nan` and `inf` are loadable values; validation decides whether values are finite, timestamps are finite and strictly increasing, and joints are within limits.

## Run the demo

```powershell
.\build\trajectory_demo.exe examples\invalid_trajectory.csv
```

The demo loads the CSV and prints a validation report, including zero-based rows with non-finite positions, limit violations, and invalid timestamps.

## Installation and package use

```powershell
cmake --install build --prefix install
```

Installation places public headers under `install/include/trajectory_tools/`, the library under `install/lib/`, and `trajectory_demo` under `install/bin/`. The CMake package files are installed under `install/lib/cmake/trajectory_tools/`.

An independent CMake consumer can use the installed package with the install prefix on `CMAKE_PREFIX_PATH`:

```cmake
find_package(trajectory_tools CONFIG REQUIRED)

add_executable(my_consumer main.cpp)
target_link_libraries(my_consumer PRIVATE trajectory_tools::trajectory_tools)
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
