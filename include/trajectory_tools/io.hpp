#pragma once

#include "trajectory_tools/trajectory.hpp"

#include <filesystem>
#include <stdexcept>

namespace trajectory_tools {

class TrajectoryFileError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

Trajectory load_trajectory_csv(const std::filesystem::path& path);

}  // namespace trajectory_tools
