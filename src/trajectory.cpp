#include "trajectory_tools/trajectory.hpp"

#include <cstddef>
#include <cmath>
#include <stdexcept>

namespace trajectory_tools {

double maximum_abs_displacement(const std::vector<double>& positions) {
    if (positions.empty()) {
        throw std::invalid_argument("positions must not be empty");
    }

    double maximum_displacement = 0.0;
    for (double position : positions) {
        const double displacement = std::abs(position - positions[0]);
        if (displacement > maximum_displacement) {
            maximum_displacement = displacement;
        }
    }

    return maximum_displacement;
}

void validate_trajectory_structure(const Trajectory& trajectory) {
    if (trajectory.timestamps.empty()) {
        throw std::invalid_argument("timestamps must not be empty");
    }

    if (trajectory.positions.empty()) {
        throw std::invalid_argument("positions must not be empty");
    }

    if (trajectory.timestamps.size() != trajectory.positions.size()) {
        throw std::invalid_argument(
            "timestamps and positions must have the same number of rows");
    }

    if (trajectory.positions.front().empty()) {
        throw std::invalid_argument("position rows must contain at least one joint");
    }

    const std::size_t joint_count = trajectory.positions.front().size();
    for (std::size_t row_index = 1; row_index < trajectory.positions.size(); ++row_index) {
        if (trajectory.positions[row_index].size() != joint_count) {
            throw std::invalid_argument("all position rows must have the same number of joints");
        }
    }
}

}  // namespace trajectory_tools
