#pragma once

#include <cstddef>
#include <vector>

namespace trajectory_tools {

struct Trajectory {
    std::vector<double> timestamps;
    std::vector<std::vector<double>> positions;
};

double maximum_abs_displacement(const std::vector<double>& positions);
void validate_trajectory_structure(const Trajectory& trajectory);
std::vector<std::size_t> find_non_finite_rows(const Trajectory& trajectory);
std::vector<std::size_t> find_invalid_timestamp_rows(const Trajectory& trajectory);

}  // namespace trajectory_tools
