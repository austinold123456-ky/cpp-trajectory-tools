#pragma once

#include <vector>

namespace trajectory_tools {

struct Trajectory {
    std::vector<double> timestamps;
    std::vector<std::vector<double>> positions;
};

double maximum_abs_displacement(const std::vector<double>& positions);
void validate_trajectory_structure(const Trajectory& trajectory);

}  // namespace trajectory_tools
