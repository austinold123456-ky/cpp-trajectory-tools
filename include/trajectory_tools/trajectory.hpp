#pragma once

#include <cstddef>
#include <vector>

namespace trajectory_tools {

struct Trajectory {
    std::vector<double> timestamps;
    std::vector<std::vector<double>> positions;
};

struct JointLimits {
    std::vector<double> lower;
    std::vector<double> upper;
};

struct TrajectoryValidationReport {
    std::vector<std::size_t> non_finite_position_rows;
    std::vector<std::size_t> out_of_limit_rows;
    std::vector<std::size_t> invalid_timestamp_rows;

    bool is_valid() const;
};

double maximum_abs_displacement(const std::vector<double>& positions);
void validate_trajectory_structure(const Trajectory& trajectory);
std::vector<std::size_t> find_non_finite_rows(const Trajectory& trajectory);
std::vector<std::size_t> find_out_of_limit_rows(const Trajectory& trajectory,
                                                 const JointLimits& limits);
std::vector<std::size_t> find_invalid_timestamp_rows(const Trajectory& trajectory);
TrajectoryValidationReport validate_trajectory(const Trajectory& trajectory,
                                                const JointLimits& limits);

}  // namespace trajectory_tools
