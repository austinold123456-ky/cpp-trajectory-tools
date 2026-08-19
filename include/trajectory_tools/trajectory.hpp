#pragma once

#include <cstddef>
#include <optional>
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

struct TrajectorySummary {
    std::size_t sample_count;
    std::size_t joint_count;
    double start_timestamp;
    double end_timestamp;
    std::vector<double> minimum_positions;
    std::vector<double> maximum_positions;
};

double maximum_abs_displacement(const std::vector<double>& positions);
// timestamps must already be sorted in ascending order. Returns the index of the first
// timestamp greater than or equal to timestamp, or std::nullopt when no such timestamp exists.
std::optional<std::size_t> find_first_timestamp_at_or_after(
    const std::vector<double>& timestamps,
    double timestamp);
void validate_trajectory_structure(const Trajectory& trajectory);
std::vector<std::size_t> find_non_finite_rows(const Trajectory& trajectory);
std::vector<std::size_t> find_out_of_limit_rows(const Trajectory& trajectory,
                                                 const JointLimits& limits);
std::vector<std::size_t> find_invalid_timestamp_rows(const Trajectory& trajectory);
TrajectoryValidationReport validate_trajectory(const Trajectory& trajectory,
                                                const JointLimits& limits);
TrajectorySummary summarize_trajectory(const Trajectory& trajectory);

}  // namespace trajectory_tools
