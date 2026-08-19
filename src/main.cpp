#include "trajectory_tools/trajectory.hpp"

#include <iostream>
#include <limits>
#include <vector>

namespace {

void print_row_indices(const std::vector<std::size_t>& row_indices) {
    for (std::size_t index = 0; index < row_indices.size(); ++index) {
        if (index > 0) {
            std::cout << ", ";
        }
        std::cout << row_indices[index];
    }
}

}  // namespace

int main() {
    const trajectory_tools::Trajectory trajectory{
        {0.0, 0.1, 0.1, 0.05},
        {{0.0, 0.0},
         {std::numeric_limits<double>::quiet_NaN(), 0.0},
         {1.5, 0.0},
         {0.0, -2.0}}};
    const trajectory_tools::JointLimits limits{{-1.0, -1.0}, {1.0, 1.0}};
    const trajectory_tools::TrajectoryValidationReport report =
        trajectory_tools::validate_trajectory(trajectory, limits);

    std::cout << "Trajectory validation report\n";
    std::cout << "Valid: " << (report.is_valid() ? "true" : "false") << '\n';
    std::cout << "Non-finite position rows (zero-based): ";
    print_row_indices(report.non_finite_position_rows);
    std::cout << '\n';
    std::cout << "Out-of-limit rows (zero-based): ";
    print_row_indices(report.out_of_limit_rows);
    std::cout << '\n';
    std::cout << "Invalid timestamp rows (zero-based): ";
    print_row_indices(report.invalid_timestamp_rows);
    std::cout << '\n';
}
