#include "trajectory_tools/io.hpp"
#include "trajectory_tools/trajectory.hpp"

#include <exception>
#include <iostream>
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

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: trajectory_demo <trajectory.csv>\n";
        return 1;
    }

    try {
        const trajectory_tools::Trajectory trajectory = trajectory_tools::load_trajectory_csv(argv[1]);
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
    } catch (const std::exception& error) {
        std::cerr << "Error: " << error.what() << '\n';
        return 1;
    }
}
