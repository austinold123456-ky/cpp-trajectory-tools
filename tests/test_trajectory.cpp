#include "trajectory_tools/trajectory.hpp"

#include <cmath>
#include <exception>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

constexpr double kTolerance = 1e-12;

bool approximately_equal(double actual, double expected) {
    return std::abs(actual - expected) <= kTolerance;
}

int check_displacement(const std::vector<double>& positions, double expected) {
    const double actual = trajectory_tools::maximum_abs_displacement(positions);
    if (approximately_equal(actual, expected)) {
        return 0;
    }

    std::cerr << "Failure: expected displacement " << expected << ", got " << actual
              << '\n';
    return 1;
}

int check_valid_trajectory(const trajectory_tools::Trajectory& trajectory) {
    try {
        trajectory_tools::validate_trajectory_structure(trajectory);
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Failure: valid trajectory threw an exception: " << error.what() << '\n';
        return 1;
    }
}

int check_invalid_trajectory(const trajectory_tools::Trajectory& trajectory,
                             const char* expected_message) {
    try {
        trajectory_tools::validate_trajectory_structure(trajectory);
        std::cerr << "Failure: invalid trajectory did not throw std::invalid_argument\n";
        return 1;
    } catch (const std::invalid_argument& error) {
        if (std::string(error.what()) == expected_message) {
            return 0;
        }

        std::cerr << "Failure: expected exception message \"" << expected_message << "\", got \""
                  << error.what() << "\"\n";
        return 1;
    } catch (...) {
        std::cerr << "Failure: invalid trajectory threw an unexpected exception type\n";
        return 1;
    }
}

int check_non_finite_rows(const trajectory_tools::Trajectory& trajectory,
                          const std::vector<std::size_t>& expected_rows) {
    const std::vector<std::size_t> actual_rows =
        trajectory_tools::find_non_finite_rows(trajectory);
    if (actual_rows == expected_rows) {
        return 0;
    }

    std::cerr << "Failure: non-finite row indices did not match the expected result\n";
    return 1;
}

int check_invalid_non_finite_trajectory(const trajectory_tools::Trajectory& trajectory) {
    try {
        trajectory_tools::find_non_finite_rows(trajectory);
        std::cerr << "Failure: structurally invalid trajectory did not throw std::invalid_argument\n";
        return 1;
    } catch (const std::invalid_argument&) {
        return 0;
    } catch (...) {
        std::cerr << "Failure: structurally invalid trajectory threw an unexpected exception type\n";
        return 1;
    }
}

}  // namespace

int main() {
    int failures = 0;

    failures += check_displacement({0.0, 0.4, -0.2, 0.6}, 0.6);
    failures += check_displacement({2.0, 1.0, 3.75, 0.25}, 1.75);

    try {
        trajectory_tools::maximum_abs_displacement({});
        std::cerr << "Failure: empty positions did not throw std::invalid_argument\n";
        ++failures;
    } catch (const std::invalid_argument&) {
    } catch (...) {
        std::cerr << "Failure: empty positions threw an unexpected exception type\n";
        ++failures;
    }

    failures += check_valid_trajectory({{0.0, 0.5}, {{1.0, 2.0}, {3.0, 4.0}}});
    failures += check_invalid_trajectory({{}, {{1.0, 2.0}}}, "timestamps must not be empty");
    failures += check_invalid_trajectory({{0.0}, {}}, "positions must not be empty");
    failures += check_invalid_trajectory(
        {{0.0, 0.5}, {{1.0, 2.0}}},
        "timestamps and positions must have the same number of rows");
    failures += check_invalid_trajectory(
        {{0.0}, {{}}}, "position rows must contain at least one joint");
    failures += check_invalid_trajectory(
        {{0.0, 0.5}, {{1.0, 2.0}, {3.0}}},
        "all position rows must have the same number of joints");

    failures += check_non_finite_rows({{0.0, 0.5}, {{1.0, 2.0}, {3.0, 4.0}}}, {});
    failures += check_non_finite_rows(
        {{0.0, 0.5, 1.0},
         {{std::numeric_limits<double>::quiet_NaN(), 1.0},
          {std::numeric_limits<double>::infinity(), 2.0},
          {-std::numeric_limits<double>::infinity(), 3.0}}},
        {0, 1, 2});
    failures += check_invalid_non_finite_trajectory({{}, {{1.0, 2.0}}});

    if (failures != 0) {
        return 1;
    }

    std::cout << "All trajectory_tools tests passed.\n";
    return 0;
}
