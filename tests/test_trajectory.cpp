#include "trajectory_tools/trajectory.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>
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

    if (failures != 0) {
        return 1;
    }

    std::cout << "All trajectory_tools tests passed.\n";
    return 0;
}
