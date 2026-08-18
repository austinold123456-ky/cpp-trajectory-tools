#include "trajectory_tools/trajectory.hpp"

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

}  // namespace trajectory_tools
