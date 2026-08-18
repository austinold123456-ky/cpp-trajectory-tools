#include "trajectory_tools/trajectory.hpp"

#include <iostream>
#include <vector>

int main() {
    const std::vector<double> positions{0.0, 0.4, -0.2, 0.6};

    std::cout << "Maximum absolute displacement: "
              << trajectory_tools::maximum_abs_displacement(positions) << '\n';
}
