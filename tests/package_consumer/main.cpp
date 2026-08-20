#include "trajectory_tools/trajectory.hpp"

int main() {
    const trajectory_tools::Trajectory trajectory{
        {0.0, 0.5},
        {{1.0, -2.0}, {3.0, 4.0}}};
    const trajectory_tools::TrajectorySummary summary =
        trajectory_tools::summarize_trajectory(trajectory);

    return summary.sample_count == 2 && summary.joint_count == 2 &&
                   summary.start_timestamp == 0.0 && summary.end_timestamp == 0.5 &&
                   summary.minimum_positions == std::vector<double>{1.0, -2.0} &&
                   summary.maximum_positions == std::vector<double>{3.0, 4.0}
               ? 0
               : 1;
}
