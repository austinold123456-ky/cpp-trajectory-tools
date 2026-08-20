#include "trajectory_tools/trajectory.hpp"
#include "trajectory_tools/io.hpp"

#include <cmath>
#include <exception>
#include <filesystem>
#include <fstream>
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

int check_timestamp_lookup() {
    const std::vector<double> timestamps{0.1, 0.3, 0.5};
    const auto before_first = trajectory_tools::find_first_timestamp_at_or_after(timestamps, 0.0);
    const auto exact_match = trajectory_tools::find_first_timestamp_at_or_after(timestamps, 0.3);
    const auto between_timestamps =
        trajectory_tools::find_first_timestamp_at_or_after(timestamps, 0.4);
    const auto after_last = trajectory_tools::find_first_timestamp_at_or_after(timestamps, 0.6);

    if (before_first && *before_first == 0 && exact_match && *exact_match == 1 &&
        between_timestamps && *between_timestamps == 2 && !after_last) {
        return 0;
    }

    std::cerr << "Failure: timestamp lookup did not return the expected indices\n";
    return 1;
}

int check_non_finite_timestamp_lookup(double timestamp) {
    const std::vector<double> timestamps{0.1, 0.3, 0.5};
    try {
        trajectory_tools::find_first_timestamp_at_or_after(timestamps, timestamp);
        std::cerr << "Failure: non-finite timestamp lookup did not throw std::invalid_argument\n";
        return 1;
    } catch (const std::invalid_argument&) {
        return 0;
    } catch (...) {
        std::cerr << "Failure: non-finite timestamp lookup threw an unexpected exception type\n";
        return 1;
    }
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

int check_trajectory_summary(const trajectory_tools::Trajectory& trajectory) {
    try {
        const trajectory_tools::TrajectorySummary summary =
            trajectory_tools::summarize_trajectory(trajectory);
        if (summary.sample_count == 3 && summary.joint_count == 2 &&
            approximately_equal(summary.start_timestamp, 0.0) &&
            approximately_equal(summary.end_timestamp, 1.0) &&
            summary.minimum_positions == std::vector<double>{-3.0, -1.0} &&
            summary.maximum_positions == std::vector<double>{2.0, 4.0}) {
            return 0;
        }

        std::cerr << "Failure: trajectory summary did not contain the expected values\n";
        return 1;
    } catch (const std::exception& error) {
        std::cerr << "Failure: valid trajectory summary threw an exception: " << error.what() << '\n';
        return 1;
    }
}

int check_invalid_trajectory_summary(const trajectory_tools::Trajectory& trajectory) {
    try {
        trajectory_tools::summarize_trajectory(trajectory);
        std::cerr << "Failure: invalid trajectory summary did not throw std::invalid_argument\n";
        return 1;
    } catch (const std::invalid_argument&) {
        return 0;
    } catch (...) {
        std::cerr << "Failure: invalid trajectory summary threw an unexpected exception type\n";
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

int check_out_of_limit_rows(const trajectory_tools::Trajectory& trajectory,
                            const trajectory_tools::JointLimits& limits,
                            const std::vector<std::size_t>& expected_rows) {
    try {
        const std::vector<std::size_t> actual_rows =
            trajectory_tools::find_out_of_limit_rows(trajectory, limits);
        if (actual_rows == expected_rows) {
            return 0;
        }

        std::cerr << "Failure: out-of-limit row indices did not match the expected result\n";
        return 1;
    } catch (const std::exception& error) {
        std::cerr << "Failure: valid limit configuration threw an exception: " << error.what() << '\n';
        return 1;
    }
}

int check_invalid_limits(const trajectory_tools::Trajectory& trajectory,
                         const trajectory_tools::JointLimits& limits,
                         const char* expected_message) {
    try {
        trajectory_tools::find_out_of_limit_rows(trajectory, limits);
        std::cerr << "Failure: invalid joint limits did not throw std::invalid_argument\n";
        return 1;
    } catch (const std::invalid_argument& error) {
        if (std::string(error.what()) == expected_message) {
            return 0;
        }

        std::cerr << "Failure: expected exception message \"" << expected_message << "\", got \""
                  << error.what() << "\"\n";
        return 1;
    } catch (...) {
        std::cerr << "Failure: invalid joint limits threw an unexpected exception type\n";
        return 1;
    }
}

int check_invalid_timestamp_rows(const trajectory_tools::Trajectory& trajectory,
                                 const std::vector<std::size_t>& expected_rows) {
    try {
        const std::vector<std::size_t> actual_rows =
            trajectory_tools::find_invalid_timestamp_rows(trajectory);
        if (actual_rows == expected_rows) {
            return 0;
        }

        std::cerr << "Failure: invalid timestamp row indices did not match the expected result\n";
        return 1;
    } catch (const std::exception& error) {
        std::cerr << "Failure: valid timestamp trajectory threw an exception: " << error.what() << '\n';
        return 1;
    }
}

int check_structurally_invalid_timestamp_trajectory(
    const trajectory_tools::Trajectory& trajectory) {
    try {
        trajectory_tools::find_invalid_timestamp_rows(trajectory);
        std::cerr << "Failure: structurally invalid trajectory did not throw std::invalid_argument\n";
        return 1;
    } catch (const std::invalid_argument&) {
        return 0;
    } catch (...) {
        std::cerr << "Failure: structurally invalid trajectory threw an unexpected exception type\n";
        return 1;
    }
}

int check_valid_trajectory_report(const trajectory_tools::Trajectory& trajectory,
                                  const trajectory_tools::JointLimits& limits) {
    try {
        const trajectory_tools::TrajectoryValidationReport report =
            trajectory_tools::validate_trajectory(trajectory, limits);
        if (report.non_finite_position_rows.empty() && report.out_of_limit_rows.empty() &&
            report.invalid_timestamp_rows.empty() && report.is_valid()) {
            return 0;
        }

        std::cerr << "Failure: valid trajectory report was not empty and valid\n";
        return 1;
    } catch (const std::exception& error) {
        std::cerr << "Failure: valid trajectory report threw an exception: " << error.what() << '\n';
        return 1;
    }
}

int check_invalid_trajectory_report(const trajectory_tools::Trajectory& trajectory,
                                    const trajectory_tools::JointLimits& limits) {
    try {
        trajectory_tools::validate_trajectory(trajectory, limits);
        std::cerr << "Failure: structurally invalid report trajectory did not throw std::invalid_argument\n";
        return 1;
    } catch (const std::invalid_argument&) {
        return 0;
    } catch (...) {
        std::cerr << "Failure: structurally invalid report trajectory threw an unexpected exception type\n";
        return 1;
    }
}

int check_invalid_trajectory_report_rows(const trajectory_tools::Trajectory& trajectory,
                                         const trajectory_tools::JointLimits& limits) {
    try {
        const trajectory_tools::TrajectoryValidationReport report =
            trajectory_tools::validate_trajectory(trajectory, limits);
        if (report.non_finite_position_rows == std::vector<std::size_t>{1} &&
            report.out_of_limit_rows == std::vector<std::size_t>{2, 3} &&
            report.invalid_timestamp_rows == std::vector<std::size_t>{2, 3} && !report.is_valid()) {
            return 0;
        }

        std::cerr << "Failure: trajectory report row indices did not match the expected result\n";
        return 1;
    } catch (const std::exception& error) {
        std::cerr << "Failure: invalid trajectory report threw an exception: " << error.what() << '\n';
        return 1;
    }
}

bool write_test_csv(const std::filesystem::path& path, const std::string& contents) {
    std::ofstream output(path);
    if (!output) {
        return false;
    }

    output << contents;
    return output.good();
}

int remove_test_csv(const std::filesystem::path& path) {
    std::error_code error;
    std::filesystem::remove(path, error);
    if (!error) {
        return 0;
    }

    std::cerr << "Failure: could not remove temporary CSV file " << path << '\n';
    return 1;
}

int check_valid_csv_load(const std::filesystem::path& path) {
    int failures = remove_test_csv(path);
    if (failures != 0) {
        return failures;
    }

    if (!write_test_csv(path, "timestamp,joint_0,joint_1\n0.0,1.0,2.0\n0.5,3.0,4.0\n")) {
        std::cerr << "Failure: could not create temporary CSV file " << path << '\n';
        return 1;
    }

    try {
        const trajectory_tools::Trajectory trajectory = trajectory_tools::load_trajectory_csv(path);
        if (trajectory.timestamps != std::vector<double>{0.0, 0.5} ||
            trajectory.positions != std::vector<std::vector<double>>{{1.0, 2.0}, {3.0, 4.0}}) {
            std::cerr << "Failure: valid CSV did not load the expected trajectory values\n";
            ++failures;
        }
    } catch (const std::exception& error) {
        std::cerr << "Failure: valid CSV threw an exception: " << error.what() << '\n';
        ++failures;
    }

    return failures + remove_test_csv(path);
}

int check_valid_crlf_csv_load(const std::filesystem::path& path) {
    int failures = remove_test_csv(path);
    if (failures != 0) {
        return failures;
    }

    if (!write_test_csv(path, "timestamp,joint_0\r\n0.0,1.0\r\n")) {
        std::cerr << "Failure: could not create temporary CSV file " << path << '\n';
        return 1;
    }

    try {
        const trajectory_tools::Trajectory trajectory = trajectory_tools::load_trajectory_csv(path);
        if (trajectory.timestamps != std::vector<double>{0.0} ||
            trajectory.positions != std::vector<std::vector<double>>{{1.0}}) {
            std::cerr << "Failure: CRLF CSV did not load the expected trajectory values\n";
            ++failures;
        }
    } catch (const std::exception& error) {
        std::cerr << "Failure: CRLF CSV threw an exception: " << error.what() << '\n';
        ++failures;
    }

    return failures + remove_test_csv(path);
}

int check_csv_error(const std::filesystem::path& path,
                    const std::string& contents,
                    const char* scenario) {
    int failures = remove_test_csv(path);
    if (failures != 0) {
        return failures;
    }

    if (!write_test_csv(path, contents)) {
        std::cerr << "Failure: could not create temporary CSV file " << path << '\n';
        return 1;
    }

    try {
        trajectory_tools::load_trajectory_csv(path);
        std::cerr << "Failure: " << scenario << " did not throw TrajectoryFileError\n";
        ++failures;
    } catch (const trajectory_tools::TrajectoryFileError&) {
    } catch (const std::exception& error) {
        std::cerr << "Failure: " << scenario << " threw an unexpected exception: " << error.what()
                  << '\n';
        ++failures;
    }

    return failures + remove_test_csv(path);
}

int check_missing_csv_error(const std::filesystem::path& path) {
    int failures = remove_test_csv(path);
    if (failures != 0) {
        return failures;
    }

    try {
        trajectory_tools::load_trajectory_csv(path);
        std::cerr << "Failure: missing CSV did not throw TrajectoryFileError\n";
        return 1;
    } catch (const trajectory_tools::TrajectoryFileError&) {
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Failure: missing CSV threw an unexpected exception: " << error.what() << '\n';
        return 1;
    }
}

}  // namespace

int main() {
    int failures = 0;

    failures += check_displacement({0.0, 0.4, -0.2, 0.6}, 0.6);
    failures += check_displacement({2.0, 1.0, 3.75, 0.25}, 1.75);
    failures += check_timestamp_lookup();
    failures += check_non_finite_timestamp_lookup(std::numeric_limits<double>::quiet_NaN());
    failures += check_non_finite_timestamp_lookup(std::numeric_limits<double>::infinity());
    failures += check_non_finite_timestamp_lookup(-std::numeric_limits<double>::infinity());

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
    failures += check_trajectory_summary(
        {{0.0, 0.5, 1.0}, {{1.0, 2.0}, {-3.0, 4.0}, {2.0, -1.0}}});
    failures += check_invalid_trajectory_summary(
        {{0.0, 0.5}, {{0.0, 0.0}, {std::numeric_limits<double>::quiet_NaN(), 0.0}}});
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

    failures += check_out_of_limit_rows(
        {{0.0, 0.5}, {{0.0, 1.0}, {-1.0, 1.5}}}, {{-1.0, 0.0}, {1.0, 2.0}}, {});
    failures += check_out_of_limit_rows(
        {{0.0, 0.5, 1.0, 1.5}, {{0.0, 0.0}, {-1.1, 0.0}, {0.0, 1.1}, {-1.1, 1.1}}},
        {{-1.0, -1.0}, {1.0, 1.0}},
        {1, 2, 3});
    failures += check_out_of_limit_rows(
        {{0.0, 0.5}, {{-1.0, 2.0}, {1.0, 0.0}}}, {{-1.0, 0.0}, {1.0, 2.0}}, {});
    failures += check_out_of_limit_rows(
        {{0.0, 0.5, 1.0},
         {{std::numeric_limits<double>::quiet_NaN(), 0.0},
          {std::numeric_limits<double>::infinity(), 0.0},
          {-std::numeric_limits<double>::infinity(), 0.0}}},
        {{-1.0, -1.0}, {1.0, 1.0}},
        {});
    failures += check_invalid_limits(
        {{0.0}, {{0.0, 0.0}}},
        {{-1.0}, {1.0, 1.0}},
        "lower limits must match the trajectory joint count");
    failures += check_invalid_limits(
        {{0.0}, {{0.0, 0.0}}},
        {{-1.0, -1.0}, {1.0}},
        "upper limits must match the trajectory joint count");
    failures += check_invalid_limits(
        {{0.0}, {{0.0, 0.0}}},
        {{0.0, 2.0}, {1.0, 1.0}},
        "lower joint limits must not exceed upper joint limits");

    failures += check_invalid_timestamp_rows({{0.0, 0.1, 0.2}, {{0.0}, {0.0}, {0.0}}}, {});
    failures += check_invalid_timestamp_rows(
        {{0.0, 0.1, 0.1, 0.05}, {{0.0}, {0.0}, {0.0}, {0.0}}},
        {2, 3});
    failures += check_invalid_timestamp_rows(
        {{0.0,
          std::numeric_limits<double>::infinity(),
          0.2,
          std::numeric_limits<double>::quiet_NaN()},
         {{0.0}, {0.0}, {0.0}, {0.0}}},
        {1, 3});
    failures += check_structurally_invalid_timestamp_trajectory({{}, {{0.0}}});

    failures += check_valid_trajectory_report(
        {{0.0, 0.1}, {{0.0, 0.0}, {0.5, -0.5}}}, {{-1.0, -1.0}, {1.0, 1.0}});
    failures += check_invalid_trajectory_report_rows(
        {{0.0, 0.1, 0.1, 0.05},
         {{0.0, 0.0},
          {std::numeric_limits<double>::quiet_NaN(), 0.0},
          {-1.1, 0.0},
          {0.0, 1.1}}},
        {{-1.0, -1.0}, {1.0, 1.0}});
    failures += check_invalid_trajectory_report({{}, {{0.0, 0.0}}}, {{-1.0, -1.0}, {1.0, 1.0}});

    const std::filesystem::path temporary_directory = std::filesystem::temp_directory_path();
    failures += check_valid_csv_load(temporary_directory / "trajectory_tools_valid.csv");
    failures += check_valid_crlf_csv_load(temporary_directory / "trajectory_tools_valid_crlf.csv");
    failures += check_missing_csv_error(temporary_directory / "trajectory_tools_missing.csv");
    failures += check_csv_error(
        temporary_directory / "trajectory_tools_non_numeric.csv",
        "timestamp,joint_0\n0.0,not-a-number\n",
        "non-numeric CSV field");
    failures += check_csv_error(
        temporary_directory / "trajectory_tools_partially_numeric_text.csv",
        "timestamp,joint_0\n0.1abc,0.0\n",
        "partially numeric text CSV field");
    failures += check_csv_error(
        temporary_directory / "trajectory_tools_leading_whitespace.csv",
        "timestamp,joint_0\n0.0, 1.0\n",
        "leading whitespace CSV field");
    failures += check_csv_error(
        temporary_directory / "trajectory_tools_trailing_whitespace.csv",
        "timestamp,joint_0\n0.0,1.0 \n",
        "trailing whitespace CSV field");
    failures += check_csv_error(
        temporary_directory / "trajectory_tools_inconsistent_columns.csv",
        "timestamp,joint_0,joint_1\n0.0,1.0\n",
        "inconsistent CSV row column count");
    failures += check_csv_error(
        temporary_directory / "trajectory_tools_invalid_header.csv",
        "time,joint_0\n0.0,1.0\n",
        "invalid CSV header");

    if (failures != 0) {
        return 1;
    }

    std::cout << "All trajectory_tools tests passed.\n";
    return 0;
}
