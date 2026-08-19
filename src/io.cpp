#include "trajectory_tools/io.hpp"

#include <fstream>
#include <string>
#include <utility>
#include <vector>

namespace trajectory_tools {
namespace {

TrajectoryFileError csv_error(const std::filesystem::path& path, const std::string& message) {
    return TrajectoryFileError("CSV trajectory file '" + path.string() + "': " + message);
}

void remove_trailing_carriage_return(std::string& line) {
    if (!line.empty() && line.back() == '\r') {
        line.pop_back();
    }
}

std::vector<std::string> split_csv_row(const std::string& line) {
    std::vector<std::string> fields;
    std::size_t field_start = 0;

    while (true) {
        const std::size_t comma = line.find(',', field_start);
        if (comma == std::string::npos) {
            fields.push_back(line.substr(field_start));
            return fields;
        }

        fields.push_back(line.substr(field_start, comma - field_start));
        field_start = comma + 1;
    }
}

double parse_numeric_field(const std::string& field,
                           const std::filesystem::path& path,
                           std::size_t row_number,
                           std::size_t column_number) {
    try {
        std::size_t parsed_characters = 0;
        const double value = std::stod(field, &parsed_characters);
        if (parsed_characters != field.size()) {
            throw csv_error(path,
                            "row " + std::to_string(row_number) + ", column " +
                                std::to_string(column_number) + " is not a complete numeric value");
        }
        return value;
    } catch (const std::invalid_argument&) {
        throw csv_error(path,
                        "row " + std::to_string(row_number) + ", column " +
                            std::to_string(column_number) + " is not numeric");
    } catch (const std::out_of_range&) {
        throw csv_error(path,
                        "row " + std::to_string(row_number) + ", column " +
                            std::to_string(column_number) + " is out of range");
    }
}

}  // namespace

Trajectory load_trajectory_csv(const std::filesystem::path& path) {
    std::ifstream input(path);
    if (!input) {
        throw csv_error(path, "could not open file");
    }

    std::string header_line;
    if (!std::getline(input, header_line)) {
        throw csv_error(path, "header must not be empty");
    }
    remove_trailing_carriage_return(header_line);
    if (header_line.empty()) {
        throw csv_error(path, "header must not be empty");
    }

    const std::vector<std::string> header = split_csv_row(header_line);
    if (header.front() != "timestamp") {
        throw csv_error(path, "first header field must be timestamp");
    }
    if (header.size() < 2) {
        throw csv_error(path, "header must contain at least one joint column");
    }
    for (std::size_t column_index = 1; column_index < header.size(); ++column_index) {
        const std::string expected_name = "joint_" + std::to_string(column_index - 1);
        if (header[column_index] != expected_name) {
            throw csv_error(path, "joint columns must be named joint_0, joint_1, and so on");
        }
    }

    Trajectory trajectory;
    std::string data_line;
    std::size_t row_number = 2;
    while (std::getline(input, data_line)) {
        remove_trailing_carriage_return(data_line);
        const std::vector<std::string> fields = split_csv_row(data_line);
        if (fields.size() != header.size()) {
            throw csv_error(path,
                            "row " + std::to_string(row_number) +
                                " does not have the same number of fields as the header");
        }

        trajectory.timestamps.push_back(parse_numeric_field(fields[0], path, row_number, 0));
        std::vector<double> position_row;
        position_row.reserve(header.size() - 1);
        for (std::size_t column_index = 1; column_index < fields.size(); ++column_index) {
            position_row.push_back(
                parse_numeric_field(fields[column_index], path, row_number, column_index));
        }
        trajectory.positions.push_back(std::move(position_row));
        ++row_number;
    }

    if (trajectory.timestamps.empty()) {
        throw csv_error(path, "CSV must contain at least one data row");
    }

    return trajectory;
}

}  // namespace trajectory_tools
