#pragma once

#include <array>
#include <string>

#include "pi_fr3_controller.hpp"

// The FR3 control law as one file, FR3_law.json in the openpi_control package.
//
// The node loads it at startup from --fr3_law, the path the Python layer
// resolves inside the package (or an explicit FR3Connection.law_path), and a
// simulation that mirrors the node reads the same file. Nothing about the law
// is compiled in, so a change to it moves the arm and its twin together and
// needs no rebuild.
struct FR3Law {
    FR3ControllerGains gains;
    FR3ControllerLimits limits;
    // libfranka's conditioning of the commanded torque: a first-order low-pass
    // at this cutoff (1000 Hz or more disables it) and, when enabled, its
    // fixed 1 Nm per millisecond rate limit.
    double torque_filter_cutoff_hz = 0.0;
    bool torque_rate_limit = true;
    // One reflex threshold per joint and per Cartesian axis, applied to both
    // the acceleration and the nominal phase and to both bounds.
    std::array<double, 7> collision_torque_thresholds{};
    std::array<double, 6> collision_force_thresholds{};
    // Where the values came from, for the startup log.
    std::string source;

    // Reads and validates a file. Every key is required, unknown keys are
    // rejected, arrays must have exactly their length, values must be finite
    // and every lower bound must sit below its upper bound. Throws
    // std::runtime_error naming the source and the offending key.
    static FR3Law load(const std::string& path);
    static FR3Law parse(const std::string& text, const std::string& source);
};
