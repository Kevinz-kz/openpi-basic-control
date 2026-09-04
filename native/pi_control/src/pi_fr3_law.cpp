#include "pi_fr3_law.hpp"

#include <cmath>
#include <fstream>
#include <initializer_list>
#include <set>
#include <sstream>
#include <stdexcept>

#include "json/json.hpp"

namespace {

using json = nlohmann::json;

[[noreturn]] void fail(const std::string& source, const std::string& what) {
    throw std::runtime_error("FR3 law " + source + ": " + what);
}

const json& field(const json& root, const char* key, const std::string& source) {
    const auto it = root.find(key);
    if (it == root.end()) fail(source, std::string("missing ") + key);
    return *it;
}

// is_number() is false for booleans, so `true` cannot pass as 1.
double finite_number(const json& value, const std::string& key, const std::string& source) {
    if (!value.is_number()) fail(source, key + " must be a number");
    const double number = value.get<double>();
    if (!std::isfinite(number)) fail(source, key + " must be finite");
    return number;
}

double number(const json& root, const char* key, const std::string& source) {
    return finite_number(field(root, key, source), key, source);
}

bool boolean(const json& root, const char* key, const std::string& source) {
    const json& value = field(root, key, source);
    if (!value.is_boolean()) fail(source, std::string(key) + " must be true or false");
    return value.get<bool>();
}

// nlohmann's own std::array conversion reads the first N items of a longer
// array without complaint, so the length is checked here first.
template <size_t N>
std::array<double, N> numbers(const json& root, const char* key, const std::string& source) {
    const json& value = field(root, key, source);
    if (!value.is_array() || value.size() != N) {
        fail(source, std::string(key) + " must be an array of " + std::to_string(N) + " numbers");
    }
    std::array<double, N> result{};
    for (size_t i = 0; i < N; ++i) {
        result[i] = finite_number(value[i], key, source);
    }
    return result;
}

template <size_t N>
void require_positive(const std::array<double, N>& values, const char* key,
                      const std::string& source) {
    for (const double value : values) {
        if (!(value > 0.0)) fail(source, std::string(key) + " must be positive");
    }
}

template <size_t N>
void require_ordered(const std::array<double, N>& lower, const std::array<double, N>& upper,
                     const char* key, const std::string& source) {
    for (size_t i = 0; i < N; ++i) {
        if (!(lower[i] < upper[i])) fail(source, std::string(key) + " lower must be below upper");
    }
}

const std::set<std::string>& known_keys() {
    static const std::set<std::string> keys = {
        "schema",
        "joint_stiffness",
        "joint_damping",
        "cartesian_stiffness",
        "cartesian_damping",
        "joint_lower",
        "joint_upper",
        "joint_margin",
        "joint_limit_stiffness",
        "velocity_limit",
        "velocity_margin",
        "velocity_limit_stiffness",
        "elbow_velocity_limit",
        "cartesian_lower",
        "cartesian_upper",
        "cartesian_margin",
        "cartesian_limit_stiffness",
        "torque_limit",
        "torque_filter_cutoff_hz",
        "torque_rate_limit",
        "collision_torque_thresholds",
        "collision_force_thresholds",
    };
    return keys;
}

}  // namespace

FR3Law FR3Law::parse(const std::string& text, const std::string& source) {
    json root;
    try {
        root = json::parse(text);
    } catch (const json::exception& error) {
        fail(source, error.what());
    }
    if (!root.is_object()) fail(source, "must be a JSON object");
    for (const auto& item : root.items()) {
        if (known_keys().count(item.key()) == 0) fail(source, "unknown key " + item.key());
    }
    if (number(root, "schema", source) != 1.0) fail(source, "schema must be 1");

    FR3Law law;
    law.source = source;
    law.gains.joint_stiffness = numbers<7>(root, "joint_stiffness", source);
    law.gains.joint_damping = numbers<7>(root, "joint_damping", source);
    law.gains.cartesian_stiffness = numbers<6>(root, "cartesian_stiffness", source);
    law.gains.cartesian_damping = numbers<6>(root, "cartesian_damping", source);
    law.limits.joint_lower = numbers<7>(root, "joint_lower", source);
    law.limits.joint_upper = numbers<7>(root, "joint_upper", source);
    law.limits.joint_margin = number(root, "joint_margin", source);
    law.limits.joint_stiffness = number(root, "joint_limit_stiffness", source);
    law.limits.velocity = numbers<7>(root, "velocity_limit", source);
    law.limits.velocity_margin = number(root, "velocity_margin", source);
    law.limits.velocity_stiffness = number(root, "velocity_limit_stiffness", source);
    law.limits.elbow_velocity = number(root, "elbow_velocity_limit", source);
    law.limits.cartesian_lower = numbers<3>(root, "cartesian_lower", source);
    law.limits.cartesian_upper = numbers<3>(root, "cartesian_upper", source);
    law.limits.cartesian_margin = number(root, "cartesian_margin", source);
    law.limits.cartesian_stiffness = number(root, "cartesian_limit_stiffness", source);
    law.limits.torque = numbers<7>(root, "torque_limit", source);
    law.torque_filter_cutoff_hz = number(root, "torque_filter_cutoff_hz", source);
    law.torque_rate_limit = boolean(root, "torque_rate_limit", source);
    law.collision_torque_thresholds = numbers<7>(root, "collision_torque_thresholds", source);
    law.collision_force_thresholds = numbers<6>(root, "collision_force_thresholds", source);

    require_ordered(law.limits.joint_lower, law.limits.joint_upper, "joint", source);
    require_ordered(law.limits.cartesian_lower, law.limits.cartesian_upper, "cartesian", source);
    require_positive(law.limits.velocity, "velocity_limit", source);
    require_positive(law.limits.torque, "torque_limit", source);
    require_positive(law.collision_torque_thresholds, "collision_torque_thresholds", source);
    require_positive(law.collision_force_thresholds, "collision_force_thresholds", source);
    if (!(law.limits.elbow_velocity > 0.0)) fail(source, "elbow_velocity_limit must be positive");
    if (!(law.torque_filter_cutoff_hz > 0.0)) {
        fail(source, "torque_filter_cutoff_hz must be positive");
    }
    for (const double margin :
         {law.limits.joint_margin, law.limits.velocity_margin, law.limits.cartesian_margin}) {
        if (margin < 0.0) fail(source, "margins must not be negative");
    }
    return law;
}

FR3Law FR3Law::load(const std::string& path) {
    std::ifstream file(path);
    if (!file) fail(path, "cannot open");
    std::stringstream buffer;
    buffer << file.rdbuf();
    return parse(buffer.str(), path);
}
