#include <gtest/gtest.h>

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

#include "json/json.hpp"
#include "pi_fr3_law.hpp"

namespace {

std::string packaged_path() {
    return std::string(OPENPI_CONTROL_SOURCE_DIR) +
           "/src/openpi_control/models/arms/FR3/FR3_law.json";
}

nlohmann::json packaged_json() {
    std::ifstream file(packaged_path());
    std::stringstream buffer;
    buffer << file.rdbuf();
    return nlohmann::json::parse(buffer.str());
}

// The message names the key, which is what a person editing the file needs.
void expect_rejected(const nlohmann::json& law, const std::string& key) {
    try {
        FR3Law::parse(law.dump(), "test");
        FAIL() << "accepted a law with a bad " << key;
    } catch (const std::runtime_error& error) {
        EXPECT_NE(std::string(error.what()).find(key), std::string::npos) << error.what();
    }
}

}  // namespace

TEST(FR3Law, LoadsThePackagedFile) {
    const FR3Law law = FR3Law::load(packaged_path());
    EXPECT_EQ(law.source, packaged_path());
    for (size_t i = 0; i < 7; ++i) {
        EXPECT_LT(law.limits.joint_lower[i], law.limits.joint_upper[i]);
        EXPECT_GT(law.limits.velocity[i], 0.0);
        EXPECT_GT(law.limits.torque[i], 0.0);
        EXPECT_GT(law.collision_torque_thresholds[i], 0.0);
    }
    EXPECT_GT(law.limits.elbow_velocity, 0.0);
    EXPECT_GT(law.torque_filter_cutoff_hz, 0.0);
    EXPECT_TRUE(law.torque_rate_limit);
}

TEST(FR3Law, RejectsAMalformedFile) {
    nlohmann::json missing = packaged_json();
    missing.erase("joint_stiffness");
    expect_rejected(missing, "joint_stiffness");

    // nlohmann would read the first seven of a longer array without a word.
    nlohmann::json too_long = packaged_json();
    too_long["joint_damping"] = {4, 6, 5, 5, 3, 2, 1, 1};
    expect_rejected(too_long, "joint_damping");

    nlohmann::json too_short = packaged_json();
    too_short["torque_limit"] = {86, 86, 86};
    expect_rejected(too_short, "torque_limit");

    nlohmann::json boolean_number = packaged_json();
    boolean_number["joint_margin"] = true;
    expect_rejected(boolean_number, "joint_margin");

    nlohmann::json unknown = packaged_json();
    unknown["joint_stiffnes"] = 1;
    expect_rejected(unknown, "joint_stiffnes");

    nlohmann::json inverted = packaged_json();
    inverted["joint_lower"][0] = 3.0;
    expect_rejected(inverted, "joint");

    EXPECT_THROW(FR3Law::load("/nonexistent/FR3_law.json"), std::runtime_error);
}
