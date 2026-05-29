#include "vdsim/scenario.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>

namespace fs = std::filesystem;

namespace {

fs::path tmp_path(const char* leaf) {
    static int counter = 0;
    return fs::temp_directory_path() /
           ("vdsim_scen_" + std::to_string(::getpid()) + "_" +
            std::to_string(counter++) + "_" + leaf);
}

void write_file(const fs::path& path, const std::string& content) {
    std::ofstream ofs(path); ofs << content;
}

}  // namespace

TEST(ScenarioYaml, RoundtripDefaults) {
    vdsim::Scenario a;
    a.name = "default_test";
    a.controls.push_back({0.0, 0.0, 0.0, 0.0, 1});
    const auto p = tmp_path("default.yaml");
    a.to_yaml(p.string());
    const auto b = vdsim::Scenario::from_yaml(p.string());
    EXPECT_EQ(a.name, b.name);
    EXPECT_DOUBLE_EQ(a.initial_vx, b.initial_vx);
    EXPECT_DOUBLE_EQ(a.duration, b.duration);
    EXPECT_DOUBLE_EQ(a.dt, b.dt);
    EXPECT_DOUBLE_EQ(a.mu, b.mu);
    EXPECT_EQ(a.interpolation, b.interpolation);
    ASSERT_EQ(a.controls.size(), b.controls.size());
    fs::remove(p);
}

TEST(ScenarioYaml, ZohSample) {
    vdsim::Scenario s;
    s.interpolation = vdsim::Scenario::Interp::ZOH;
    s.controls = {
        {0.0, 0.0, 0.0, 0.00, 1},
        {1.0, 0.5, 0.0, 0.05, 1},
        {2.0, 0.0, 0.8, 0.00, 1},
    };
    EXPECT_DOUBLE_EQ(s.sample(0.5).throttle, 0.0);
    EXPECT_DOUBLE_EQ(s.sample(1.0).throttle, 0.5);
    EXPECT_DOUBLE_EQ(s.sample(1.5).throttle, 0.5);
    EXPECT_DOUBLE_EQ(s.sample(1.999).steer, 0.05);
    EXPECT_DOUBLE_EQ(s.sample(2.5).brake, 0.8);
    EXPECT_DOUBLE_EQ(s.sample(-1.0).throttle, 0.0);
}

TEST(ScenarioYaml, LinearSample) {
    vdsim::Scenario s;
    s.interpolation = vdsim::Scenario::Interp::Linear;
    s.controls = {
        {0.0, 0.0, 0.0, 0.0, 1},
        {1.0, 1.0, 0.0, 0.1, 1},
    };
    EXPECT_DOUBLE_EQ(s.sample(0.5).throttle, 0.5);
    EXPECT_NEAR(s.sample(0.5).steer, 0.05, 1e-12);
    EXPECT_DOUBLE_EQ(s.sample(1.5).throttle, 1.0);   // saturated
}

TEST(ScenarioYaml, UnsortedControlsThrows) {
    const auto p = tmp_path("unsorted.yaml");
    write_file(p,
        "controls:\n"
        "  - { t: 0.0, steer: 0.0 }\n"
        "  - { t: 2.0, steer: 0.0 }\n"
        "  - { t: 1.0, steer: 0.1 }\n");
    EXPECT_THROW(vdsim::Scenario::from_yaml(p.string()), std::runtime_error);
    fs::remove(p);
}

TEST(ScenarioYaml, ClampsThrottleBrakeOnLoad) {
    const auto p = tmp_path("clamp.yaml");
    write_file(p,
        "controls:\n"
        "  - { t: 0.0, throttle: 2.0, brake: -0.5, steer: 0.0 }\n");
    const auto s = vdsim::Scenario::from_yaml(p.string());
    EXPECT_DOUBLE_EQ(s.controls.front().throttle, 1.0);
    EXPECT_DOUBLE_EQ(s.controls.front().brake,    0.0);
    fs::remove(p);
}

TEST(ScenarioYaml, MissingFileThrows) {
    EXPECT_THROW(vdsim::Scenario::from_yaml("/no/such/path.yaml"),
                 std::runtime_error);
}

TEST(ScenarioYaml, BadInterpThrows) {
    const auto p = tmp_path("bad_interp.yaml");
    write_file(p, "interpolation: cubic\n");
    EXPECT_THROW(vdsim::Scenario::from_yaml(p.string()), std::runtime_error);
    fs::remove(p);
}

TEST(ScenarioYaml, NegativeDurationThrows) {
    const auto p = tmp_path("neg_dur.yaml");
    write_file(p, "duration: -1.0\n");
    EXPECT_THROW(vdsim::Scenario::from_yaml(p.string()), std::runtime_error);
    fs::remove(p);
}

TEST(ScenarioYaml, EmptyControlsBecomesZero) {
    const auto p = tmp_path("empty.yaml");
    write_file(p, "duration: 1.0\ndt: 0.01\n");
    const auto s = vdsim::Scenario::from_yaml(p.string());
    ASSERT_EQ(s.controls.size(), 1u);
    EXPECT_DOUBLE_EQ(s.controls.front().throttle, 0.0);
    fs::remove(p);
}
