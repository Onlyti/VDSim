// Work order 48c C3: VehicleParams::rack_per_rad (tie-rod rack travel per road-wheel
// rad, L4/L5 hardpoint paths). Default 0.08 m/rad, YAML/to_yaml round trip, invalid
// values rejected (std::invalid_argument -> Python ValueError).
#include "vdsim/interfaces.hpp"
#include "vdsim/params.hpp"

#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>

using namespace vdsim;

namespace {

const std::string kSrc = VDSIM_SOURCE_DIR;

std::string tmp_yaml(const std::string& tag) {
    return ::testing::TempDir() + "rack_per_rad_" + tag + ".yaml";
}

/** Copy of generic_sedan.yaml with one extra line appended. */
std::string sedan_with(const std::string& tag, const std::string& line) {
    std::ifstream in(kSrc + "/configs/vehicles/generic_sedan.yaml");
    const std::string path = tmp_yaml(tag);
    std::ofstream out(path);
    out << in.rdbuf() << "\n" << line << "\n";
    return path;
}

}  // namespace

TEST(RackPerRadKey, DefaultIsPointZeroEight) {
    EXPECT_EQ(VehicleParams{}.rack_per_rad, 0.08);
    const auto vp = VehicleParams::from_yaml(kSrc + "/configs/vehicles/generic_sedan.yaml");
    EXPECT_EQ(vp.rack_per_rad, 0.08);
}

TEST(RackPerRadKey, YamlRoundTrip) {
    const auto vp = VehicleParams::from_yaml(sedan_with("rt", "rack_per_rad: 0.05"));
    EXPECT_EQ(vp.rack_per_rad, 0.05);
    const std::string out = tmp_yaml("rt_out");
    vp.to_yaml(out);
    EXPECT_EQ(VehicleParams::from_yaml(out).rack_per_rad, 0.05);
    std::remove(out.c_str());
}

TEST(RackPerRadKey, InvalidYamlValuesThrow) {
    for (const char* v : {"0", "-0.08", ".nan", ".inf"}) {
        SCOPED_TRACE(v);
        const std::string p = sedan_with(std::string("bad") + (v[0] == '-' ? "neg" : v + 1),
                                         std::string("rack_per_rad: ") + v);
        EXPECT_THROW(VehicleParams::from_yaml(p), std::invalid_argument);
    }
}

TEST(RackPerRadKey, InvalidProgrammaticValueRejectedAtInitialize) {
    for (const char* level : {"L3", "L4", "L5"}) {
        SCOPED_TRACE(level);
        VehicleParams vp;
        vp.rack_per_rad = -1.0;
        auto dyn = create_dynamics_for_level(level);
        EXPECT_THROW(dyn->initialize(vp, TireSetup(TireParams{}), SolverParams{}),
                     std::invalid_argument);
        vp.rack_per_rad = std::numeric_limits<double>::quiet_NaN();
        EXPECT_THROW(dyn->initialize(vp, TireSetup(TireParams{}), SolverParams{}),
                     std::invalid_argument);
    }
}
