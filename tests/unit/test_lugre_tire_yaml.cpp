#include "vdsim/params.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>

namespace fs = std::filesystem;

namespace {

fs::path tmp_path(const char* name) {
    return fs::temp_directory_path() / name;
}

void write_file(const fs::path& path, const std::string& content) {
    std::ofstream ofs(path);
    ASSERT_TRUE(static_cast<bool>(ofs));
    ofs << content;
}

}  // namespace

TEST(TireYaml, LuGreRoundtrip) {
    vdsim::TireParams a;
    a.lugre.enabled = true;
    a.lugre.sigma0  = 180000.0;
    a.lugre.sigma1  = 4000.0;
    a.lugre.sigma2  = 60.0;
    a.lugre.m_eff   = 35.0;

    const auto path = tmp_path("tire_lugre.yaml");
    a.to_yaml(path.string());
    const auto b = vdsim::TireParams::from_yaml(path.string());

    EXPECT_EQ(a.lugre.enabled, b.lugre.enabled);
    EXPECT_NEAR(a.lugre.sigma0, b.lugre.sigma0, 1e-6);
    EXPECT_NEAR(a.lugre.sigma1, b.lugre.sigma1, 1e-6);
    EXPECT_NEAR(a.lugre.sigma2, b.lugre.sigma2, 1e-6);
    EXPECT_NEAR(a.lugre.m_eff,  b.lugre.m_eff,  1e-6);
    fs::remove(path);
}
