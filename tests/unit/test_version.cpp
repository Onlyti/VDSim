#include "vdsim/version.hpp"

#include <gtest/gtest.h>

TEST(VersionTest, MajorMinorPatch) {
    EXPECT_EQ(vdsim::VERSION_MAJOR, 0);
    EXPECT_EQ(vdsim::VERSION_MINOR, 1);
    EXPECT_EQ(vdsim::VERSION_PATCH, 0);
}

TEST(VersionTest, StringFormat) {
    EXPECT_EQ(vdsim::version_string(), "0.1.0");
}
