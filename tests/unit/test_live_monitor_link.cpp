// Unit tests for the vdsim_realtime browser monitor link (cosim/live_monitor_link.hpp):
// datagram encoders, the --monitor argument and the interpreter lookup. The child
// process and shutdown behaviour are covered end to end by
// tests/scripts/test_realtime_monitor.py.
#include <gtest/gtest.h>

#include "live_monitor_link.hpp"

#include <limits>
#include <string>
#include <vector>

using vdsim::cosim::monitor_find_executable;
using vdsim::cosim::monitor_port_from_args;
using vdsim::cosim::monitor_scene_json;
using vdsim::cosim::monitor_state_json;

namespace {
int port_of(std::vector<std::string> args) {
    std::vector<char*> v;
    for (auto& a : args) v.push_back(a.data());
    return monitor_port_from_args(static_cast<int>(v.size()), v.data());
}
}  // namespace

TEST(LiveMonitorLink, StateUsesThePageWireKeys) {
    vdsim::SimOutput o;
    o.state.position = vdsim::Vec3(1.5, -2.25, 0.5);
    o.state.velocity = vdsim::Vec3(12.0, 0.0, 0.0);
    o.sim_time = 3.0;
    o.steer_applied = 0.125;
    o.roll = 0.01;
    o.pitch = -0.02;
    const std::string j = monitor_state_json(o);
    EXPECT_EQ(j, "{\"state\":{\"x\":1.5,\"y\":-2.25,\"yaw\":0,\"t\":3,\"vx\":12,"
                 "\"delta\":0.125,\"roll\":0.01,\"pitch\":-0.02,\"z\":0.5}}");
}

TEST(LiveMonitorLink, NonFiniteNumbersBecomeNull) {
    vdsim::SimOutput o;
    o.roll = std::numeric_limits<double>::quiet_NaN();
    o.pitch = std::numeric_limits<double>::infinity();
    const std::string j = monitor_state_json(o);
    EXPECT_NE(j.find("\"roll\":null"), std::string::npos) << j;
    EXPECT_NE(j.find("\"pitch\":null"), std::string::npos) << j;
    EXPECT_EQ(j.find("nan"), std::string::npos) << j;
    EXPECT_EQ(j.find("inf"), std::string::npos) << j;
}

TEST(LiveMonitorLink, SceneCarriesTheDrawingGeometry) {
    vdsim::VehicleParams vp;
    vp.cg_to_front = 1.2;
    vp.cg_to_rear = 1.6;
    vp.track_front = 1.5;
    vp.track_rear = 1.7;
    vp.wheel_radius_nominal = 0.33;
    EXPECT_EQ(monitor_scene_json(vp),
              "{\"scene\":{\"vehicle\":{\"cg_to_front\":1.2,\"cg_to_rear\":1.6,"
              "\"track\":1.6,\"wheel_radius\":0.33}}}");
}

TEST(LiveMonitorLink, MonitorArgument) {
    EXPECT_EQ(port_of({"vdsim_realtime", "--scene=a.yaml"}), 0);                 // off by default
    EXPECT_EQ(port_of({"vdsim_realtime", "--monitor"}), vdsim::cosim::kMonitorDefaultPort);
    EXPECT_EQ(port_of({"vdsim_realtime", "--monitor=18790"}), 18790);
    EXPECT_EQ(port_of({"vdsim_realtime", "--monitor="}), -1);
    EXPECT_EQ(port_of({"vdsim_realtime", "--monitor=0"}), -1);
    EXPECT_EQ(port_of({"vdsim_realtime", "--monitor=65536"}), -1);
    EXPECT_EQ(port_of({"vdsim_realtime", "--monitor=87x"}), -1);
    // --monitor-python= is a different option, not a port.
    EXPECT_EQ(port_of({"vdsim_realtime", "--monitor-python=/usr/bin/python3"}), 0);
}

#ifndef _WIN32
TEST(LiveMonitorLink, InterpreterLookup) {
    EXPECT_FALSE(monitor_find_executable("sh").empty());            // on PATH
    EXPECT_EQ(monitor_find_executable("/bin/sh"), "/bin/sh");       // explicit path
    EXPECT_TRUE(monitor_find_executable("/nonexistent/python3").empty());
    EXPECT_TRUE(monitor_find_executable("vdsim-no-such-interpreter").empty());
    EXPECT_TRUE(monitor_find_executable("/tmp").empty());           // a directory is not runnable
}

TEST(LiveMonitorLink, StartRefusesAMissingInterpreter) {
    vdsim::cosim::MonitorLink link;
    vdsim::cosim::MonitorOptions opt;
    opt.port = 1;
    opt.python = "/nonexistent/python3";
    std::string why;
    EXPECT_FALSE(link.start(opt, vdsim::VehicleParams{}, &why));
    EXPECT_FALSE(link.active());
    EXPECT_NE(why.find("not found"), std::string::npos) << why;
}
#endif
