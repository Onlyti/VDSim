// vdsim_realtime — real-time UDP co-simulation server.
//
// Runs a SimSession in real time (RealTimeRunner), receiving CMD packets and
// emitting STATE packets per docs/vdsim_bridge_interface_requirements.md. This
// is the real-vehicle-equivalent boundary: free-run, no shared clock, ZOH-
// latched commands, ECU-style fail-safe on command timeout.
//
// Usage:
//   vdsim_realtime <vehicle.yaml> <tire.yaml> [--level=L1|L2|L3]
//       [--cmd-port=7001] [--state-ip=127.0.0.1] [--state-port=7002]
//       [--rate=200] [--vx0=0] [--cmd-timeout=0.1]
#include "cosim_protocol.hpp"

#include "vdsim/coordinate.hpp"
#include "vdsim/interfaces.hpp"
#include "vdsim/realtime_runner.hpp"
#include "vdsim/sensors.hpp"
#include "vdsim/sim_session.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

namespace {
std::atomic<bool> g_run {true};
void on_sigint(int) { g_run.store(false); }

double optd(int c, char** v, const char* k, double d) {
    const size_t L = std::strlen(k);
    for (int i = 1; i < c; ++i) if (!std::strncmp(v[i], k, L)) return std::atof(v[i] + L);
    return d;
}
std::string opts(int c, char** v, const char* k, const std::string& d) {
    const size_t L = std::strlen(k);
    for (int i = 1; i < c; ++i) if (!std::strncmp(v[i], k, L)) return std::string(v[i] + L);
    return d;
}
std::unique_ptr<vdsim::IVehicleDynamics> make_dyn(const std::string& lvl) {
    if (lvl == "L1") return vdsim::create_bicycle();
    if (lvl == "L3") return vdsim::create_fourteen_dof();
    return vdsim::create_seven_dof();
}
double now_s() {
    return std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

// Select the contact provider from road config, mirroring make_sim_session
// (bindings.cpp). A terrain heightmap file (if given) wins; otherwise the
// analytic surface chosen by which params are set.
//
// Terrain file (little-endian binary): int32 nx, int32 ny, double x0, y0, dx,
// dy, then nx*ny doubles row-major h[iy*nx+ix]. (Written by the GUI bridge.)
std::unique_ptr<vdsim::IContactProvider> make_ground(
    const std::string& terrain, double mu, double mu_right, double mu_boundary,
    double grade, double bank, double rough_amp, double rough_wl, int iso_class) {
    if (!terrain.empty()) {
        std::ifstream f(terrain, std::ios::binary);
        int32_t nx = 0, ny = 0; double x0 = 0, y0 = 0, dx = 0, dy = 0;
        if (f && f.read(reinterpret_cast<char*>(&nx), 4) &&
            f.read(reinterpret_cast<char*>(&ny), 4) &&
            f.read(reinterpret_cast<char*>(&x0), 8) &&
            f.read(reinterpret_cast<char*>(&y0), 8) &&
            f.read(reinterpret_cast<char*>(&dx), 8) &&
            f.read(reinterpret_cast<char*>(&dy), 8) && nx > 0 && ny > 0) {
            std::vector<double> h(static_cast<size_t>(nx) * ny);
            if (f.read(reinterpret_cast<char*>(h.data()),
                       static_cast<std::streamsize>(h.size()) * 8))
                return vdsim::create_heightmap_ground(std::move(h), nx, ny, x0, y0, dx, dy, mu);
        }
        std::fprintf(stderr, "[vdsim_realtime] terrain load failed: %s (flat-ground)\n",
                     terrain.c_str());
    }
    if (iso_class >= 0) return vdsim::create_iso8608_ground(0.0, mu, iso_class, 1u);
    if (rough_amp > 0.0) return vdsim::create_rough_ground(0.0, mu, rough_amp, rough_wl);
    if (grade != 0.0 || bank != 0.0) return vdsim::create_inclined_ground(0.0, grade, bank, mu);
    if (mu_right >= 0.0) return vdsim::create_split_mu_ground(0.0, mu, mu_right, mu_boundary);
    return vdsim::create_flat_ground(0.0, mu);
}
}  // namespace

int main(int argc, char** argv) {
    if (argc < 3) {
        std::fprintf(stderr,
            "usage: %s <vehicle.yaml> <tire.yaml> [--level=L1|L2|L3] "
            "[--cmd-port=7001] [--state-ip=127.0.0.1] [--state-port=7002] "
            "[--rate=200] [--vx0=0] [--cmd-timeout=0.1] "
            "[--mu=1.0] [--mu-right=-1] [--mu-boundary=0] [--grade=0] [--bank=0] "
            "[--rough-amp=0] [--rough-wl=4] [--iso-class=-1] [--terrain=<file>] "
            "[--sensors=<yaml>] [--sensor-delay=0] [--x0=0] [--y0=0] [--yaw0=0]\n",
            argv[0]);
        return 2;
    }
    const auto vp = vdsim::VehicleParams::from_yaml(argv[1]);
    const auto tp = vdsim::TireParams::from_yaml(argv[2]);
    vdsim::SolverParams sp;

    const std::string level = opts(argc, argv, "--level=", "L2");
    const int cmd_port      = (int)optd(argc, argv, "--cmd-port=", 7001);
    const std::string st_ip = opts(argc, argv, "--state-ip=", "127.0.0.1");
    const int st_port       = (int)optd(argc, argv, "--state-port=", 7002);
    const double rate       = optd(argc, argv, "--rate=", 200.0);
    const double vx0        = optd(argc, argv, "--vx0=", 0.0);
    const double cmd_to     = optd(argc, argv, "--cmd-timeout=", 0.1);
    const double dt         = 1.0 / rate;

    // Road config (mirrors make_sim_session): terrain file wins, else analytic.
    const double mu          = optd(argc, argv, "--mu=", 1.0);
    const double mu_right    = optd(argc, argv, "--mu-right=", -1.0);
    const double mu_boundary = optd(argc, argv, "--mu-boundary=", 0.0);
    const double grade       = optd(argc, argv, "--grade=", 0.0);
    const double bank        = optd(argc, argv, "--bank=", 0.0);
    const double rough_amp   = optd(argc, argv, "--rough-amp=", 0.0);
    const double rough_wl    = optd(argc, argv, "--rough-wl=", 4.0);
    const int    iso_class   = (int)optd(argc, argv, "--iso-class=", -1.0);
    const std::string terrain = opts(argc, argv, "--terrain=", "");
    const std::string sensors_yaml = opts(argc, argv, "--sensors=", "");
    const double sensor_delay = optd(argc, argv, "--sensor-delay=", 0.0);
    const double x0   = optd(argc, argv, "--x0=", 0.0);
    const double y0   = optd(argc, argv, "--y0=", 0.0);
    const double yaw0 = optd(argc, argv, "--yaw0=", 0.0);

    vdsim::SimConfig cfg; cfg.nominal_dt = dt;
    if (!sensors_yaml.empty()) cfg.sensors = vdsim::SensorParams::from_yaml(sensors_yaml);
    cfg.sensor_delay_s = sensor_delay;
    vdsim::SimSession sim(make_dyn(level),
        make_ground(terrain, mu, mu_right, mu_boundary, grade, bank, rough_amp, rough_wl, iso_class),
        vp, tp, sp, cfg);
    vdsim::State s0;
    s0.position = vdsim::Vec3(x0, y0, 0.0);
    s0.orientation = vdsim::quat_from_euler(vdsim::Euler{0.0, 0.0, yaw0});
    s0.velocity.x() = vx0;
    const double w0 = (vp.wheel_radius_nominal > 0.0) ? vx0 / vp.wheel_radius_nominal : 0.0;
    s0.wheel_spin = {{w0, w0, w0, w0}};
    sim.reset(s0);

    // UDP socket: bind cmd_port for recv, sendto state dest.
    int sock = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) { std::perror("socket"); return 1; }
    sockaddr_in bind_addr{}; bind_addr.sin_family = AF_INET;
    bind_addr.sin_addr.s_addr = INADDR_ANY; bind_addr.sin_port = htons((uint16_t)cmd_port);
    if (::bind(sock, (sockaddr*)&bind_addr, sizeof(bind_addr)) < 0) {
        std::perror("bind"); ::close(sock); return 1;
    }
    // Recv timeout so the recv thread can observe shutdown and join cleanly.
    timeval rcvto{}; rcvto.tv_sec = 0; rcvto.tv_usec = 100000;  // 100 ms
    ::setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &rcvto, sizeof(rcvto));
    sockaddr_in dst{}; dst.sin_family = AF_INET; dst.sin_port = htons((uint16_t)st_port);
    ::inet_pton(AF_INET, st_ip.c_str(), &dst.sin_addr);

    std::signal(SIGINT, on_sigint);

    // RealTimeRunner paces the sim; fail-safe brake on command timeout.
    vdsim::RealTimeRunner::Config rc;
    rc.dt = dt; rc.cmd_timeout_s = cmd_to;
    rc.failsafe = vdsim::CmdL4{0.0, 0.3, 0.0, 1, false};
    vdsim::RealTimeRunner runner(sim, rc);

    // Recv thread: validate + latch incoming CMD, discard out-of-order seq.
    std::atomic<uint32_t> last_seq {0};
    std::atomic<uint64_t> cmd_count {0};
    std::thread recv_thread([&] {
        uint8_t buf[256];
        while (g_run.load()) {
            const ssize_t n = ::recv(sock, buf, sizeof(buf), 0);
            if (n <= 0) continue;
            vdsim::cosim::CmdFields f;
            if (!vdsim::cosim::decode_cmd(buf, (size_t)n, f)) continue;   // bad magic/crc
            if (f.seq != 0 && f.seq <= last_seq.load()) continue;        // stale
            last_seq.store(f.seq);
            vdsim::CmdL4 u; u.throttle = f.throttle; u.brake = f.brake;
            u.steer_angle_wheel = f.steer_tire; u.gear = f.gear;
            u.handbrake = (f.handbrake != 0);
            sim.set_input(u);
            cmd_count.fetch_add(1);
        }
    });

    runner.start();
    std::fprintf(stderr,
        "[vdsim_realtime] %s @ %.0f Hz | cmd :%d  state -> %s:%d | Ctrl-C to stop\n",
        level.c_str(), rate, cmd_port, st_ip.c_str(), st_port);

    // Main thread: emit STATE at the sim rate.
    uint32_t seq = 0;
    auto next = std::chrono::steady_clock::now();
    const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(dt));
    uint8_t out[vdsim::cosim::kStateBytes];
    while (g_run.load()) {
        const auto o = sim.output();
        vdsim::cosim::StateFields s;
        s.seq = seq++; s.timestamp = now_s();
        s.x = o.state.position.x(); s.y = o.state.position.y(); s.z = o.state.position.z();
        s.roll = o.roll; s.pitch = o.pitch; s.yaw = vdsim::yaw_from_quat(o.state.orientation);
        s.vx = o.state.velocity.x(); s.vy = o.state.velocity.y(); s.vz = o.state.velocity.z();
        s.roll_rate = o.state.angular_velocity.x();
        s.pitch_rate = o.state.angular_velocity.y();
        s.yaw_rate = o.state.angular_velocity.z();
        s.ax = o.ax; s.ay = o.ay;
        for (int i = 0; i < 4; ++i) {
            s.wheel_spin[i] = o.state.wheel_spin[i]; s.Fz[i] = o.Fz[i];
            s.slip_ratio[i] = o.slip_ratio[i]; s.slip_angle[i] = o.slip_angle[i];
            s.susp[i] = o.state.susp_compression[i];
            s.fx[i] = o.tire_forces[i].x(); s.fy[i] = o.tire_forces[i].y();
        }
        s.steer_applied = o.steer_applied; s.wheel_radius = vp.wheel_radius_nominal;
        s.rack_torque = o.rack_torque;
        s.m_ax = o.sensors.ax; s.m_ay = o.sensors.ay; s.m_wz = o.sensors.wz;
        s.m_steer = o.sensors.steer; s.m_gnss_x = o.sensors.gnss_x; s.m_gnss_y = o.sensors.gnss_y;
        const int len = vdsim::cosim::encode_state(out, s);
        ::sendto(sock, out, (size_t)len, 0, (sockaddr*)&dst, sizeof(dst));
        next += period;
        std::this_thread::sleep_until(next);
    }

    runner.stop();
    recv_thread.join();
    ::close(sock);
    std::fprintf(stderr, "[vdsim_realtime] stopped. sim_time=%.2fs, cmds=%llu\n",
                 sim.sim_time(), (unsigned long long)cmd_count.load());
    return 0;
}
