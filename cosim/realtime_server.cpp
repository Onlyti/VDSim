// vdsim_realtime — real-time UDP co-simulation server (single or multi-vehicle).
//
// Flat (N=1):  vdsim_realtime <vehicle.yaml> <tire.yaml> [options]
// World (N>1): vdsim_realtime --scenario=<world.yaml> [options]
#include "cosim_protocol.hpp"
#include "world_scenario.hpp"

#include "vdsim/coordinate.hpp"
#include "vdsim/interfaces.hpp"
#include "vdsim/realtime_runner.hpp"
#include "vdsim/sensors.hpp"
#include "vdsim/sim_session.hpp"

#ifdef _WIN32
  #include <winsock2.h>
  #include <ws2tcpip.h>
  using socket_t = SOCKET;
  static int close_socket(socket_t s) { return ::closesocket(s); }
#else
  #include <arpa/inet.h>
  #include <netinet/in.h>
  #include <sys/socket.h>
  #include <unistd.h>
  using socket_t = int;
  static constexpr socket_t INVALID_SOCKET = -1;
  static int close_socket(socket_t s) { return ::close(s); }
#endif

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>
#include <thread>
#include <unordered_map>
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

bool addr_same(const sockaddr_in& a, const sockaddr_in& b) {
    return a.sin_family == b.sin_family &&
           a.sin_addr.s_addr == b.sin_addr.s_addr &&
           a.sin_port == b.sin_port;
}

struct Subscriber {
    sockaddr_in addr {};
    double last_seen {0.0};
};

void fill_state(vdsim::cosim::StateFields& s, const vdsim::SimOutput& o,
                double wheel_radius, uint32_t vid) {
    s.vehicle_id = vid;
    s.x = o.state.position.x(); s.y = o.state.position.y(); s.z = o.state.position.z();
    s.roll = o.roll; s.pitch = o.pitch;
    s.yaw = vdsim::yaw_from_quat(o.state.orientation);
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
    s.steer_applied = o.steer_applied; s.wheel_radius = wheel_radius;
    s.rack_torque = o.rack_torque;
    s.m_ax = o.sensors.ax; s.m_ay = o.sensors.ay; s.m_wz = o.sensors.wz;
    s.m_steer = o.sensors.steer; s.m_gnss_x = o.sensors.gnss_x; s.m_gnss_y = o.sensors.gnss_y;
}

void touch_sub(std::vector<Subscriber>& subs, const sockaddr_in& addr, double t) {
    for (auto& s : subs) {
        if (addr_same(s.addr, addr)) { s.last_seen = t; return; }
    }
    subs.push_back({addr, t});
}

void prune_subs(std::vector<Subscriber>& subs, double t, double timeout) {
    subs.erase(std::remove_if(subs.begin(), subs.end(),
        [&](const Subscriber& s) { return (t - s.last_seen) > timeout; }),
        subs.end());
}

void broadcast_state(socket_t sock, const std::vector<Subscriber>& subs,
                     const uint8_t* buf, int len) {
    for (const auto& sub : subs) {
        ::sendto(sock, reinterpret_cast<const char*>(buf), len, 0,
                 reinterpret_cast<const sockaddr*>(&sub.addr),
                 static_cast<int>(sizeof(sub.addr)));
    }
}

#ifdef _WIN32
using socklen_t = int;
#endif

int run_flat(int argc, char** argv) {
    const auto vp = vdsim::VehicleParams::from_yaml(argv[1]);
    const auto tp = vdsim::TireParams::from_yaml(argv[2]);
    vdsim::SolverParams sp;

    const std::string level = opts(argc, argv, "--level=", "L2");
    const int cmd_port      = static_cast<int>(optd(argc, argv, "--cmd-port=", 7001));
    const std::string st_ip = opts(argc, argv, "--state-ip=", "127.0.0.1");
    const int st_port       = static_cast<int>(optd(argc, argv, "--state-port=", 7002));
    const double rate       = optd(argc, argv, "--rate=", 200.0);
    const double vx0        = optd(argc, argv, "--vx0=", 0.0);
    const double cmd_to     = optd(argc, argv, "--cmd-timeout=", 0.1);
    const double dt         = 1.0 / rate;

    const double mu          = optd(argc, argv, "--mu=", 1.0);
    const double mu_right    = optd(argc, argv, "--mu-right=", -1.0);
    const double mu_boundary = optd(argc, argv, "--mu-boundary=", 0.0);
    const double grade       = optd(argc, argv, "--grade=", 0.0);
    const double bank        = optd(argc, argv, "--bank=", 0.0);
    const double rough_amp   = optd(argc, argv, "--rough-amp=", 0.0);
    const double rough_wl    = optd(argc, argv, "--rough-wl=", 4.0);
    const int    iso_class   = static_cast<int>(optd(argc, argv, "--iso-class=", -1.0));
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

    socket_t sock = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sock == INVALID_SOCKET) { std::perror("socket"); return 1; }
    sockaddr_in bind_addr{}; bind_addr.sin_family = AF_INET;
    bind_addr.sin_addr.s_addr = INADDR_ANY; bind_addr.sin_port = htons(static_cast<uint16_t>(cmd_port));
    if (::bind(sock, reinterpret_cast<sockaddr*>(&bind_addr), sizeof(bind_addr)) < 0) {
        std::perror("bind"); close_socket(sock); return 1;
    }
#ifdef _WIN32
    DWORD rcvto = 100;
    ::setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<const char*>(&rcvto), sizeof(rcvto));
#else
    timeval rcvto{}; rcvto.tv_sec = 0; rcvto.tv_usec = 100000;
    ::setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &rcvto, sizeof(rcvto));
#endif

    std::vector<Subscriber> subscribers;
    sockaddr_in seed{}; seed.sin_family = AF_INET;
    seed.sin_port = htons(static_cast<uint16_t>(st_port));
    ::inet_pton(AF_INET, st_ip.c_str(), &seed.sin_addr);
    touch_sub(subscribers, seed, now_s());

    std::signal(SIGINT, on_sigint);

    vdsim::RealTimeRunner::Config rc;
    rc.dt = dt; rc.cmd_timeout_s = cmd_to;
    rc.failsafe = vdsim::CmdL4{0.0, 0.3, 0.0, 1, false};
    vdsim::RealTimeRunner runner(sim, rc);

    std::atomic<uint32_t> last_seq {0};
    std::atomic<uint64_t> cmd_count {0};
    std::thread recv_thread([&] {
        uint8_t buf[256];
        while (g_run.load()) {
            sockaddr_in from {}; socklen_t flen = sizeof(from);
            const int n = ::recvfrom(sock, reinterpret_cast<char*>(buf),
                                     static_cast<int>(sizeof(buf)), 0,
                                     reinterpret_cast<sockaddr*>(&from), &flen);
            if (n <= 0) continue;
            vdsim::cosim::CmdFields f;
            if (!vdsim::cosim::decode_cmd(buf, static_cast<size_t>(n), f)) continue;
            if (f.vehicle_id != 0) continue;
            if (f.seq != 0 && f.seq <= last_seq.load()) continue;
            last_seq.store(f.seq);
            vdsim::CmdL4 u; u.throttle = f.throttle; u.brake = f.brake;
            u.steer_angle_wheel = f.steer_tire; u.gear = f.gear;
            u.handbrake = (f.handbrake != 0);
            sim.set_input(u);
            cmd_count.fetch_add(1);
            touch_sub(subscribers, from, now_s());
        }
    });

    runner.start();
    std::fprintf(stderr,
        "[vdsim_realtime] flat %s @ %.0f Hz | cmd :%d | subscribers (incl. %s:%d)\n",
        level.c_str(), rate, cmd_port, st_ip.c_str(), st_port);

    uint32_t seq = 0;
    auto next = std::chrono::steady_clock::now();
    const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(dt));
    uint8_t out[vdsim::cosim::kStateBytes];
    while (g_run.load()) {
        const double t = now_s();
        touch_sub(subscribers, seed, t);   // keep --state-port subscriber alive
        prune_subs(subscribers, t, 2.0);
        const auto o = sim.output();
        vdsim::cosim::StateFields s;
        s.seq = seq++; s.timestamp = t;
        fill_state(s, o, vp.wheel_radius_nominal, 0);
        const int len = vdsim::cosim::encode_state(out, s);
        broadcast_state(sock, subscribers, out, len);
        next += period;
        std::this_thread::sleep_until(next);
    }

    runner.stop();
    recv_thread.join();
    close_socket(sock);
    std::fprintf(stderr, "[vdsim_realtime] stopped. sim_time=%.2fs, cmds=%llu\n",
                 sim.sim_time(), static_cast<unsigned long long>(cmd_count.load()));
    return 0;
}

struct WorldVehicle {
    uint32_t id;
    vdsim::VehicleParams vp;
    std::unique_ptr<vdsim::SimSession> sim;
    uint32_t last_seq {0};
    std::chrono::steady_clock::time_point last_cmd {std::chrono::steady_clock::now()};
};

int run_world(const std::string& scenario_path, int argc, char** argv) {
    vdsim::cosim::WorldScenario world;
    try {
        world = vdsim::cosim::load_world_scenario(scenario_path);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "[vdsim_realtime] scenario: %s\n", e.what());
        return 1;
    }

    const int cmd_port = static_cast<int>(optd(argc, argv, "--cmd-port=", 7001));
    const std::string st_ip = opts(argc, argv, "--state-ip=", "127.0.0.1");
    const int st_port = static_cast<int>(optd(argc, argv, "--state-port=", 7002));
    const double rate = world.rate > 1.0 ? world.rate : optd(argc, argv, "--rate=", 200.0);
    const double cmd_to = world.cmd_timeout > 0.0 ? world.cmd_timeout
                                                  : optd(argc, argv, "--cmd-timeout=", 0.1);
    const double dt = 1.0 / rate;
    const auto& rd = world.road;

    vdsim::SolverParams sp;
    vdsim::SimConfig cfg; cfg.nominal_dt = dt;
    if (!rd.sensors.empty()) cfg.sensors = vdsim::SensorParams::from_yaml(rd.sensors);
    cfg.sensor_delay_s = rd.sensor_delay;

    std::vector<WorldVehicle> fleet;
    std::unordered_map<uint32_t, WorldVehicle*> by_id;
    for (const auto& spn : world.vehicles) {
        WorldVehicle wv;
        wv.id = spn.id;
        wv.vp = vdsim::VehicleParams::from_yaml(spn.vehicle_yaml);
        const auto tp = vdsim::TireParams::from_yaml(spn.tire_yaml);
        wv.sim = std::make_unique<vdsim::SimSession>(
            make_dyn(spn.level),
            make_ground(rd.terrain, rd.mu, rd.mu_right, rd.mu_boundary,
                        rd.grade, rd.bank, rd.rough_amp, rd.rough_wl, rd.iso_class),
            wv.vp, tp, sp, cfg);
        vdsim::State s0;
        s0.position = vdsim::Vec3(spn.x0, spn.y0, 0.0);
        s0.orientation = vdsim::quat_from_euler(vdsim::Euler{0.0, 0.0, spn.yaw0});
        s0.velocity.x() = spn.vx0;
        const double w0 = (wv.vp.wheel_radius_nominal > 0.0)
            ? spn.vx0 / wv.vp.wheel_radius_nominal : 0.0;
        s0.wheel_spin = {{w0, w0, w0, w0}};
        wv.sim->reset(s0);
        fleet.push_back(std::move(wv));
    }
    for (auto& wv : fleet)
        by_id[wv.id] = &wv;

    socket_t sock = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sock == INVALID_SOCKET) { std::perror("socket"); return 1; }
    sockaddr_in bind_addr{}; bind_addr.sin_family = AF_INET;
    bind_addr.sin_addr.s_addr = INADDR_ANY;
    bind_addr.sin_port = htons(static_cast<uint16_t>(cmd_port));
    if (::bind(sock, reinterpret_cast<sockaddr*>(&bind_addr), sizeof(bind_addr)) < 0) {
        std::perror("bind"); close_socket(sock); return 1;
    }
#ifdef _WIN32
    DWORD rcvto = 100;
    ::setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<const char*>(&rcvto), sizeof(rcvto));
#else
    timeval rcvto{}; rcvto.tv_sec = 0; rcvto.tv_usec = 100000;
    ::setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &rcvto, sizeof(rcvto));
#endif

    std::vector<Subscriber> subscribers;
    sockaddr_in seed{}; seed.sin_family = AF_INET;
    seed.sin_port = htons(static_cast<uint16_t>(st_port));
    ::inet_pton(AF_INET, st_ip.c_str(), &seed.sin_addr);
    touch_sub(subscribers, seed, now_s());

    std::signal(SIGINT, on_sigint);
    const vdsim::CmdL4 failsafe{0.0, 0.3, 0.0, 1, false};
    std::atomic<uint64_t> cmd_count {0};

    std::thread recv_thread([&] {
        uint8_t buf[256];
        while (g_run.load()) {
            sockaddr_in from {}; socklen_t flen = sizeof(from);
            const int n = ::recvfrom(sock, reinterpret_cast<char*>(buf),
                                     static_cast<int>(sizeof(buf)), 0,
                                     reinterpret_cast<sockaddr*>(&from), &flen);
            if (n <= 0) continue;
            vdsim::cosim::CmdFields f;
            if (!vdsim::cosim::decode_cmd(buf, static_cast<size_t>(n), f)) continue;
            auto it = by_id.find(f.vehicle_id);
            if (it == by_id.end()) continue;
            WorldVehicle& wv = *it->second;
            if (f.seq != 0 && f.seq <= wv.last_seq) continue;
            wv.last_seq = f.seq;
            vdsim::CmdL4 u; u.throttle = f.throttle; u.brake = f.brake;
            u.steer_angle_wheel = f.steer_tire; u.gear = f.gear;
            u.handbrake = (f.handbrake != 0);
            wv.sim->set_input(u);
            wv.last_cmd = std::chrono::steady_clock::now();
            cmd_count.fetch_add(1);
            touch_sub(subscribers, from, now_s());
        }
    });

    std::fprintf(stderr,
        "[vdsim_realtime] world %zu vehicles @ %.0f Hz | cmd :%d | %s\n",
        fleet.size(), rate, cmd_port, scenario_path.c_str());

    uint32_t seq = 0;
    auto next = std::chrono::steady_clock::now();
    const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(dt));
    const auto failsafe_dur = std::chrono::duration<double>(cmd_to);
    uint8_t out[vdsim::cosim::kStateBytes];
    while (g_run.load()) {
        const double t = now_s();
        touch_sub(subscribers, seed, t);   // keep --state-port subscriber alive
        prune_subs(subscribers, t, 2.0);
        const auto now_tp = std::chrono::steady_clock::now();
        for (auto& wv : fleet) {
            if (cmd_to > 0.0 &&
                std::chrono::duration<double>(now_tp - wv.last_cmd) > failsafe_dur) {
                wv.sim->set_input(failsafe);
            }
            wv.sim->tick(dt);
        }
        for (const auto& wv : fleet) {
            const auto o = wv.sim->output();
            vdsim::cosim::StateFields s;
            s.seq = seq++; s.timestamp = t;
            fill_state(s, o, wv.vp.wheel_radius_nominal, wv.id);
            const int len = vdsim::cosim::encode_state(out, s);
            broadcast_state(sock, subscribers, out, len);
        }
        next += period;
        std::this_thread::sleep_until(next);
    }

    recv_thread.join();
    close_socket(sock);
    std::fprintf(stderr, "[vdsim_realtime] stopped. cmds=%llu\n",
                 static_cast<unsigned long long>(cmd_count.load()));
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    const std::string scenario = opts(argc, argv, "--scenario=", "");
#ifdef _WIN32
    WSADATA wsa;
    if (::WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
        std::fprintf(stderr, "WSAStartup failed\n"); return 1;
    }
#endif
    int rc;
    if (!scenario.empty()) {
        rc = run_world(scenario, argc, argv);
    } else if (argc >= 3) {
        rc = run_flat(argc, argv);
    } else {
        std::fprintf(stderr,
            "usage: %s <vehicle.yaml> <tire.yaml> [options]\n"
            "   or: %s --scenario=<world.yaml> [options]\n"
            "  [--cmd-port=7001] [--state-ip=127.0.0.1] [--state-port=7002]\n"
            "  [--rate=200] [--vx0=0] [--cmd-timeout=0.1] [--mu=1] [--grade=0] ...\n",
            argv[0], argv[0]);
        rc = 2;
    }
#ifdef _WIN32
    ::WSACleanup();
#endif
    return rc;
}
