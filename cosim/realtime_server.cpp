// vdsim_realtime — real-time UDP co-simulation server (catalog scene or world YAML).
//
//   vdsim_realtime --scene=<scene.yaml|world.yaml> [options]
#include "cosim_protocol.hpp"
#include "scene_loader.hpp"

#include "vdsim/coordinate.hpp"
#include "vdsim/interfaces.hpp"
#include "vdsim/params.hpp"
#include "vdsim/sensors.hpp"
#include "vdsim/sim_session.hpp"
#include "vdsim/suspension.hpp"

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
#include <optional>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <filesystem>
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

double clamp_time_scale(double v) {
    if (v < 0.05) return 0.05;
    if (v > 10.0) return 10.0;
    return v;
}

double read_live_time_scale(const std::string& scenario_path, double fallback) {
    const auto p = std::filesystem::path(scenario_path).parent_path() / "time_scale";
    std::ifstream f(p);
    if (!f) return clamp_time_scale(fallback);
    double v = fallback;
    f >> v;
    return clamp_time_scale(v);
}
std::string opts(int c, char** v, const char* k, const std::string& d) {
    const size_t L = std::strlen(k);
    for (int i = 1; i < c; ++i) if (!std::strncmp(v[i], k, L)) return std::string(v[i] + L);
    return d;
}

static bool parse_on_off(const char* val) {
    if (!val || !*val) return true;
    if (!std::strcmp(val, "1") || !std::strcmp(val, "on") || !std::strcmp(val, "true")) return true;
    if (!std::strcmp(val, "0") || !std::strcmp(val, "off") || !std::strcmp(val, "false")) return false;
    return std::atof(val) > 0.5;
}

std::optional<bool> lugre_cli_override(int c, char** v) {
    for (int i = 1; i < c; ++i) {
        if (!std::strcmp(v[i], "--lugre")) return true;
        if (!std::strcmp(v[i], "--no-lugre")) return false;
        if (!std::strncmp(v[i], "--lugre=", 8)) return parse_on_off(v[i] + 8);
    }
    return std::nullopt;
}

void apply_lugre_cli(vdsim::TireParams& tp, int c, char** v) {
    if (const auto ov = lugre_cli_override(c, v))
        tp.lugre.enabled = *ov;
}
std::unique_ptr<vdsim::IVehicleDynamics> make_dyn(const std::string& lvl) {
    if (lvl == "L1") return vdsim::create_bicycle();
    if (lvl == "L3") return vdsim::create_fourteen_dof();
    if (lvl == "L4") return vdsim::create_fourteen_dof_kinematic();
    if (lvl == "L5") return vdsim::create_stunt_dof();
    return vdsim::create_seven_dof();
}
std::string resolve_susp_yaml(std::string ref) {
    if (ref.empty()) return ref;
    {
        std::ifstream f(ref);
        if (f.good()) return ref;
    }
    const auto slash = ref.find_last_of("/\\");
    const std::string stem = (slash == std::string::npos) ? ref : ref.substr(slash + 1);
    const auto dot = stem.find_last_of('.');
    const std::string base = (dot == std::string::npos) ? stem : stem.substr(0, dot);
    const std::string candidate = "configs/parts/susp_kinematics/kin/" + base + ".yaml";
    std::ifstream f2(candidate);
    if (f2.good()) return candidate;
    return ref;
}
void attach_susp_parts(vdsim::IVehicleDynamics& dyn, const std::string& lvl,
                       const std::string& front_yaml, const std::string& rear_yaml) {
    if (lvl != "L3" && lvl != "L4") return;
    const std::string front = resolve_susp_yaml(front_yaml);
    const std::string rear  = resolve_susp_yaml(rear_yaml);
    if (!front.empty()) {
        try {
            auto k = vdsim::create_native_kinematics_from_yaml(front);
            if (!vdsim::attach_front_kinematics(dyn, std::move(k)))
                std::fprintf(stderr, "[vdsim_realtime] front kinematics attach failed\n");
        } catch (const std::exception& e) {
            std::fprintf(stderr, "[vdsim_realtime] front susp %s: %s\n",
                         front.c_str(), e.what());
        }
    }
    if (!rear.empty()) {
        try {
            auto k = vdsim::create_native_kinematics_from_yaml(rear);
            if (!vdsim::attach_rear_kinematics(dyn, std::move(k)))
                std::fprintf(stderr, "[vdsim_realtime] rear kinematics attach failed\n");
        } catch (const std::exception& e) {
            std::fprintf(stderr, "[vdsim_realtime] rear susp %s: %s\n",
                         rear.c_str(), e.what());
        }
    }
}
double now_s() {
    return std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

std::unique_ptr<vdsim::IContactProvider> make_ground(
    const vdsim::cosim::RoadConfig& rd, const vdsim::cosim::StuntConfig& stunt) {
    if (!stunt.ground.empty()) {
        if (stunt.ground == "ramp")
            return vdsim::create_ramp_ground(
                stunt.ramp_x_start, stunt.ramp_x_top, stunt.ramp_height,
                stunt.ramp_lip, rd.mu);
        if (stunt.ground == "loop")
            return vdsim::create_loop_ground(
                stunt.loop_center_x, stunt.loop_center_z, stunt.loop_radius, rd.mu);
        std::fprintf(stderr, "[vdsim_realtime] unknown stunt.ground=%s (flat)\n",
                     stunt.ground.c_str());
    }
    const std::string& terrain = rd.terrain;
    const double mu = rd.mu, mu_right = rd.mu_right, mu_boundary = rd.mu_boundary;
    const double grade = rd.grade, bank = rd.bank;
    const double rough_amp = rd.rough_amp, rough_wl = rd.rough_wl;
    const int iso_class = rd.iso_class;
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

void settle_spawn_on_ground(vdsim::IContactProvider& ground,
                            const vdsim::VehicleParams& vp, vdsim::State& s) {
    auto max_pen = [&]() {
        vdsim::ContactArray c{};
        ground.query(s, vp, c);
        double lift = 0.0;
        for (int i = 0; i < vdsim::NUM_WHEELS; ++i) {
            if (c[i].is_valid)
                lift = std::max(lift, c[i].penetration);
        }
        return lift;
    };
    if (s.position.z() < 1e-6) s.position.z() = vp.cg_height;
    for (int drop = 0; drop < 120; ++drop) {
        if (max_pen() > 1e-5) break;
        s.position.z() -= 0.04;
        if (s.position.z() < -2.0) break;
    }
    for (int climb = 0; climb < 400; ++climb) {
        if (max_pen() > 1e-5) break;
        s.position.z() += 0.08;
        if (s.position.z() > 250.0) break;
    }
    for (int k = 0; k < 24; ++k) {
        const double lift = max_pen();
        if (lift < 1e-5) break;
        s.position.z() += lift;
    }
    for (int k = 0; k < 32; ++k) {
        const double pen = max_pen();
        if (pen < 1e-6) break;
        s.position.z() -= std::min(pen * 0.5, 0.02);
    }
    s.velocity.z() = 0.0;
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
    s.steer_applied = o.steer_applied;
    s.throttle_applied = o.throttle_applied;
    s.brake_applied = o.brake_applied;
    s.wheel_radius = wheel_radius;
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

struct WorldVehicle {
    uint32_t id;
    vdsim::VehicleParams vp;
    std::unique_ptr<vdsim::SimSession> sim;
    uint32_t last_seq {0};
    std::chrono::steady_clock::time_point last_cmd {std::chrono::steady_clock::now()};
};

int run_scene(const std::string& scene_path, int argc, char** argv) {
    vdsim::cosim::WorldScenario world;
    try {
        world = vdsim::cosim::load_scene(scene_path);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "[vdsim_realtime] scene: %s\n", e.what());
        return 1;
    }

    const int cmd_port = static_cast<int>(optd(argc, argv, "--cmd-port=", 7001));
    const std::string st_ip = opts(argc, argv, "--state-ip=", "127.0.0.1");
    const int st_port = static_cast<int>(optd(argc, argv, "--state-port=", 7002));
    const double rate = world.rate > 1.0 ? world.rate : optd(argc, argv, "--rate=", 200.0);
    const double cmd_to = world.cmd_timeout > 0.0 ? world.cmd_timeout
                                                  : optd(argc, argv, "--cmd-timeout=", 0.1);
    const double dt = 1.0 / rate;
    double time_scale = world.time_scale > 0.0 ? world.time_scale : 1.0;
    const double cli_ts = optd(argc, argv, "--time-scale=", 0.0);
    if (cli_ts > 0.0) time_scale = cli_ts;
    time_scale = clamp_time_scale(time_scale);
    const auto& rd = world.road;

    vdsim::SolverParams sp;
    vdsim::SimConfig cfg; cfg.nominal_dt = dt;
    if (!rd.sensors.empty()) cfg.sensors = vdsim::SensorParams::from_yaml(rd.sensors);
    cfg.sensor_delay_s = rd.sensor_delay;
    const bool stunt_world = !world.stunt.ground.empty();
    if (stunt_world) {
        sp.stunt_physics = true;
        sp.max_substep_dt = 1e-4;
        sp.max_substeps = 24;
        if (world.stunt.ground == "loop") {
            sp.loop_radius = world.stunt.loop_radius;
            sp.loop_center_x = world.stunt.loop_center_x;
            sp.loop_center_z = world.stunt.loop_center_z;
            sp.loop_rail_guide = world.stunt.rail_guide;
        }
    }

    std::vector<WorldVehicle> fleet;
    std::unordered_map<uint32_t, WorldVehicle*> by_id;
    for (const auto& spn : world.vehicles) {
        WorldVehicle wv;
        wv.id = spn.id;
        wv.vp = vdsim::VehicleParams::from_yaml(spn.vehicle_yaml);
        auto tp = vdsim::TireParams::from_yaml(spn.tire_yaml);
        apply_lugre_cli(tp, argc, argv);
        vdsim::SolverParams sp_v = sp;
        if (spn.level == "L5" && !stunt_world) {
            sp_v.stunt_physics = true;
            sp_v.max_substep_dt = 2e-4;
        }
        auto dyn = make_dyn(spn.level);
        attach_susp_parts(*dyn, spn.level, spn.front_susp, spn.rear_susp);
        auto gnd = make_ground(rd, world.stunt);
        vdsim::State s0;
        s0.position = vdsim::Vec3(spn.x0, spn.y0, spn.z0);
        s0.orientation = vdsim::quat_from_euler(vdsim::Euler{0.0, 0.0, spn.yaw0});
        s0.velocity.x() = spn.vx0;
        const double w0 = (wv.vp.wheel_radius_nominal > 0.0)
            ? spn.vx0 / wv.vp.wheel_radius_nominal : 0.0;
        s0.wheel_spin = {{w0, w0, w0, w0}};
        settle_spawn_on_ground(*gnd, wv.vp, s0);
        wv.sim = std::make_unique<vdsim::SimSession>(
            std::move(dyn), std::move(gnd), wv.vp, tp, sp_v, cfg);
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
        fleet.size(), rate, cmd_port, scene_path.c_str());

    uint32_t seq = 0;
    auto next = std::chrono::steady_clock::now();
    const auto failsafe_dur = std::chrono::duration<double>(cmd_to);
    uint8_t out[vdsim::cosim::kStateBytes];
    while (g_run.load()) {
        time_scale = read_live_time_scale(scene_path, time_scale);
        const double period_s = dt / time_scale;
        const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(period_s));
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
    std::string scene = opts(argc, argv, "--scene=", "");
    if (scene.empty())
        scene = opts(argc, argv, "--scenario=", "");
#ifdef _WIN32
    WSADATA wsa;
    if (::WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
        std::fprintf(stderr, "WSAStartup failed\n"); return 1;
    }
#endif
    int rc;
    if (!scene.empty()) {
        rc = run_scene(scene, argc, argv);
    } else {
        std::fprintf(stderr,
            "usage: %s --scene=<scene.yaml|world.yaml> [options]\n"
            "  (--scenario= accepted as deprecated alias)\n"
            "  [--cmd-port=7001] [--state-ip=127.0.0.1] [--state-port=7002]\n"
            "  [--rate=200] [--time-scale=1] [--cmd-timeout=0.1]\n"
            "  [--lugre] [--no-lugre]  (override tire YAML lugre.enabled)\n",
            argv[0]);
        rc = 2;
    }
#ifdef _WIN32
    ::WSACleanup();
#endif
    return rc;
}
