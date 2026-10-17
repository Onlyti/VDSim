// Live 3D monitor link for vdsim_realtime (work order 36).
//
// `vdsim_realtime --monitor[=PORT]` starts the browser monitor's page server
// (`python -m vdsim_livemonitor serve`) as a child process for the lifetime of
// the simulator and feeds it one vehicle's state over loopback UDP, using the
// same datagram the Python publisher `vdsim_livemonitor.LiveMonitor.push()`
// sends. Monitor trouble (no Python, no module, port taken, child exits) is a
// one-line warning: the simulation keeps running without the monitor.
//
// Header-only, like the rest of cosim/, so the encoders are unit-testable
// without linking the server. The child process side is POSIX only; on Windows
// start() declines with a reason.
#pragma once

#include "vdsim/coordinate.hpp"
#include "vdsim/params.hpp"
#include "vdsim/sim_session.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#ifndef _WIN32
  #include <arpa/inet.h>
  #include <fcntl.h>
  #include <netinet/in.h>
  #include <signal.h>
  #include <sys/socket.h>
  #include <sys/stat.h>
  #include <sys/types.h>
  #include <sys/wait.h>
  #include <unistd.h>
  #ifdef __linux__
    #include <sys/prctl.h>
  #endif
extern char** environ;
#endif

namespace vdsim::cosim {

// Page server port when `--monitor` is given without a value (vdsim_livemonitor.DEFAULT_PORT).
constexpr int kMonitorDefaultPort = 8790;
// Upper bound on state datagrams per second (the page server broadcasts at 30 Hz).
constexpr double kMonitorRateHz = 30.0;
// Scene (vehicle geometry) resend period [s] (vdsim_livemonitor.SCENE_RESEND_S).
constexpr double kMonitorSceneResendS = 2.0;
// Scenes sent at kMonitorSceneEarlyS [s] spacing right after start (child still binding).
constexpr int    kMonitorSceneEarlyCount = 4;
constexpr double kMonitorSceneEarlyS = 0.25;
// Grace period between SIGTERM and SIGKILL when stopping the page server [s].
constexpr double kMonitorStopGraceS = 3.0;

/// @brief Append `"key":value` to a JSON object body.
/// @param out Object body being built (no braces).
/// @param key Member name (plain ASCII, not escaped).
/// @param v Value; NaN/inf become `null` (the page skips them, like the
///        Python publisher's allow_nan=False encoding).
inline void monitor_json_member(std::string& out, const char* key, double v) {
    char buf[64];
    if (std::isfinite(v))
        std::snprintf(buf, sizeof(buf), "\"%s\":%.9g", key, v);
    else
        std::snprintf(buf, sizeof(buf), "\"%s\":null", key);
    if (!out.empty()) out += ',';
    out += buf;
}

/// @brief Encode one monitor state datagram.
/// @param o Session output of the monitored vehicle.
/// @return `{"state":{...}}` with the page's wire keys: x, y, z [m] and yaw
///         [rad] (world frame), t = simulation time [s], vx [m/s], delta =
///         realized road-wheel steer [rad], roll, pitch [rad] -- the keys
///         `LiveMonitor.push()` produces from a VDSimPlant observation.
inline std::string monitor_state_json(const SimOutput& o) {
    std::string b;
    monitor_json_member(b, "x", o.state.position.x());
    monitor_json_member(b, "y", o.state.position.y());
    monitor_json_member(b, "yaw", yaw_from_quat(o.state.orientation));
    monitor_json_member(b, "t", o.sim_time);
    monitor_json_member(b, "vx", o.state.velocity.x());
    monitor_json_member(b, "delta", o.steer_applied);
    monitor_json_member(b, "roll", o.roll);
    monitor_json_member(b, "pitch", o.pitch);
    monitor_json_member(b, "z", o.state.position.z());
    return "{\"state\":{" + b + "}}";
}

/// @brief Encode the scene message carrying the vehicle drawing geometry.
/// @param vp Parameters of the monitored vehicle.
/// @return `{"scene":{"vehicle":{...}}}` with the keys of
///         `vdsim_livemonitor.vehicle_geometry()` (axle positions, mean track,
///         nominal wheel radius [m]).
inline std::string monitor_scene_json(const VehicleParams& vp) {
    std::string v;
    monitor_json_member(v, "cg_to_front", vp.cg_to_front);
    monitor_json_member(v, "cg_to_rear", vp.cg_to_rear);
    monitor_json_member(v, "track", 0.5 * (vp.track_front + vp.track_rear));
    monitor_json_member(v, "wheel_radius", vp.wheel_radius_nominal);
    return "{\"scene\":{\"vehicle\":{" + v + "}}}";
}

/// @brief Read the monitor port from the command line.
/// @param argc,argv Server arguments.
/// @return 0 when `--monitor` is absent (monitor off), kMonitorDefaultPort for a
///         bare `--monitor`, the value of `--monitor=PORT`, or -1 when PORT is
///         not an integer in 1..65535.
inline int monitor_port_from_args(int argc, char** argv) {
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--monitor") return kMonitorDefaultPort;
        if (a.rfind("--monitor=", 0) == 0) {
            const std::string v = a.substr(10);
            char* end = nullptr;
            const long p = std::strtol(v.c_str(), &end, 10);
            if (v.empty() || *end != '\0' || p < 1 || p > 65535) return -1;
            return static_cast<int>(p);
        }
    }
    return 0;
}

/// @brief Resolve an executable name the way execvp would, in the parent.
/// @param name Path (contains '/') or a bare name looked up on PATH.
/// @return Path of an executable regular file, or empty when none is found.
inline std::string monitor_find_executable(const std::string& name) {
#ifdef _WIN32
    return name;
#else
    auto runnable = [](const std::string& p) {
        struct stat st {};
        return ::stat(p.c_str(), &st) == 0 && S_ISREG(st.st_mode) && ::access(p.c_str(), X_OK) == 0;
    };
    if (name.empty()) return {};
    if (name.find('/') != std::string::npos) return runnable(name) ? name : std::string();
    const char* path = std::getenv("PATH");
    const std::string rest = path ? path : "/usr/bin:/bin";
    size_t pos = 0;
    while (pos <= rest.size()) {
        const size_t colon = rest.find(':', pos);
        std::string dir = rest.substr(pos, colon == std::string::npos ? std::string::npos : colon - pos);
        if (dir.empty()) dir = ".";
        const std::string cand = dir + "/" + name;
        if (runnable(cand)) return cand;
        if (colon == std::string::npos) break;
        pos = colon + 1;
    }
    return {};
#endif
}

// Where and how to start the page server.
struct MonitorOptions {
    int         port {kMonitorDefaultPort};   // page (TCP) and ingest (UDP 127.0.0.1) port
    std::string python {"python3"};           // interpreter, path or name on PATH
    std::string module_dir;                   // prepended to PYTHONPATH when it exists
};

/// Page server child process plus the loopback state sender.
///
/// Lifetime: start() once, push() every simulation tick (rate-limited inside),
/// stop() (or the destructor) on every exit path. A failed start() or a child
/// that dies later leaves the link inactive; push() then does nothing.
class MonitorLink {
public:
    MonitorLink() = default;
    MonitorLink(const MonitorLink&) = delete;
    MonitorLink& operator=(const MonitorLink&) = delete;
    ~MonitorLink() { stop(); }

    /// @brief Start the page server child and open the state socket.
    /// @param opt Port, interpreter and module directory.
    /// @param vp Monitored vehicle (drawing geometry for the scene message).
    /// @param why Set to a one-line reason when the monitor stays off.
    /// @return true when the child was started; false leaves the link inactive.
    bool start(const MonitorOptions& opt, const VehicleParams& vp, std::string* why) {
#ifdef _WIN32
        (void)opt; (void)vp;
        if (why) *why = "--monitor is not supported on Windows builds of vdsim_realtime";
        return false;
#else
        port_ = opt.port;
        const std::string py = monitor_find_executable(opt.python);
        if (py.empty()) {
            if (why) *why = "python interpreter '" + opt.python + "' not found";
            return false;
        }
        if (!port_free(port_)) {
            if (why) *why = "port " + std::to_string(port_) + " is already in use";
            return false;
        }
        // Everything the child needs is built here: between fork() and exec()
        // only async-signal-safe calls are allowed.
        std::vector<std::string> args = {py, "-m", "vdsim_livemonitor", "serve",
                                         "--port", std::to_string(port_)};
        std::vector<std::string> env;
        std::string pypath;
        struct stat st {};
        if (!opt.module_dir.empty() && ::stat(opt.module_dir.c_str(), &st) == 0 && S_ISDIR(st.st_mode))
            pypath = opt.module_dir;
        for (char** e = environ; e && *e; ++e) {
            const std::string kv = *e;
            if (kv.rfind("PYTHONPATH=", 0) == 0) {
                const std::string old = kv.substr(11);
                if (!old.empty()) pypath = pypath.empty() ? old : pypath + ":" + old;
                continue;
            }
            env.push_back(kv);
        }
        if (!pypath.empty()) env.push_back("PYTHONPATH=" + pypath);
        std::vector<char*> argv_c, env_c;
        for (auto& a : args) argv_c.push_back(a.data());
        argv_c.push_back(nullptr);
        for (auto& e : env) env_c.push_back(e.data());
        env_c.push_back(nullptr);

        const pid_t parent = ::getpid();
        const pid_t pid = ::fork();
        if (pid < 0) {
            if (why) *why = "fork failed";
            return false;
        }
        if (pid == 0) {
#ifdef __linux__
            // A parent killed outright (SIGKILL) must not leave the page server behind.
            ::prctl(PR_SET_PDEATHSIG, SIGTERM);
            if (::getppid() != parent) ::_exit(0);
#else
            (void)parent;
#endif
            const int null_fd = ::open("/dev/null", O_RDWR);
            if (null_fd >= 0) {
                ::dup2(null_fd, 0); ::dup2(null_fd, 1); ::dup2(null_fd, 2);
                if (null_fd > 2) ::close(null_fd);
            }
            ::execve(argv_c[0], argv_c.data(), env_c.data());
            ::_exit(127);
        }
        pid_ = pid;
        sock_ = ::socket(AF_INET, SOCK_DGRAM, 0);
        dst_ = sockaddr_in{};
        dst_.sin_family = AF_INET;
        dst_.sin_port = htons(static_cast<uint16_t>(port_));
        dst_.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        scene_ = monitor_scene_json(vp);
        active_ = sock_ >= 0;
        if (!active_) {
            if (why) *why = "cannot open the state socket";
            stop();
        }
        return active_;
#endif
    }

    /// @return true while the child is believed to be running.
    bool active() const { return active_; }
    /// @return page server process id, -1 when none is running.
    long pid() const {
#ifdef _WIN32
        return -1;
#else
        return static_cast<long>(pid_);
#endif
    }
    /// @return state datagrams sent so far.
    unsigned long long n_sent() const { return n_sent_; }

    /// @brief Send the vehicle state, at most kMonitorRateHz times per second.
    /// @param o Session output of the monitored vehicle.
    /// @param wall_s Monotonic wall clock [s] used for the rate limit.
    /// @param why Set to a one-line reason when the child is found dead; the
    ///        link turns inactive and the caller should print it once.
    /// @return false once the link is (or just became) inactive.
    bool push(const SimOutput& o, double wall_s, std::string* why) {
        if (!active_) return false;
        if (wall_s - last_push_ < 1.0 / kMonitorRateHz) return true;
        last_push_ = wall_s;
#ifndef _WIN32
        if (!child_running(why)) return false;
        // The child binds its ingest port a moment after start(), so the first
        // scenes are sent at a shorter period before settling to the resend one.
        if (wall_s - last_scene_ >= (n_scene_ < kMonitorSceneEarlyCount ? kMonitorSceneEarlyS
                                                                         : kMonitorSceneResendS)) {
            send(scene_);
            last_scene_ = wall_s;
            ++n_scene_;
        }
        send(monitor_state_json(o));
        ++n_sent_;
#else
        (void)o; (void)why;
#endif
        return true;
    }

    /// @brief Terminate and reap the page server, close the socket. Idempotent.
    void stop() {
        active_ = false;
#ifndef _WIN32
        if (sock_ >= 0) { ::close(sock_); sock_ = -1; }
        if (pid_ <= 0) return;
        ::kill(pid_, SIGTERM);
        const int polls = static_cast<int>(kMonitorStopGraceS / 0.02);
        for (int i = 0; i < polls; ++i) {
            if (::waitpid(pid_, nullptr, WNOHANG) != 0) { pid_ = -1; return; }
            ::usleep(20000);
        }
        ::kill(pid_, SIGKILL);
        ::waitpid(pid_, nullptr, 0);
        pid_ = -1;
#endif
    }

private:
#ifndef _WIN32
    /// @brief Check that nothing holds TCP 0.0.0.0:port or UDP 127.0.0.1:port.
    /// @param port Port to test.
    /// @return true when both binds succeed (they are released right away).
    static bool port_free(int port) {
        auto try_bind = [port](int type, uint32_t addr) {
            const int s = ::socket(AF_INET, type, 0);
            if (s < 0) return false;
            const int one = 1;   // as Python's http.server, so TIME_WAIT is not "in use"
            if (type == SOCK_STREAM) ::setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
            sockaddr_in a {};
            a.sin_family = AF_INET;
            a.sin_port = htons(static_cast<uint16_t>(port));
            a.sin_addr.s_addr = htonl(addr);
            const bool ok = ::bind(s, reinterpret_cast<sockaddr*>(&a), sizeof(a)) == 0;
            ::close(s);
            return ok;
        };
        return try_bind(SOCK_STREAM, INADDR_ANY) && try_bind(SOCK_DGRAM, INADDR_LOOPBACK);
    }

    /// @brief Reap the child if it exited; an exit turns the link inactive.
    /// @param why Set to a one-line reason when the child is gone.
    /// @return true while the child runs.
    bool child_running(std::string* why) {
        int status = 0;
        const pid_t r = ::waitpid(pid_, &status, WNOHANG);
        if (r == 0) return true;
        pid_ = -1;
        active_ = false;
        if (sock_ >= 0) { ::close(sock_); sock_ = -1; }
        if (why) {
            if (r > 0 && WIFEXITED(status) && WEXITSTATUS(status) == 127)
                *why = "page server could not be executed";
            else if (r > 0 && WIFEXITED(status))
                *why = "page server exited with status " + std::to_string(WEXITSTATUS(status))
                     + " (vdsim_livemonitor not importable or its port taken)";
            else
                *why = "page server stopped unexpectedly";
        }
        return false;
    }

    /// @brief One datagram to the page server's ingest port; loss is not an error.
    void send(const std::string& msg) {
        ::sendto(sock_, msg.data(), msg.size(), 0,
                 reinterpret_cast<const sockaddr*>(&dst_), sizeof(dst_));
    }

    pid_t       pid_ {-1};
    int         sock_ {-1};
    sockaddr_in dst_ {};
#endif
    int         port_ {kMonitorDefaultPort};
    bool        active_ {false};
    double      last_push_ {-1.0e300};
    double      last_scene_ {-1.0e300};
    unsigned long long n_sent_ {0};
    int         n_scene_ {0};
    std::string scene_;
};

}  // namespace vdsim::cosim
