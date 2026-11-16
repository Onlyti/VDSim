// User module fault boundary (work order 51b): guard wrappers and the name registry.
#include "vdsim/module_guard.hpp"

#include "module_guard_internal.hpp"

#include <cmath>
#include <cstdlib>
#include <exception>
#include <functional>
#include <mutex>
#include <string>
#include <typeinfo>
#include <unordered_map>
#include <utility>
#include <vector>

#if defined(__GNUG__)
#include <cxxabi.h>
#endif

namespace vdsim {

namespace {
std::mutex& name_mutex() {
    static std::mutex m;
    return m;
}
std::unordered_map<const void*, std::string>& name_table() {
    static std::unordered_map<const void*, std::string> t;
    return t;
}
}  // namespace

void register_module_name(const void* module, std::string name) {
    if (!module) return;
    std::lock_guard<std::mutex> lk(name_mutex());
    name_table()[module] = std::move(name);
}

void forget_module_name(const void* module) {
    std::lock_guard<std::mutex> lk(name_mutex());
    name_table().erase(module);
}

namespace detail {

namespace {

/// Registered name of `p` (consumed), else the demangled dynamic type name.
std::string display_name(const void* p, const std::type_info& t) {
    {
        std::lock_guard<std::mutex> lk(name_mutex());
        auto it = name_table().find(p);
        if (it != name_table().end()) {
            std::string n = std::move(it->second);
            name_table().erase(it);
            return n;
        }
    }
#if defined(__GNUG__)
    int status = 0;
    char* d = abi::__cxa_demangle(t.name(), nullptr, nullptr, &status);
    if (status == 0 && d) {
        std::string n(d);
        std::free(d);
        return n;
    }
#endif
    return t.name();
}

/// Shared part of every wrapper: the latch, the kind and the module name.
struct GuardBase {
    ModuleFaultLatch* latch;
    std::string kind;
    std::string name;

    /// Run f; on an exception record it and return false.
    template <class F>
    bool run(F&& f) noexcept {
        try {
            f();
            return true;
        } catch (const std::exception& e) {
            latch->record(kind, name, std::string("exception: ") + e.what());
        } catch (...) {
            latch->record(kind, name, "exception: unknown (not std::exception)");
        }
        return false;
    }
    /// Record a non-finite field; true when v is finite.
    bool finite(double v, const char* field) {
        if (std::isfinite(v)) return true;
        latch->record(kind, name, std::string("non-finite output ") + field);
        return false;
    }
    bool faulted() const { return latch->set; }
};

const char* const kWheelField[] = {"wheel_torque[0]", "wheel_torque[1]", "wheel_torque[2]",
                                   "wheel_torque[3]"};

class GuardedBrake final : public IBrakeSystem, GuardBase {
public:
    GuardedBrake(std::shared_ptr<IBrakeSystem> m, GuardBase b) : GuardBase(std::move(b)), m_(std::move(m)) {}
    LcLevel min_lon_level() const override { return m_->min_lon_level(); }
    LcLevel max_lon_level() const override { return m_->max_lon_level(); }
    std::array<double, NUM_WHEELS> wheel_torque(const SubsystemContext& ctx) override {
        std::array<double, NUM_WHEELS> t {{0.0, 0.0, 0.0, 0.0}};
        if (faulted()) return t;
        std::array<double, NUM_WHEELS> r {};
        if (!run([&] { r = m_->wheel_torque(ctx); })) return t;
        for (int i = 0; i < NUM_WHEELS; ++i)
            if (!finite(r[i], kWheelField[i])) return t;
        return r;
    }
    void begin_step(const SubsystemContext& ctx, double dt) override {
        if (!faulted()) run([&] { m_->begin_step(ctx, dt); });
    }
    void reset() override { m_->reset(); }

private:
    std::shared_ptr<IBrakeSystem> m_;
};

class GuardedSteering final : public ISteeringSystem, GuardBase {
public:
    GuardedSteering(std::shared_ptr<ISteeringSystem> m, GuardBase b)
        : GuardBase(std::move(b)), m_(std::move(m)) {}
    LcLevel min_lat_level() const override { return m_->min_lat_level(); }
    LcLevel max_lat_level() const override { return m_->max_lat_level(); }
    SteeringOutput apply(const SubsystemContext& ctx) override {
        if (faulted()) return SteeringOutput{};
        SteeringOutput o;
        if (!run([&] { o = m_->apply(ctx); })) return SteeringOutput{};
        if (!finite(o.rack_travel, "rack_travel") || !finite(o.motor_force, "motor_force") ||
            !finite(o.roadwheel_angle, "roadwheel_angle"))
            return SteeringOutput{};
        return o;
    }
    void begin_step(const SubsystemContext& ctx, double dt) override {
        if (!faulted()) run([&] { m_->begin_step(ctx, dt); });
    }
    void reset() override { m_->reset(); }

private:
    std::shared_ptr<ISteeringSystem> m_;
};

class GuardedDrivetrain final : public IDrivetrain, GuardBase {
public:
    GuardedDrivetrain(std::shared_ptr<IDrivetrain> m, GuardBase b)
        : GuardBase(std::move(b)), m_(std::move(m)) {}
    LcLevel min_lon_level() const override { return m_->min_lon_level(); }
    LcLevel max_lon_level() const override { return m_->max_lon_level(); }
    DrivetrainOutput apply(const SubsystemContext& ctx) override {
        if (faulted()) return DrivetrainOutput{};
        DrivetrainOutput o;
        if (!run([&] { o = m_->apply(ctx); })) return DrivetrainOutput{};
        for (int i = 0; i < NUM_WHEELS; ++i)
            if (!finite(o.wheel_torque[i], kWheelField[i])) return DrivetrainOutput{};
        if (!finite(o.brake_absorbed, "brake_absorbed")) return DrivetrainOutput{};
        return o;
    }
    void begin_step(const SubsystemContext& ctx, double dt) override {
        if (!faulted()) run([&] { m_->begin_step(ctx, dt); });
    }
    void reset() override { m_->reset(); }
    double engine_rpm() const override { return m_->engine_rpm(); }
    int current_gear() const override { return m_->current_gear(); }
    double wheel_engine_inertia(int wheel) const override { return m_->wheel_engine_inertia(wheel); }
    bool set_shift_policy(ShiftPolicy fn) override { return m_->set_shift_policy(std::move(fn)); }

private:
    std::shared_ptr<IDrivetrain> m_;
};

class GuardedSuspension final : public ISuspension, GuardBase {
public:
    GuardedSuspension(std::shared_ptr<ISuspension> m, GuardBase b)
        : GuardBase(std::move(b)), m_(std::move(m)) {}
    double force(const SubsystemContext& ctx, const CornerInput& c) override {
        if (faulted()) return 0.0;
        double f = 0.0;
        if (!run([&] { f = m_->force(ctx, c); })) return 0.0;
        return finite(f, "force") ? f : 0.0;
    }
    void reset() override { m_->reset(); }

private:
    std::shared_ptr<ISuspension> m_;
};

class GuardedAntiRollBar final : public IAntiRollBar, GuardBase {
public:
    GuardedAntiRollBar(std::shared_ptr<IAntiRollBar> m, GuardBase b)
        : GuardBase(std::move(b)), m_(std::move(m)) {}
    std::pair<double, double> force(const SubsystemContext& ctx, const AxleDefl& d) override {
        if (faulted()) return {0.0, 0.0};
        std::pair<double, double> f {0.0, 0.0};
        if (!run([&] { f = m_->force(ctx, d); })) return {0.0, 0.0};
        if (!finite(f.first, "force[0]") || !finite(f.second, "force[1]")) return {0.0, 0.0};
        return f;
    }
    void reset() override { m_->reset(); }

private:
    std::shared_ptr<IAntiRollBar> m_;
};

class GuardedAero final : public IAeroModel, GuardBase {
public:
    GuardedAero(std::shared_ptr<IAeroModel> m, GuardBase b) : GuardBase(std::move(b)), m_(std::move(m)) {}
    std::vector<AeroForce> compute(const SubsystemContext& ctx, const AeroInput& in) override {
        if (faulted()) return {};
        std::vector<AeroForce> v;
        if (!run([&] { v = m_->compute(ctx, in); })) return {};
        static const char* const ax[] = {"x", "y", "z"};
        for (std::size_t k = 0; k < v.size(); ++k)
            for (int j = 0; j < 3; ++j) {
                const std::string at = "[" + std::to_string(k) + "].";
                if (!finite(v[k].force_body[j], (at + "force_body." + ax[j]).c_str()) ||
                    !finite(v[k].point[j], (at + "point." + ax[j]).c_str()))
                    return {};
            }
        return v;
    }
    void begin_step(const SubsystemContext& ctx, double dt) override {
        if (!faulted()) run([&] { m_->begin_step(ctx, dt); });
    }
    void reset() override { m_->reset(); }

private:
    std::shared_ptr<IAeroModel> m_;
};

/// Common wrap step: pass through nullptr / already guarded, else build W.
template <class W, class I>
std::shared_ptr<I> wrap(std::shared_ptr<I> m, ModuleFaultLatch* latch, const char* kind) {
    if (!m || dynamic_cast<W*>(m.get())) return m;
    I& ref = *m;
    GuardBase b {latch, kind, display_name(m.get(), typeid(ref))};
    ++latch->installed;
    return std::make_shared<W>(std::move(m), std::move(b));
}

}  // namespace

ModuleFaultLatch* module_latch_of(IVehicleDynamics& dyn) {
    auto* h = dynamic_cast<ModuleGuardHost*>(&dyn);
    return h ? &h->module_latch() : nullptr;
}

std::shared_ptr<IBrakeSystem> guard_brake(std::shared_ptr<IBrakeSystem> m, ModuleFaultLatch* latch) {
    return wrap<GuardedBrake>(std::move(m), latch, "brake");
}
std::shared_ptr<ISteeringSystem> guard_steering(std::shared_ptr<ISteeringSystem> m,
                                                ModuleFaultLatch* latch) {
    return wrap<GuardedSteering>(std::move(m), latch, "steering");
}
std::shared_ptr<IDrivetrain> guard_drivetrain(std::shared_ptr<IDrivetrain> m, ModuleFaultLatch* latch) {
    return wrap<GuardedDrivetrain>(std::move(m), latch, "drivetrain");
}
std::shared_ptr<ISuspension> guard_suspension(std::shared_ptr<ISuspension> m, ModuleFaultLatch* latch) {
    return wrap<GuardedSuspension>(std::move(m), latch, "suspension");
}
std::shared_ptr<IAntiRollBar> guard_antirollbar(std::shared_ptr<IAntiRollBar> m,
                                                ModuleFaultLatch* latch) {
    return wrap<GuardedAntiRollBar>(std::move(m), latch, "antirollbar");
}
std::shared_ptr<IAeroModel> guard_aero(std::shared_ptr<IAeroModel> m, ModuleFaultLatch* latch) {
    return wrap<GuardedAero>(std::move(m), latch, "aero");
}

}  // namespace detail
}  // namespace vdsim
