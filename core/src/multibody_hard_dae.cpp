#include "vdsim/multibody.hpp"

#include "vdsim/multibody_math.hpp"

#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>

#include <cmath>
#include <memory>
#include <stdexcept>

namespace vdsim::mb {

namespace {

constexpr double kBaumgarteAlpha = 12.0;
constexpr double kBaumgarteBeta  = 12.0;

Vec3 yaml_vec3(const YAML::Node& n) {
    return Vec3(n[0].as<double>(), n[1].as<double>(), n[2].as<double>());
}

struct ConstrainedStepResult {
    double qdd {0.0};
    double lam {0.0};
};

ConstrainedStepResult solve_constrained_revolute(double I, double phi_q, double tau,
                                                 double phi, double phi_d,
                                                 double gamma_target_ddot) {
    const double gamma = gamma_target_ddot - 2.0 * kBaumgarteAlpha * phi_d
                       - kBaumgarteBeta * kBaumgarteBeta * phi;
    ConstrainedStepResult out;
    if (std::abs(phi_q) < 1e-9) {
        out.qdd = tau / std::max(0.2, I);
        return out;
    }
    out.lam = (phi_q * tau - I * gamma) / (phi_q * phi_q);
    out.qdd = (tau - phi_q * out.lam) / std::max(0.2, I);
    return out;
}

bool trilaterate3(const Vec3& c1, double r1, const Vec3& c2, double r2,
                  const Vec3& c3, double r3, const Vec3& near, Vec3& out) {
    Vec3 ex = c2 - c1;
    const double d = ex.norm();
    if (d < 1e-9) return false;
    ex /= d;
    const Vec3 tmp = c3 - c1;
    const double i_ = ex.dot(tmp);
    Vec3 ey_raw = tmp - i_ * ex;
    const double ny = ey_raw.norm();
    if (ny < 1e-9) return false;
    const Vec3 ey = ey_raw / ny;
    const Vec3 ez = ex.cross(ey);
    const double j_ = ey.dot(c3 - c1);
    const double x = (r1 * r1 - r2 * r2 + d * d) / (2.0 * d);
    const double y = (r1 * r1 - r3 * r3 + i_ * i_ + j_ * j_ - 2.0 * i_ * x) / (2.0 * j_);
    const double z_sq = r1 * r1 - x * x - y * y;
    if (z_sq < -1e-9) return false;
    const double z = std::sqrt(std::max(0.0, z_sq));
    const Vec3 p_pos = c1 + x * ex + y * ey + z * ez;
    const Vec3 p_neg = c1 + x * ex + y * ey - z * ez;
    out = ((p_pos - near).norm() <= (p_neg - near).norm()) ? p_pos : p_neg;
    return true;
}

template <typename F>
Eigen::VectorXd lm_solve(F&& f, Eigen::VectorXd x0, int max_iter = 200,
                         double tol = 1e-9) {
    Eigen::VectorXd x = x0;
    double lambda = 1e-3;
    Eigen::VectorXd r = f(x);
    double cost = r.squaredNorm();
    const int n = static_cast<int>(x.size());
    const int m = static_cast<int>(r.size());
    for (int it = 0; it < max_iter; ++it) {
        Eigen::MatrixXd J(m, n);
        constexpr double h = 1e-6;
        for (int i = 0; i < n; ++i) {
            Eigen::VectorXd xp = x;
            xp(i) += h;
            J.col(i) = (f(xp) - r) / h;
        }
        const Eigen::MatrixXd JTJ = J.transpose() * J;
        const Eigen::VectorXd JTr = J.transpose() * r;
        Eigen::MatrixXd A = JTJ;
        A.diagonal().array() += lambda * JTJ.diagonal().array().max(1e-12);
        const Eigen::VectorXd dx = A.ldlt().solve(-JTr);
        const Eigen::VectorXd x_new = x + dx;
        const Eigen::VectorXd r_new = f(x_new);
        const double cost_new = r_new.squaredNorm();
        if (cost_new < cost) {
            x = x_new;
            r = r_new;
            cost = cost_new;
            lambda *= 0.5;
            if (dx.norm() < tol) break;
        } else {
            lambda *= 2.0;
            if (lambda > 1e10) break;
        }
    }
    return x;
}

WheelPose pose_from_spin(const Vec3& wheel, const Vec3& spin, const Vec3& wheel_static,
                         const std::string& side, const Vec3& kp = Vec3::Zero(),
                         const Vec3& strut_top = Vec3::Zero()) {
    WheelPose wp;
    wp.position_world = wheel;
    wp.toe_rad = std::atan2(spin.x(), spin.y());
    wp.camber_rad = std::atan2(-spin.z(), std::abs(spin.y()));
    if (side == "right") {
        wp.toe_rad = -wp.toe_rad;
        wp.camber_rad = -wp.camber_rad;
    }
    if (strut_top.norm() > 1e-9) {
        const Vec3 kp_vec = kp - strut_top;
        const double kp_xz = std::hypot(kp_vec.x(), kp_vec.z());
        wp.caster_rad = (kp_xz > 1e-9) ? std::atan2(kp_vec.x(), -kp_vec.z()) : 0.0;
    }
    (void)wheel_static;
    return wp;
}

class TrailingArmHardJointDae final : public IHardJointDaeModel {
public:
    explicit TrailingArmHardJointDae(const std::string& yaml_path,
                                     const SuspensionTopology& topo) {
        YAML::Node root = YAML::LoadFile(yaml_path);
        side_ = root["side"] ? root["side"].as<std::string>() : "left";
        wheel_static_ = yaml_vec3(root["wheel"]["center"]);
        wheel_spin_axis_ = yaml_vec3(root["wheel"]["spin_axis"]);
        pivot_ = yaml_vec3(root["arm_pivot"]["chassis_inboard"]);
        const Vec3 pivot_out = yaml_vec3(root["arm_pivot"]["chassis_outboard"]);
        axis_ = (pivot_out - pivot_).normalized();
        wheel_off_ = wheel_static_ - pivot_;
        I_theta_ = corner_inertia_about_axis(topo.bodies, axis_, pivot_);
    }

    void initialize(HardJointCornerState& st,
                    const PrescribedCornerMotion& mot) const override {
        st.q = solve_theta_for_travel(mot.travel_z);
        st.qd = 0.0;
        st.knuckle_aa.setZero();
    }

    WheelPose step(HardJointCornerState& st, const PrescribedCornerMotion& mot,
                   const WheelLoad& load, double dt) const override {
        if (dt <= 0.0) return pose_at(st.q, mot.steer_rack_dy);
        const double z_target = wheel_static_.z() + mot.travel_z;
        const Vec3 wheel = wheel_at(st.q);
        const Vec3 J = axis_.cross(wheel - pivot_);
        const double phi_q = J.z();
        const double phi = wheel.z() - z_target;
        const double phi_d = phi_q * st.qd - mot.travel_z_dot;
        const double phi_qq = axis_.cross(J).z();
        const double tau = J.dot(load.force_world);
        const auto sol = solve_constrained_revolute(
            I_theta_, phi_q, tau, phi, phi_d,
            mot.travel_z_ddot - phi_qq * st.qd * st.qd);
        const int nsub = std::max(1, static_cast<int>(std::ceil(dt / 0.001)));
        const double h = dt / static_cast<double>(nsub);
        for (int i = 0; i < nsub; ++i) {
            st.qd += h * sol.qdd;
            st.q += h * st.qd;
        }
        return pose_at(st.q, mot.steer_rack_dy);
    }

private:
    Vec3 wheel_at(double theta) const {
        return pivot_ + rodrigues(axis_, theta) * wheel_off_;
    }

    double solve_theta_for_travel(double travel) const {
        const double target_z = wheel_static_.z() + travel;
        double theta = 0.0;
        for (int it = 0; it < 40; ++it) {
            const Vec3 w = wheel_at(theta);
            const double err = w.z() - target_z;
            if (std::abs(err) < 1e-9) break;
            const double dth = 1e-5;
            const double slope = (wheel_at(theta + dth).z() - w.z()) / dth;
            if (std::abs(slope) < 1e-12) break;
            theta -= err / slope;
        }
        return theta;
    }

    WheelPose pose_at(double theta, double /*steer*/) const {
        const Mat3 R = rodrigues(axis_, theta);
        const Vec3 wheel = pivot_ + R * wheel_off_;
        const Vec3 spin = R * wheel_spin_axis_;
        return pose_from_spin(wheel, spin, wheel_static_, side_);
    }

    std::string side_;
    Vec3 wheel_static_, wheel_spin_axis_, pivot_, axis_, wheel_off_;
    double I_theta_ {1.0};
};

class MacPhersonHardJointDae final : public IHardJointDaeModel {
public:
    explicit MacPhersonHardJointDae(const std::string& yaml_path,
                                    const SuspensionTopology& topo) {
        YAML::Node root = YAML::LoadFile(yaml_path);
        side_ = root["side"] ? root["side"].as<std::string>() : "left";
        wheel_static_ = yaml_vec3(root["wheel"]["center"]);
        wheel_spin_axis_ = yaml_vec3(root["wheel"]["spin_axis"]);
        lca_cf_ = yaml_vec3(root["lca"]["chassis_front"]);
        lca_cr_ = yaml_vec3(root["lca"]["chassis_rear"]);
        lca_axis_ = (lca_cr_ - lca_cf_).normalized();
        lca_pivot_ = lca_cf_;
        lca_knuckle_static_ = yaml_vec3(root["lca"]["knuckle"]);
        strut_top_ = yaml_vec3(root["strut"]["top"]);
        strut_bottom_ = yaml_vec3(root["strut"]["bottom"]);
        tube_axis_body_ = (strut_top_ - strut_bottom_).normalized();
        tr_inner_static_ = yaml_vec3(root["tie_rod"]["rack"]);
        tr_knuckle_static_ = yaml_vec3(root["tie_rod"]["knuckle"]);
        L_tr_ = (tr_knuckle_static_ - tr_inner_static_).norm();
        off_sk_ = strut_bottom_ - lca_knuckle_static_;
        off_tk_ = tr_knuckle_static_ - lca_knuckle_static_;
        off_wheel_ = wheel_static_ - lca_knuckle_static_;
        I_theta_ = corner_inertia_about_axis(topo.bodies, lca_axis_, lca_pivot_);
    }

    void initialize(HardJointCornerState& st,
                    const PrescribedCornerMotion& mot) const override {
        st.q = solve_theta_for_travel(mot.travel_z, mot.steer_rack_dy, st.knuckle_aa);
        st.qd = 0.0;
    }

    WheelPose step(HardJointCornerState& st, const PrescribedCornerMotion& mot,
                   const WheelLoad& load, double dt) const override {
        if (dt <= 0.0) return pose_at(st.q, mot.steer_rack_dy, st.knuckle_aa);
        const double z_target = wheel_static_.z() + mot.travel_z;
        const auto geom = geometry_at(st.q, mot.steer_rack_dy, st.knuckle_aa);
        // CENTRAL differences for w'(=J) and w''(=phi_qq) at dth=1e-3 (was forward diff
        // at 1e-5, plus a redundant re-solve for the 2nd difference). See the
        // kinematic_at variant — robustness/cleanup, not a behavioral change.
        const double dth = 1e-3;
        const auto geom_p = geometry_at(st.q + dth, mot.steer_rack_dy, geom.aa);
        const auto geom_m = geometry_at(st.q - dth, mot.steer_rack_dy, geom.aa);
        const Vec3 J = (geom_p.wheel - geom_m.wheel) / (2.0 * dth);
        const double phi_q = J.z();
        const double phi = geom.wheel.z() - z_target;
        const double phi_d = phi_q * st.qd - mot.travel_z_dot;
        const double phi_qq = (geom_p.wheel.z() - 2.0 * geom.wheel.z() + geom_m.wheel.z())
                            / (dth * dth);
        const double tau = J.dot(load.force_world);
        const auto sol = solve_constrained_revolute(
            I_theta_, phi_q, tau, phi, phi_d,
            mot.travel_z_ddot - phi_qq * st.qd * st.qd);
        const int nsub = std::max(1, static_cast<int>(std::ceil(dt / 0.001)));
        const double h = dt / static_cast<double>(nsub);
        for (int i = 0; i < nsub; ++i) {
            st.qd += h * sol.qdd;
            st.q += h * st.qd;
            st.knuckle_aa = geometry_at(st.q, mot.steer_rack_dy, st.knuckle_aa).aa;
        }
        return pose_at(st.q, mot.steer_rack_dy, st.knuckle_aa);
    }

private:
    struct Geom { Vec3 wheel, sk, spin, aa; };

    Vec3 lca_knuckle_at(double theta) const {
        return lca_pivot_ + rodrigues(lca_axis_, theta) * (lca_knuckle_static_ - lca_pivot_);
    }

    Geom geometry_at(double theta, double steer_dy, Vec3 aa0) const {
        const Vec3 lk = lca_knuckle_at(theta);
        const Vec3 tr_inner = tr_inner_static_ + Vec3(0.0, steer_dy, 0.0);
        aa0 = solve_knuckle_aa(lk, tr_inner, aa0);
        const Mat3 R = axis_angle_to_R(aa0);
        Geom g;
        g.aa = aa0;
        g.sk = lk + R * off_sk_;
        g.wheel = lk + R * off_wheel_;
        g.spin = R * wheel_spin_axis_;
        return g;
    }

    Vec3 solve_knuckle_aa(const Vec3& lk, const Vec3& tr_inner, Vec3 aa0) const {
        auto residual4 = [&](const Vec3& aa) {
            const Mat3 R = axis_angle_to_R(aa);
            const Vec3 sk = lk + R * off_sk_;
            const Vec3 tk = lk + R * off_tk_;
            const Vec3 cross = (sk - strut_top_).cross(R * tube_axis_body_);
            Eigen::Vector4d r;
            r << cross.x(), cross.y(), cross.z(),
                 (tk - tr_inner).norm() - L_tr_;
            return r;
        };
        constexpr double h = 1e-6;
        for (int it = 0; it < 60; ++it) {
            const Eigen::Vector4d r = residual4(aa0);
            if (r.norm() < 1e-9) break;
            Eigen::Matrix<double, 4, 3> J;
            for (int k = 0; k < 3; ++k) {
                Vec3 dp = Vec3::Zero();
                dp[k] = h;
                J.col(k) = (residual4(aa0 + dp) - r) / h;
            }
            const Eigen::Vector3d da =
                (J.transpose() * J).ldlt().solve(-J.transpose() * r);
            aa0 += da;
            if (da.norm() < 1e-10) break;
        }
        return aa0;
    }

    double solve_theta_for_travel(double travel, double steer_dy, Vec3& aa) const {
        const double target_z = wheel_static_.z() + travel;
        double theta = 0.0;
        for (int it = 0; it < 40; ++it) {
            aa = geometry_at(theta, steer_dy, aa).aa;
            const Vec3 w = geometry_at(theta, steer_dy, aa).wheel;
            const double err = w.z() - target_z;
            if (std::abs(err) < 1e-8) break;
            const double dth = 1e-5;
            const double slope =
                (geometry_at(theta + dth, steer_dy, aa).wheel.z() - w.z()) / dth;
            if (std::abs(slope) < 1e-12) break;
            theta -= err / slope;
        }
        return theta;
    }

    WheelPose pose_at(double theta, double steer_dy, const Vec3& aa) const {
        const auto g = geometry_at(theta, steer_dy, aa);
        return pose_from_spin(g.wheel, g.spin, wheel_static_, side_, g.sk, strut_top_);
    }

    std::string side_;
    Vec3 wheel_static_, wheel_spin_axis_;
    Vec3 lca_cf_, lca_cr_, lca_axis_, lca_pivot_, lca_knuckle_static_;
    Vec3 strut_top_, strut_bottom_, tube_axis_body_;
    Vec3 tr_inner_static_, tr_knuckle_static_;
    Vec3 off_sk_, off_tk_, off_wheel_;
    double L_tr_ {0.0};
    double I_theta_ {1.0};
};

class DoubleWishboneHardJointDae final : public IHardJointDaeModel {
public:
    explicit DoubleWishboneHardJointDae(const std::string& yaml_path,
                                        const SuspensionTopology& topo) {
        YAML::Node root = YAML::LoadFile(yaml_path);
        side_ = root["side"] ? root["side"].as<std::string>() : "left";
        wheel_static_ = yaml_vec3(root["wheel"]["center"]);
        wheel_spin_axis_ = yaml_vec3(root["wheel"]["spin_axis"]);
        lca_cf_ = yaml_vec3(root["lca"]["chassis_front"]);
        lca_cr_ = yaml_vec3(root["lca"]["chassis_rear"]);
        lca_axis_ = (lca_cr_ - lca_cf_).normalized();
        lca_pivot_ = lca_cf_;
        lca_knuckle_static_ = yaml_vec3(root["lca"]["knuckle"]);
        uca_cf_ = yaml_vec3(root["uca"]["chassis_front"]);
        uca_cr_ = yaml_vec3(root["uca"]["chassis_rear"]);
        uca_axis_ = (uca_cr_ - uca_cf_).normalized();
        uca_pivot_ = uca_cf_;
        uca_knuckle_static_ = yaml_vec3(root["uca"]["knuckle"]);
        L_LU_ = (uca_knuckle_static_ - lca_knuckle_static_).norm();
        tr_inner_static_ = yaml_vec3(root["tie_rod"]["rack"]);
        tr_knuckle_static_ = yaml_vec3(root["tie_rod"]["knuckle"]);
        L_tr_ = (tr_knuckle_static_ - tr_inner_static_).norm();
        L_LT_ = (tr_knuckle_static_ - lca_knuckle_static_).norm();
        L_UT_ = (tr_knuckle_static_ - uca_knuckle_static_).norm();
        const Vec3 ax0 = (uca_knuckle_static_ - lca_knuckle_static_).normalized();
        const Vec3 tk_off0 = tr_knuckle_static_ - lca_knuckle_static_;
        const Vec3 ay0_raw = tk_off0 - ax0.dot(tk_off0) * ax0;
        const Vec3 ay0 = ay0_raw / std::max(1e-9, ay0_raw.norm());
        const Vec3 az0 = ax0.cross(ay0);
        R0_.col(0) = ax0;
        R0_.col(1) = ay0;
        R0_.col(2) = az0;
        wheel_off_local_ = R0_.transpose() * (wheel_static_ - lca_knuckle_static_);
        I_theta_ = corner_inertia_about_axis(topo.bodies, lca_axis_, lca_pivot_);
    }

    void initialize(HardJointCornerState& st,
                    const PrescribedCornerMotion& mot) const override {
        st.q = solve_theta_for_travel(mot.travel_z, mot.steer_rack_dy);
        st.qd = 0.0;
        st.knuckle_aa.setZero();
    }

    WheelPose step(HardJointCornerState& st, const PrescribedCornerMotion& mot,
                   const WheelLoad& load, double dt) const override {
        if (dt <= 0.0) return pose_at(st.q, mot.steer_rack_dy);
        const double z_target = wheel_static_.z() + mot.travel_z;
        KinState geom;
        if (!kinematic_at(st.q, mot.steer_rack_dy, geom)) return pose_at(st.q, mot.steer_rack_dy);
        // CENTRAL differences for the travel Jacobian w'(=J) and curvature w''(=phi_qq).
        // w' was a forward difference (O(dth) biased); central is O(dth^2). The 2nd
        // difference reuses the same two side points (no extra solves — the old code
        // re-evaluated q+dth a second time). dth=1e-3 keeps the curvature well clear of
        // any inner-solver noise floor (each wheel_pos comes from an LM solve); in
        // practice the solver is smooth enough that 1e-5 and 1e-3 agree, so this is a
        // robustness/cleanup change, not a behavioral one.
        constexpr double dth = 1e-3;
        KinState geom_p, geom_m;
        if (!kinematic_at(st.q + dth, mot.steer_rack_dy, geom_p))
            return pose_at(st.q, mot.steer_rack_dy);
        const bool have_m = kinematic_at(st.q - dth, mot.steer_rack_dy, geom_m);
        const Vec3 J = have_m
            ? Vec3((geom_p.wheel_pos - geom_m.wheel_pos) / (2.0 * dth))
            : Vec3((geom_p.wheel_pos - geom.wheel_pos) / dth);
        const double phi_q = J.z();
        const double phi = geom.wheel_pos.z() - z_target;
        const double phi_d = phi_q * st.qd - mot.travel_z_dot;
        const double phi_qq = have_m
            ? (geom_p.wheel_pos.z() - 2.0 * geom.wheel_pos.z() + geom_m.wheel_pos.z())
              / (dth * dth)
            : 0.0;
        const double tau = J.dot(load.force_world);
        const auto sol = solve_constrained_revolute(
            I_theta_, phi_q, tau, phi, phi_d,
            mot.travel_z_ddot - phi_qq * st.qd * st.qd);
        const int nsub = std::max(1, static_cast<int>(std::ceil(dt / 0.001)));
        const double h = dt / static_cast<double>(nsub);
        for (int i = 0; i < nsub; ++i) {
            st.qd += h * sol.qdd;
            st.q += h * st.qd;
        }
        return pose_at(st.q, mot.steer_rack_dy);
    }

private:
    struct KinState {
        Vec3 lk, uk, tk, wheel_pos, spin_axis_world;
        Mat3 R_now;
    };

    Vec3 lca_knuckle_at(double theta) const {
        return lca_pivot_ + rodrigues(lca_axis_, theta) * (lca_knuckle_static_ - lca_pivot_);
    }

    Vec3 uca_knuckle_at(double theta) const {
        return uca_pivot_ + rodrigues(uca_axis_, theta) * (uca_knuckle_static_ - uca_pivot_);
    }

    bool kinematic_at(double theta_l, double steer_dy, KinState& out) const {
        out.lk = lca_knuckle_at(theta_l);
        double theta_u = 0.0;
        for (int it = 0; it < 30; ++it) {
            const Vec3 uk = uca_knuckle_at(theta_u);
            const double d = (uk - out.lk).norm();
            const double err = d - L_LU_;
            if (std::abs(err) < 1e-7) {
                out.uk = uk;
                goto have_uca;
            }
            constexpr double dth = 1e-5;
            const double slope = ((uca_knuckle_at(theta_u + dth) - out.lk).norm() - d) / dth;
            if (std::abs(slope) < 1e-9) break;
            theta_u -= err / slope;
        }
        out.uk = uca_knuckle_at(theta_u);
    have_uca:;
        const Vec3 tr_inner = tr_inner_static_ + Vec3(0.0, steer_dy, 0.0);
        if (!trilaterate3(out.lk, L_LT_, out.uk, L_UT_, tr_inner, L_tr_,
                          tr_knuckle_static_, out.tk))
            return false;
        const Vec3 ax = (out.uk - out.lk).normalized();
        const Vec3 tk_off = out.tk - out.lk;
        const Vec3 ay_raw = tk_off - ax.dot(tk_off) * ax;
        const Vec3 ay = ay_raw / std::max(1e-9, ay_raw.norm());
        const Vec3 az = ax.cross(ay);
        out.R_now.col(0) = ax;
        out.R_now.col(1) = ay;
        out.R_now.col(2) = az;
        out.wheel_pos = out.lk + out.R_now * wheel_off_local_;
        const Mat3 R_delta = out.R_now * R0_.transpose();
        out.spin_axis_world = R_delta * wheel_spin_axis_;
        return true;
    }

    double solve_theta_for_travel(double travel, double steer_dy) const {
        const double target_z = wheel_static_.z() + travel;
        double theta = 0.0;
        for (int it = 0; it < 40; ++it) {
            KinState s;
            if (!kinematic_at(theta, steer_dy, s)) break;
            const double err = s.wheel_pos.z() - target_z;
            if (std::abs(err) < 1e-8) break;
            constexpr double dth = 1e-5;
            KinState sp;
            if (!kinematic_at(theta + dth, steer_dy, sp)) break;
            const double slope = (sp.wheel_pos.z() - s.wheel_pos.z()) / dth;
            if (std::abs(slope) < 1e-12) break;
            theta -= err / slope;
        }
        return theta;
    }

    WheelPose pose_at(double theta, double steer_dy) const {
        KinState s;
        if (!kinematic_at(theta, steer_dy, s)) return {};
        WheelPose wp = pose_from_spin(s.wheel_pos, s.spin_axis_world, wheel_static_, side_,
                                      s.uk - s.lk);
        return wp;
    }

    std::string side_;
    Vec3 wheel_static_, wheel_spin_axis_;
    Vec3 lca_cf_, lca_cr_, lca_axis_, lca_pivot_, lca_knuckle_static_;
    Vec3 uca_cf_, uca_cr_, uca_axis_, uca_pivot_, uca_knuckle_static_;
    Vec3 tr_inner_static_, tr_knuckle_static_;
    double L_LU_ {0}, L_LT_ {0}, L_UT_ {0}, L_tr_ {0};
    Mat3 R0_;
    Vec3 wheel_off_local_;
    double I_theta_ {1.0};
};

class FiveLinkHardJointDae final : public IHardJointDaeModel {
public:
    explicit FiveLinkHardJointDae(const std::string& yaml_path,
                                  const SuspensionTopology& topo) {
        YAML::Node root = YAML::LoadFile(yaml_path);
        side_ = root["side"] ? root["side"].as<std::string>() : "left";
        wheel_static_ = yaml_vec3(root["wheel"]["center"]);
        wheel_spin_axis_ = yaml_vec3(root["wheel"]["spin_axis"]);
        static constexpr const char* LINK_NAMES[5] = {
            "upper_fore", "upper_aft", "lower_fore", "lower_aft", "toe_link",
        };
        const YAML::Node& links = root["links"];
        for (int i = 0; i < 5; ++i) {
            const auto& L = links[LINK_NAMES[i]];
            chassis_pts_[i] = yaml_vec3(L["chassis"]);
            knuckle_pts_static_[i] = yaml_vec3(L["knuckle"]);
            link_len_[i] = (knuckle_pts_static_[i] - chassis_pts_[i]).norm();
            knuckle_off_[i] = knuckle_pts_static_[i] - wheel_static_;
        }
        la_chassis_ = chassis_pts_[3];
        la_off_chassis_ = knuckle_pts_static_[3] - la_chassis_;
        la_axis_ = Vec3::UnitY();
        I_theta_ = corner_inertia_about_axis(topo.bodies, la_axis_, la_chassis_);
    }

    void initialize(HardJointCornerState& st,
                    const PrescribedCornerMotion& mot) const override {
        const auto g = full_pose_at(mot.travel_z, st.knuckle_aa);
        st.knuckle_aa = g.aa;
        st.q = lower_aft_theta_from_attach(g.la_attach);
        st.qd = 0.0;
    }

    WheelPose step(HardJointCornerState& st, const PrescribedCornerMotion& mot,
                   const WheelLoad& load, double dt) const override {
        if (dt <= 0.0) {
            const auto g = full_pose_at(mot.travel_z, st.knuckle_aa);
            st.knuckle_aa = g.aa;
            st.q = lower_aft_theta_from_attach(g.la_attach);
            return pose_from_spin(g.wheel, g.spin, wheel_static_, side_);
        }
        const double z_target = wheel_static_.z() + mot.travel_z;
        const auto geom = driven_pose_at(st.q, st.knuckle_aa);
        constexpr double dth = 1e-5;
        const auto geom_p = driven_pose_at(st.q + dth, geom.aa);
        const Vec3 J = (geom_p.wheel - geom.wheel) / dth;
        const double phi_q = J.z();
        const double phi = geom.wheel.z() - z_target;
        const double phi_d = phi_q * st.qd - mot.travel_z_dot;
        const auto geom_m = driven_pose_at(st.q - dth, geom.aa);
        const double phi_qq = (geom_p.wheel.z() - 2.0 * geom.wheel.z() + geom_m.wheel.z())
                            / (dth * dth);
        const double tau = J.dot(load.force_world);
        const auto sol = solve_constrained_revolute(
            I_theta_, phi_q, tau, phi, phi_d,
            mot.travel_z_ddot - phi_qq * st.qd * st.qd);
        const int nsub = std::max(1, static_cast<int>(std::ceil(dt / 0.001)));
        const double h = dt / static_cast<double>(nsub);
        for (int i = 0; i < nsub; ++i) {
            st.qd += h * sol.qdd;
            st.q += h * st.qd;
            st.knuckle_aa = driven_pose_at(st.q, st.knuckle_aa).aa;
        }
        const auto g = driven_pose_at(st.q, st.knuckle_aa);
        return pose_from_spin(g.wheel, g.spin, wheel_static_, side_);
    }

private:
    struct Geom {
        Vec3 wheel, spin, aa, la_attach;
    };

    Vec3 lower_aft_attach_at(double theta) const {
        return la_chassis_ + rodrigues(la_axis_, theta) * la_off_chassis_;
    }

    double lower_aft_theta_from_attach(const Vec3& la_world) const {
        const Vec3 dr = la_world - la_chassis_;
        const double ox = la_off_chassis_.x();
        const double oz = la_off_chassis_.z();
        return std::atan2(ox * dr.z() - oz * dr.x(), ox * dr.x() + oz * dr.z());
    }

    Geom full_pose_at(double travel_z, Vec3 aa0) const {
        Eigen::VectorXd pose(6);
        pose << wheel_static_.x(), wheel_static_.y(), wheel_static_.z() + travel_z,
                 aa0.x(), aa0.y(), aa0.z();
        const double target_z = wheel_static_.z() + travel_z;
        auto residuals = [&](const Eigen::VectorXd& v) -> Eigen::VectorXd {
            const Vec3 pos(v(0), v(1), v(2));
            const Vec3 aa(v(3), v(4), v(5));
            const Mat3 R = axis_angle_to_R(aa);
            Eigen::VectorXd r(6);
            for (int i = 0; i < 5; ++i) {
                const Vec3 k = pos + R * knuckle_off_[i];
                r(i) = (k - chassis_pts_[i]).norm() - link_len_[i];
            }
            r(5) = pos.z() - target_z;
            return r;
        };
        const Eigen::VectorXd solved = lm_solve(residuals, pose);
        Geom g;
        g.wheel = Vec3(solved(0), solved(1), solved(2));
        g.aa = Vec3(solved(3), solved(4), solved(5));
        const Mat3 R = axis_angle_to_R(g.aa);
        g.la_attach = g.wheel + R * knuckle_off_[3];
        g.spin = R * wheel_spin_axis_;
        return g;
    }

    Geom driven_pose_at(double theta, Vec3 aa0) const {
        const Vec3 p_la = lower_aft_attach_at(theta);
        Eigen::VectorXd pose(6);
        pose << wheel_static_.x(), wheel_static_.y(), wheel_static_.z(),
                 aa0.x(), aa0.y(), aa0.z();
        auto residuals = [&](const Eigen::VectorXd& v) -> Eigen::VectorXd {
            const Vec3 pos(v(0), v(1), v(2));
            const Vec3 aa(v(3), v(4), v(5));
            const Mat3 R = axis_angle_to_R(aa);
            Eigen::VectorXd r(7);
            const Vec3 la_pt = pos + R * knuckle_off_[3];
            r(0) = la_pt.x() - p_la.x();
            r(1) = la_pt.y() - p_la.y();
            r(2) = la_pt.z() - p_la.z();
            int row = 3;
            for (int i = 0; i < 5; ++i) {
                if (i == 3) continue;
                const Vec3 k = pos + R * knuckle_off_[i];
                r(row++) = (k - chassis_pts_[i]).norm() - link_len_[i];
            }
            return r;
        };
        const Eigen::VectorXd solved = lm_solve(residuals, pose);
        Geom g;
        g.wheel = Vec3(solved(0), solved(1), solved(2));
        g.aa = Vec3(solved(3), solved(4), solved(5));
        g.la_attach = p_la;
        g.spin = axis_angle_to_R(g.aa) * wheel_spin_axis_;
        return g;
    }

    std::string side_;
    Vec3 wheel_static_, wheel_spin_axis_;
    Vec3 la_chassis_, la_off_chassis_, la_axis_;
    std::array<Vec3, 5> chassis_pts_, knuckle_pts_static_, knuckle_off_;
    std::array<double, 5> link_len_;
    double I_theta_ {1.0};
};

class KinematicHardJointDae final : public IHardJointDaeModel {
public:
    explicit KinematicHardJointDae(SuspensionTopology topo)
        : topo_(std::move(topo)), kin_(create_kinematic_solver()) {}

    void initialize(HardJointCornerState& st,
                    const PrescribedCornerMotion& mot) const override {
        (void)st;
        (void)mot;
    }

    WheelPose step(HardJointCornerState& st, const PrescribedCornerMotion& mot,
                   const WheelLoad& load, double dt) const override {
        (void)st;
        (void)load;
        (void)dt;
        return kin_->forward_kinematics(topo_, mot.travel_z, mot.steer_rack_dy);
    }

private:
    mutable SuspensionTopology topo_;
    std::unique_ptr<IMultibodySolver> kin_;
};

}  // namespace

std::unique_ptr<IHardJointDaeModel>
create_hard_joint_dae_model(const SuspensionTopology& topo) {
    if (topo.kin_yaml_path.empty())
        throw std::runtime_error("mb: hard joint DAE requires kin_yaml_path");
    switch (topo.kind) {
        case TopologyKind::TrailingArm:
            return std::make_unique<TrailingArmHardJointDae>(topo.kin_yaml_path, topo);
        case TopologyKind::MacPherson:
            return std::make_unique<MacPhersonHardJointDae>(topo.kin_yaml_path, topo);
        case TopologyKind::DoubleWishbone:
            return std::make_unique<DoubleWishboneHardJointDae>(topo.kin_yaml_path, topo);
        case TopologyKind::MultiLink5:
            return std::make_unique<FiveLinkHardJointDae>(topo.kin_yaml_path, topo);
        default: {
            auto copy = topo;
            return std::make_unique<KinematicHardJointDae>(std::move(copy));
        }
    }
}

void step_hard_joint_dae(HardJointCornerState& state,
                         IHardJointDaeModel& model,
                         SuspensionTopology& topo,
                         const PrescribedCornerMotion& mot,
                         const WheelLoad& load,
                         double dt) {
    const WheelPose wp = model.step(state, mot, load, dt);
    topo.toe_deg = wp.toe_rad * 180.0 / M_PI;
    topo.camber_deg = wp.camber_rad * 180.0 / M_PI;
    topo.caster_deg = wp.caster_rad * 180.0 / M_PI;
    topo.compliance_toe_deg = 0.0;
    topo.compliance_camber_deg = 0.0;
}

}  // namespace vdsim::mb
