// gui/davis/imu_pose.h — IMU attitude estimation for the IMU window.
//
// Gyro-dominant design modeled on dv-processing's RotationIntegrator
// (the reference has NO accel-fused attitude estimator: it integrates
// the gyroscope alone and takes the gyro bias as a CONSTANT offset,
// measured offline by its imu-bias-estimation utility — a static
// capture averaged over ~1 s). We keep that constant-offset model but
// refine the constant online, ONLY while the chip is provably still:
// near 1 g, below 10 deg/s, AND with the attitude already agreeing
// with gravity (tilt error small). Under those conditions gyro − bias
// is pure bias, so the estimate converges in seconds and keeps
// tracking temperature drift over long sessions; any real motion
// freezes it, so it cannot absorb the specific-force error that
// corrupts online bias trackers (hardware-proven failure mode on the
// DAVIS346: a Mahony integral driven from the accel error produced a
// large closed-path residual).
//
// On top sits a weak roll/pitch correction from the accelerometer gravity
// reference (Mahony et al., IEEE TAC 2008 — proportional term only),
// applied ONLY at rest (near 1 g AND below 10 deg/s — the same rest gate
// as the bias leak). Between rest points the attitude is PURE
// bias-subtracted gyro integration: during motion the accelerometer
// measures specific force, not gravity, and letting it touch the attitude
// feeds path error into the integrator and breaks loop closure (observed
// on the DAVIS346; the reference RotationIntegrator is "perfect" at
// closed paths for exactly the reason that it never lets the accel near
// the attitude). At rest the gravity reference re-anchors roll/pitch, so
// the display stays meaningful over long sessions while every motion
// segment stays gyro-pure. Yaw has no absolute reference (no
// magnetometer): alignment sets yaw = 0 and it then wanders at the
// residual-bias rate — disclosed physics, same in the reference.
//
// Initialization is instant: the first near-1 g sample aligns the
// attitude (bias starts at 0 and refines in the background; the accel
// correction already holds roll/pitch to ~bias/(2kp) ≈ 0.4 deg in the
// meantime).
//
// Chip-agnostic: consumes gyro (deg/s) and accel (g) samples, so it
// works unchanged for DAVIS346 and DVXplorer (the wire decoders
// already normalise units and axis order). Header-only and
// dependency-free so the math is unit-testable.

#ifndef GUI_DAVIS_IMU_POSE_H
#define GUI_DAVIS_IMU_POSE_H

#include <algorithm>
#include <cmath>

#include "imu_types.h"

namespace gui::davis {

class ImuPose {
public:
    /// @param kp proportional gain for the accel tilt feedback (per unit
    ///        cross-product error; higher = faster convergence, more noise)
    explicit ImuPose(double kp = 2.0) : two_kp_(2.0 * kp) {}

    /// Drops the attitude and the bias estimate.
    void reset() {
        w_ = 1; x_ = 0; y_ = 0; z_ = 0;
        bias_x_ = bias_y_ = bias_z_ = 0;
        still_run_us_ = 0;
        aligned_t_us_ = -1;
        aligned_ = false;
        last_t_ = -1;
    }

    /// Feeds one IMU sample (dt taken from the sample timestamps).
    void update(const ImuSample& s) {
        const double ax = s.accel_x, ay = s.accel_y, az = s.accel_z;
        const double amag = std::sqrt(ax * ax + ay * ay + az * az);
        const bool gravity_ok = amag > 0.7 && amag < 1.3;

        // Initial alignment: orient the sensor frame so the estimated
        // body-frame up (= R^T · (0,0,1), what the correction compares
        // against the accel) equals the measured gravity direction. That
        // is the rotation taking the measurement TO world up: axis = m × z.
        if (!aligned_) {
            if (!gravity_ok) return;
            const double nx = ax / amag, ny = ay / amag, nz = az / amag;
            const double angle = std::acos(std::clamp(nz, -1.0, 1.0));
            double axis_x = ny, axis_y = -nx;
            const double axis_norm = std::sqrt(axis_x * axis_x + axis_y * axis_y);
            if (axis_norm < 1e-12) {
                w_ = 1; x_ = y_ = z_ = 0;
            } else {
                const double half = angle / 2.0;
                axis_x /= axis_norm;
                axis_y /= axis_norm;
                w_ = std::cos(half);
                x_ = axis_x * std::sin(half);
                y_ = axis_y * std::sin(half);
                z_ = 0;
            }
            aligned_ = true;
            aligned_t_us_ = s.t;
            last_t_ = s.t;
            return;
        }

        const double dt = static_cast<double>(s.t - last_t_) / 1e6;
        last_t_ = s.t;
        if (dt <= 0 || dt > 0.2) return;

        const double gm =
            std::sqrt(s.gyro_x * s.gyro_x + s.gyro_y * s.gyro_y +
                      s.gyro_z * s.gyro_z);

        // Convert gyro to rad/s with the current offset removed.
        double wx = (s.gyro_x - bias_x_) * kDeg2Rad;
        double wy = (s.gyro_y - bias_y_) * kDeg2Rad;
        double wz = (s.gyro_z - bias_z_) * kDeg2Rad;

        // Estimated direction of gravity in the body frame.
        const double vx = 2.0 * (x_ * z_ - w_ * y_);
        const double vy = 2.0 * (w_ * x_ + y_ * z_);
        const double vz = w_ * w_ - x_ * x_ - y_ * y_ + z_ * z_;
        // Error = cross(measured_accel_normalised, estimated_gravity).
        double ex = 0, ey = 0, ez = 0;
        const bool at_rest = gravity_ok && gm < kRestGyroDps;
        if (at_rest) {
            const double mx = ax / amag, my = ay / amag, mz = az / amag;
            ex = my * vz - mz * vy;
            ey = mz * vx - mx * vz;
            ez = mx * vy - my * vx;
            wx += two_kp_ * ex;
            wy += two_kp_ * ey;
            wz += two_kp_ * ez;
        }

        // Bias refinement, frozen like the reference's constant offset.
        // The leak runs ONLY while (a) the chip is quasi-still by every
        // gate AND (b) one of: the initial warm-up since alignment (so the
        // zero-init estimate converges), or an unbroken >= 3 s park at
        // < 3 deg/s (a deliberate "put it down", which re-opens the leak
        // for temperature re-calibration). Mid-motion pauses and slow
        // rotations never sustain 3 s below 3 deg/s, so between parks the
        // bias is FROZEN — closed paths then close with pure
        // bias-subtracted integration, exactly like the reference.
        const bool park_sample = gravity_ok && gm < kParkGyroDps;
        if (park_sample) {
            still_run_us_ += static_cast<std::int64_t>(dt * 1e6);
        } else {
            still_run_us_ = 0;
        }
        const bool warmup = aligned_t_us_ >= 0 && s.t - aligned_t_us_ < kWarmupUs;
        if (at_rest && (warmup || still_run_us_ >= kParkHoldUs) &&
            std::sqrt(ex * ex + ey * ey + ez * ez) < kBiasTiltErrorGate) {
            const double a = dt / kBiasTauS;
            bias_x_ += (s.gyro_x - bias_x_) * a;
            bias_y_ += (s.gyro_y - bias_y_) * a;
            bias_z_ += (s.gyro_z - bias_z_) * a;
        }

        // Quaternion integration: q' = q + 0.5 * q (x) omega * dt.
        const Q4 q{w_, x_, y_, z_};
        const Q4 om{0, wx, wy, wz};
        const Q4 dq = qmul(q, om);
        w_ += 0.5 * dq.w * dt;
        x_ += 0.5 * dq.x * dt;
        y_ += 0.5 * dq.y * dt;
        z_ += 0.5 * dq.z * dt;
        const double n = std::sqrt(w_ * w_ + x_ * x_ + y_ * y_ + z_ * z_);
        w_ /= n; x_ /= n; y_ /= n; z_ /= n;
    }

    [[nodiscard]] bool aligned() const { return aligned_; }
    [[nodiscard]] double w() const { return w_; }
    [[nodiscard]] double x() const { return x_; }
    [[nodiscard]] double y() const { return y_; }
    [[nodiscard]] double z() const { return z_; }
    /// The current gyro-bias estimate (deg/s) — diagnostics; converges from
    /// 0 while the chip is still.
    [[nodiscard]] double bias_x_dps() const { return bias_x_; }
    [[nodiscard]] double bias_y_dps() const { return bias_y_; }
    [[nodiscard]] double bias_z_dps() const { return bias_z_; }

private:
    struct Q4 {
        double w, x, y, z;
    };
    static constexpr double kDeg2Rad = M_PI / 180.0;
    /// Rest gate shared by the tilt correction and the bias leak: below
    /// this rotation rate the chip counts as stationary (handheld tremor
    /// passes, any deliberate turn does not). BOTH accel paths are confined
    /// to rest — during motion the attitude is PURE bias-subtracted gyro
    /// integration, which is what makes closed paths return (the reference
    /// RotationIntegrator behaves this way; every accel touch during
    /// motion feeds specific force into the path and breaks the loop
    /// closure — observed on the DAVIS346).
    static constexpr double kRestGyroDps = 10.0;
    /// Bias leak additionally requires the attitude to already agree with
    /// gravity (cross-product error; ~2.9 deg), so a slow real rotation —
    /// which drags the tilt error up — cannot be absorbed as bias either.
    static constexpr double kBiasTiltErrorGate = 0.05;
    /// Bias leak time constant (s).
    static constexpr double kBiasTauS = 2.0;
    /// Park discrimination for re-opening the bias leak after the initial
    /// warm-up: the run counts only below this rotation rate (true park;
    /// handheld rest reads |bias| + tremor ~ 1.5-2 deg/s, deliberate slow
    /// rotations read more).
    static constexpr double kParkGyroDps = 3.0;
    /// The leak re-opens only after this much unbroken park (us). Mid-
    /// motion direction-reversal pauses are ~ 0.5-2 s and never reach it.
    static constexpr std::int64_t kParkHoldUs = 3000000;
    /// Initial warm-up after alignment during which the leak runs
    /// unconditionally (the zero-init estimate converges here).
    static constexpr std::int64_t kWarmupUs = 10000000;

    static Q4 qmul(const Q4& a, const Q4& b) {
        return {a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
                a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
                a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
                a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
    }

    double w_{1}, x_{0}, y_{0}, z_{0};
    double bias_x_{0}, bias_y_{0}, bias_z_{0};
    double two_kp_{4.0};
    bool aligned_{false};
    std::int64_t last_t_{-1};
    std::int64_t aligned_t_us_{-1};
    std::int64_t still_run_us_{0};
};

} // namespace gui::davis

#endif // GUI_DAVIS_IMU_POSE_H
