// gui/davis/imu_pose.h — IMU attitude estimation for the IMU window.
//
// Gyro-dominant design modeled on dv-processing's RotationIntegrator
// (the reference has NO accel-fused attitude estimator: it integrates
// the gyroscope alone and takes the gyro bias as a CONSTANT offset).
// We adopt that model and measure the constant ourselves: the first
// ~1 s of qualified (still, ~1 g) samples gives the bias, subtracted
// from every sample afterwards. On top of it sits a weak, MOTION-GATED
// roll/pitch correction from the accelerometer gravity reference
// (Mahony et al., "Nonlinear complementary filters on the special
// orthogonal group", IEEE TAC 2008 — proportional term only).
//
// The Mahony INTEGRAL (online bias tracking) is deliberately absent:
// during motion the accelerometer measures specific force, not gravity,
// so a bias tracker fed from the accel error is corrupted exactly when
// the camera moves — observed on the real DAVIS346 as a large
// closed-path residual. A constant offset cannot be corrupted; its
// cost is slow yaw wander from the residual (0.01–0.05 deg/s), and
// yaw has no absolute reference anyway (no magnetometer).
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

    /// Drops the attitude, the bias and the capture window.
    void reset() {
        w_ = 1; x_ = 0; y_ = 0; z_ = 0;
        bias_x_ = bias_y_ = bias_z_ = 0;
        sum_ax_ = sum_ay_ = sum_az_ = 0;
        sum_gx_ = sum_gy_ = sum_gz_ = 0;
        capture_n_ = 0;
        capture_t0_ = -1;
        aligned_ = false;
        last_t_ = -1;
    }

    /// Feeds one IMU sample (dt taken from the sample timestamps).
    void update(const ImuSample& s) {
        const double ax = s.accel_x, ay = s.accel_y, az = s.accel_z;
        const double amag = std::sqrt(ax * ax + ay * ay + az * az);
        const bool gravity_ok = amag > 0.7 && amag < 1.3;
        const double gm =
            std::sqrt(s.gyro_x * s.gyro_x + s.gyro_y * s.gyro_y +
                      s.gyro_z * s.gyro_z);

        // Phase 1 — bias capture (dv's constant gyroscopeOffset, measured
        // instead of passed in): accumulate a ~1 s still window. Any sample
        // that is not near-1 g or not still restarts the window.
        if (!aligned_) {
            if (!gravity_ok || gm >= kStillGyroDps) {
                capture_n_ = 0;
                sum_ax_ = sum_ay_ = sum_az_ = 0;
                sum_gx_ = sum_gy_ = sum_gz_ = 0;
                capture_t0_ = -1;
                return;
            }
            if (capture_n_ == 0) capture_t0_ = s.t;
            sum_ax_ += ax; sum_ay_ += ay; sum_az_ += az;
            sum_gx_ += s.gyro_x; sum_gy_ += s.gyro_y; sum_gz_ += s.gyro_z;
            ++capture_n_;
            const bool span_ok = s.t - capture_t0_ >= kBiasWindowUs &&
                                 capture_n_ >= kBiasWindowMinSamples;
            if (!span_ok) return;
            const double n = static_cast<double>(capture_n_);
            bias_x_ = sum_gx_ / n;
            bias_y_ = sum_gy_ / n;
            bias_z_ = sum_gz_ / n;
            // Initial alignment: orient the sensor frame so the estimated
            // body-frame up (= R^T · (0,0,1), what the correction compares
            // against the accel) equals the mean measured gravity direction.
            // That is the rotation taking the measurement TO world up:
            // axis = m × z.
            const double mx = sum_ax_ / n, my = sum_ay_ / n, mz = sum_az_ / n;
            const double mn = std::sqrt(mx * mx + my * my + mz * mz);
            const double nx = mx / mn, ny = my / mn, nz = mz / mn;
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
            last_t_ = s.t;
            return;
        }

        // Phase 2 — tracking.
        const double dt = static_cast<double>(s.t - last_t_) / 1e6;
        last_t_ = s.t;
        if (dt <= 0 || dt > 0.2) return;

        // Gyro with the constant offset removed (rad/s).
        double wx = (s.gyro_x - bias_x_) * kDeg2Rad;
        double wy = (s.gyro_y - bias_y_) * kDeg2Rad;
        double wz = (s.gyro_z - bias_z_) * kDeg2Rad;

        // Weak gravity correction for roll/pitch, trusted only when the
        // accelerometer actually measures gravity (near 1 g) AND the chip
        // is not rotating fast (above the gate the accel reads specific
        // force and its direction is meaningless). No integral — see the
        // header comment.
        if (gravity_ok && gm < kGyroGateDps) {
            const double am = amag;
            // Estimated direction of gravity in the body frame.
            const double vx = 2.0 * (x_ * z_ - w_ * y_);
            const double vy = 2.0 * (w_ * x_ + y_ * z_);
            const double vz = w_ * w_ - x_ * x_ - y_ * y_ + z_ * z_;
            // Error = cross(measured_accel_normalised, estimated_gravity).
            const double mx = ax / am, my = ay / am, mz = az / am;
            const double ex = my * vz - mz * vy;
            const double ey = mz * vx - mx * vz;
            const double ez = mx * vy - my * vx;
            wx += two_kp_ * ex;
            wy += two_kp_ * ey;
            wz += two_kp_ * ez;
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
    /// The captured constant gyro offset (deg/s) — diagnostics; zero until
    /// the still-window capture completes.
    [[nodiscard]] double bias_x_dps() const { return bias_x_; }
    [[nodiscard]] double bias_y_dps() const { return bias_y_; }
    [[nodiscard]] double bias_z_dps() const { return bias_z_; }

private:
    struct Q4 {
        double w, x, y, z;
    };
    static constexpr double kDeg2Rad = M_PI / 180.0;
    /// Capture-phase gates: near-1 g accel and < 10 deg/s gyro (rest +
    /// handheld tremor pass; any real motion restarts the window).
    static constexpr double kStillGyroDps = 10.0;
    /// Correction gate: above this rotation rate the accel correction is
    /// suppressed (the gyro rules; specific force is meaningless).
    static constexpr double kGyroGateDps = 60.0;
    /// Bias window: ≥ 1 s span AND ≥ 200 samples (rate-independent floor).
    static constexpr std::int64_t kBiasWindowUs = 1000000;
    static constexpr std::size_t kBiasWindowMinSamples = 200;

    static Q4 qmul(const Q4& a, const Q4& b) {
        return {a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
                a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
                a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
                a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
    }

    double w_{1}, x_{0}, y_{0}, z_{0};
    double bias_x_{0}, bias_y_{0}, bias_z_{0};
    double sum_ax_{0}, sum_ay_{0}, sum_az_{0};
    double sum_gx_{0}, sum_gy_{0}, sum_gz_{0};
    std::size_t capture_n_{0};
    std::int64_t capture_t0_{-1};
    double two_kp_{4.0};
    bool aligned_{false};
    std::int64_t last_t_{-1};
};

} // namespace gui::davis

#endif // GUI_DAVIS_IMU_POSE_H
