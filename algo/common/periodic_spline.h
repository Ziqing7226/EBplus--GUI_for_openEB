// algo/common/periodic_spline.h — periodic cubic spline for trajectory smoothing.
//
// Inspired by jAER PeriodicSpline. Fits a periodic cubic spline to a closed
// trajectory (e.g. a circle/ellipse traced by a rotating target) and resamples
// it at uniform parameter values for smoothing / interpolation. The spline is
// represented in Hermite form (values + slopes m_i). The slopes solve the cyclic
// tridiagonal system  m_{i-1} + 4*m_i + m_{i+1} = 3*(y_{i+1}-y_{i-1})/step
// (uniform spacing), solved via the Sherman-Morrison formula so the result is
// truly C2-continuous across the wrap-around seam. Header-only.

#ifndef GUI_ALGO_COMMON_PERIODIC_SPLINE_H
#define GUI_ALGO_COMMON_PERIODIC_SPLINE_H

#include <cmath>
#include <cstddef>
#include <vector>

#include "kmeans.h"  // Point2D

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace gui_algo {

/// @brief Periodic cubic spline for closed-curve trajectory smoothing.
class PeriodicSpline {
public:
    /// @brief Fits the spline to @p points (treated as a closed polygon in order).
    /// @param xs,ys Coordinate arrays (must be equal length >= 3).
    void fit(const std::vector<double>& xs, const std::vector<double>& ys) {
        const std::size_t n = xs.size();
        if (n != ys.size() || n < 3) return;
        xs_ = xs;
        ys_ = ys;
        // Periodic parameter t in [0, 2π); uniform spacing.
        ts_.resize(n);
        const double step = 2.0 * M_PI / static_cast<double>(n);
        for (std::size_t i = 0; i < n; ++i) ts_[i] = static_cast<double>(i) * step;
        solve_periodic(xs_, cx_);
        solve_periodic(ys_, cy_);
    }

    /// @brief Evaluates the smoothed spline at parameter @p t (any real; wraps).
    Point2D evaluate(double t) const {
        const std::size_t n = ts_.size();
        if (n == 0) return Point2D();
        const double two_pi = 2.0 * M_PI;
        const double step = two_pi / static_cast<double>(n);
        // Wrap t into [0, 2π).
        t = t - two_pi * std::floor(t / two_pi);
        // Find segment [i, i+1) (with wrap-around for the last seam).
        std::size_t i = static_cast<std::size_t>(t / step);
        if (i >= n) i = n - 1;
        const std::size_t j = (i + 1) % n;
        const double t0 = ts_[i];
        const double u = (t - t0) / step;
        const double u2 = u * u, u3 = u2 * u;
        const double h00 = 2 * u3 - 3 * u2 + 1;
        const double h10 = u3 - 2 * u2 + u;
        const double h01 = -2 * u3 + 3 * u2;
        const double h11 = u3 - u2;
        const double x = h00 * xs_[i] + h10 * step * cx_[i] +
                         h01 * xs_[j] + h11 * step * cx_[j];
        const double y = h00 * ys_[i] + h10 * step * cy_[i] +
                         h01 * ys_[j] + h11 * step * cy_[j];
        return Point2D(x, y);
    }

    /// @brief Resamples the curve into @p m uniformly-spaced points.
    std::vector<Point2D> resample(std::size_t m) const {
        std::vector<Point2D> out;
        out.reserve(m);
        const double two_pi = 2.0 * M_PI;
        for (std::size_t i = 0; i < m; ++i) {
            out.push_back(evaluate(static_cast<double>(i) / m * two_pi));
        }
        return out;
    }

private:
    /// @brief Solves the cyclic tridiagonal slope system for a periodic cubic
    /// spline with uniform spacing. Computes slopes m[0..n-1] from values y[],
    /// where  m_{i-1} + 4*m_i + m_{i+1} = 3*(y_{i+1}-y_{i-1})/step  (cyclic).
    /// Uses the Sherman-Morrison formula to reduce the cyclic system to two
    /// ordinary tridiagonal solves (Thomas algorithm), giving C2 continuity
    /// across the wrap seam.
    void solve_periodic(const std::vector<double>& y, std::vector<double>& m) {
        const std::size_t n = y.size();
        if (n < 3) { m.assign(n, 0.0); return; }
        const double step = 2.0 * M_PI / static_cast<double>(n);
        // RHS: d[i] = 3*(y[i+1]-y[i-1])/step  (indices mod n).
        std::vector<double> d(n);
        for (std::size_t i = 0; i < n; ++i) {
            d[i] = 3.0 * (y[(i + 1) % n] - y[(i - 1 + n) % n]) / step;
        }
        // Cyclic matrix A has diagonal 4, sub/super 1, and corner entries
        // A[0][n-1] = beta = 1, A[n-1][0] = alpha = 1.
        // Decompose A = T' + u*v^T, where T' is the ordinary tridiagonal with
        // the corner contributions folded into the end diagonals
        // (b'[0] = 4 - beta = 3, b'[n-1] = 4 - alpha = 3), and
        // u = (beta, 0, ..., 0, alpha), v = (1, 0, ..., 0, 1).
        std::vector<double> a(n, 1.0), b(n, 4.0), c(n, 1.0);
        b[0] = 3.0;
        b[n - 1] = 3.0;
        std::vector<double> u(n, 0.0);
        u[0] = 1.0;       // beta
        u[n - 1] = 1.0;   // alpha
        // Solve T' * ysol = d  and  T' * zsol = u (Thomas algorithm).
        std::vector<double> ysol, zsol;
        thomas(a, b, c, d, ysol);
        thomas(a, b, c, u, zsol);
        // x = ysol - zsol * (v·ysol) / (1 + v·zsol), with v = (1,0,...,0,1).
        const double vy = ysol[0] + ysol[n - 1];
        const double vz = zsol[0] + zsol[n - 1];
        const double denom = 1.0 + vz;
        const double factor = (denom != 0.0) ? (vy / denom) : 0.0;
        m.resize(n);
        for (std::size_t i = 0; i < n; ++i) {
            m[i] = ysol[i] - zsol[i] * factor;
        }
    }

    /// @brief Thomas algorithm for an ordinary tridiagonal system T x = r.
    /// a[0] and c[n-1] are not used (phantom entries).
    void thomas(const std::vector<double>& a, const std::vector<double>& b,
                const std::vector<double>& c, const std::vector<double>& r,
                std::vector<double>& x) const {
        const std::size_t n = b.size();
        x.assign(n, 0.0);
        std::vector<double> cp(n), dp(n);
        cp[0] = c[0] / b[0];
        dp[0] = r[0] / b[0];
        for (std::size_t i = 1; i < n; ++i) {
            const double denom = b[i] - a[i] * cp[i - 1];
            cp[i] = (i < n - 1) ? (c[i] / denom) : 0.0;
            dp[i] = (r[i] - a[i] * dp[i - 1]) / denom;
        }
        x[n - 1] = dp[n - 1];
        for (std::ptrdiff_t i = static_cast<std::ptrdiff_t>(n) - 2; i >= 0; --i) {
            x[i] = dp[i] - cp[i] * x[i + 1];
        }
    }

    std::vector<double> ts_, xs_, ys_, cx_, cy_;
};

} // namespace gui_algo

#endif // GUI_ALGO_COMMON_PERIODIC_SPLINE_H
