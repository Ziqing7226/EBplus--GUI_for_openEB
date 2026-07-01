// algo/common/kmeans.h — KMeans clustering for 2D points.
//
// Used for event color quantization, trajectory clustering, and cluster seed
// initialisation for object trackers. Standard Lloyd's algorithm with k-means++
// seeding. Header-only, no OpenCV dependency for portability within algo/.

#ifndef GUI_ALGO_COMMON_KMEANS_H
#define GUI_ALGO_COMMON_KMEANS_H

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <random>
#include <vector>

namespace gui_algo {

/// @brief 2D point used by the clustering utilities.
struct Point2D {
    double x{0.0};
    double y{0.0};
    Point2D() = default;
    Point2D(double x_, double y_) : x(x_), y(y_) {}
};

/// @brief KMeans clustering for 2D points (Lloyd's algorithm + k-means++ init).
class KMeans {
public:
    /// @brief Constructs the clusterer.
    /// @param k Number of clusters.
    /// @param max_iters Maximum iterations.
    /// @param seed RNG seed for reproducible k-means++ initialisation.
    KMeans(std::size_t k, std::size_t max_iters = 100, std::uint64_t seed = 42)
        : k_(k == 0 ? 1 : k), max_iters_(max_iters), rng_(seed) {}

    /// @brief Fits the clusters to @p points.
    /// @return Final inertia (sum of squared distances to nearest centroid).
    double fit(const std::vector<Point2D>& points) {
        if (points.empty()) { centroids_.clear(); return 0.0; }
        if (points.size() <= k_) {
            centroids_.assign(points.begin(), points.end());
            return 0.0;
        }
        init_plus_plus(points);
        double prev_inertia = std::numeric_limits<double>::max();
        for (std::size_t iter = 0; iter < max_iters_; ++iter) {
            const double inertia = assign(points);
            update_centroids(points);
            if (std::abs(prev_inertia - inertia) < 1e-9 * prev_inertia) break;
            prev_inertia = inertia;
        }
        // Recompute inertia against the final centroids_ so the returned value
        // is consistent with the centroids callers observe (the loop body moves
        // centroids one step past the inertia used for the convergence test).
        return assign(points);
    }

    /// @brief Returns the cluster index (0..k-1) nearest to @p p.
    std::size_t predict(const Point2D& p) const {
        std::size_t best = 0;
        double best_d = std::numeric_limits<double>::max();
        for (std::size_t i = 0; i < centroids_.size(); ++i) {
            const double d = dist2(p, centroids_[i]);
            if (d < best_d) { best_d = d; best = i; }
        }
        return best;
    }

    const std::vector<Point2D>& centroids() const { return centroids_; }

private:
    static double dist2(const Point2D& a, const Point2D& b) {
        const double dx = a.x - b.x, dy = a.y - b.y;
        return dx * dx + dy * dy;
    }

    void init_plus_plus(const std::vector<Point2D>& points) {
        centroids_.clear();
        centroids_.reserve(k_);
        // First centroid: uniform random pick.
        std::uniform_int_distribution<std::size_t> uni(0, points.size() - 1);
        centroids_.push_back(points[uni(rng_)]);
        // Subsequent centroids: weighted by squared distance to nearest existing.
        std::vector<double> d2(points.size(), std::numeric_limits<double>::max());
        for (std::size_t c = 1; c < k_; ++c) {
            double sum = 0.0;
            for (std::size_t i = 0; i < points.size(); ++i) {
                double best = d2[i];
                for (const auto& ctr : centroids_) {
                    const double dd = dist2(points[i], ctr);
                    if (dd < best) best = dd;
                }
                d2[i] = best;
                sum += best;
            }
            if (sum <= 0.0) { // Degenerate: all points coincide with centroids.
                centroids_.push_back(points[uni(rng_)]);
                continue;
            }
            std::uniform_real_distribution<double> pick(0.0, sum);
            double target = pick(rng_);
            std::size_t chosen = points.size() - 1;
            for (std::size_t i = 0; i < points.size(); ++i) {
                target -= d2[i];
                if (target <= 0.0) { chosen = i; break; }
            }
            centroids_.push_back(points[chosen]);
        }
    }

    double assign(const std::vector<Point2D>& points) {
        labels_.assign(points.size(), 0);
        double inertia = 0.0;
        for (std::size_t i = 0; i < points.size(); ++i) {
            std::size_t best = 0;
            double best_d = std::numeric_limits<double>::max();
            for (std::size_t c = 0; c < centroids_.size(); ++c) {
                const double d = dist2(points[i], centroids_[c]);
                if (d < best_d) { best_d = d; best = c; }
            }
            labels_[i] = best;
            inertia += best_d;
        }
        return inertia;
    }

    void update_centroids(const std::vector<Point2D>& points) {
        std::vector<double> sx(centroids_.size(), 0.0), sy(centroids_.size(), 0.0);
        std::vector<std::size_t> counts(centroids_.size(), 0);
        for (std::size_t i = 0; i < points.size(); ++i) {
            const auto c = labels_[i];
            sx[c] += points[i].x;
            sy[c] += points[i].y;
            ++counts[c];
        }
        for (std::size_t c = 0; c < centroids_.size(); ++c) {
            if (counts[c] > 0) {
                centroids_[c].x = sx[c] / counts[c];
                centroids_[c].y = sy[c] / counts[c];
            }
        }
    }

    std::size_t k_;
    std::size_t max_iters_;
    std::mt19937_64 rng_;
    std::vector<Point2D> centroids_;
    std::vector<std::size_t> labels_;
};

} // namespace gui_algo

#endif // GUI_ALGO_COMMON_KMEANS_H
