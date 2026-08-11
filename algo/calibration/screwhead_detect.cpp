// algo/calibration/screwhead_detect.cpp — see header.

#include "screwhead_detect.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace gui_algo {

namespace {

// Three-valued frame colours (BGR). Must match CalibrationWizard::render_event_frame.
const cv::Vec3b kGold(0, 215, 255);    // ON polarity
const cv::Vec3b kWhite(255, 255, 255); // OFF polarity
// kBlend = (kGold + kWhite) / 2, computed in render_event_frame.

// --- Joint cross+ring localisation parameters --------------------------------
//
// The detector jointly localises each marker via two PEER features of equal
// standing:
//   * the dashed CROSS (density peak via distance transform), and
//   * the solid RING (radial-histogram peak + angular-coverage test).
// Detecting EITHER feature means a possible marker centre was found. Each
// contributes an equal confidence (kCrossConfidence = kRingConfidence = 0.5)
// to the coordinate; the 8-neighbourhood of the final centre adds a further
// kNeighborConfidence = 0.2 (a centre surrounded by signal is more reliable
// than one in empty space). The grid fit is a weighted RANSAC: centres with
// both features (weight ≈ 1.0–1.2) anchor the grid, single-feature centres
// (weight ≈ 0.5–0.7) fill gaps.
//
// The old half/half polarity test was removed: under camera motion the ring
// edges fire OPPOSITE polarities at every angle (outer/inner, leading/trailing
// quadrants), so the "one semicircle all-ON, the other all-OFF" model never
// holds. Polarity is now only used to colour the frame; verification is purely
// geometric (ring signal strength + angular coverage).

// Angular resolution for the ring coverage test (bins around the circle).
constexpr int kRingAngleBins = 36;
// Local search radius (±px) around the seed centre when maximising the
// radial-histogram peak. Corrects peak shifts caused by the cross candidate
// being off-centre (distance-transform peak biased toward a thicker ring arc).
constexpr int kSeedSearch = 2;
// The ring acceptance knobs (min angular coverage kRingCoverFrac, min ring
// pixels, ring search margin) are runtime RingParams (defaults: 0.60 / 6 / 12)
// so the wizard's capture-review dialog can relax them and re-detect. The old
// constexpr thresholds moved into RingParams in intrinsic.h.

// Confidence weights for grid-fit voting. Cross and ring are PEER detections:
// each independently indicates a possible centre and contributes 50%. The
// 8-neighbourhood of the final centre contributes 20%. A marker with both
// cross+ring and neighbour support has weight 1.2; cross-only with support 0.7;
// ring-only 0.5. These replace the old confirmed(2.0)/suspected(1.0) split —
// the two features now have EQUAL standing, not a 2:1 ratio.
constexpr float kCrossConfidence    = 0.5f;
constexpr float kRingConfidence     = 0.5f;
constexpr float kNeighborConfidence = 0.2f;

// Match tolerance (fraction of grid spacing d) for grid fitting. 0.40 allows
// markers that are slightly off the predicted position (due to perspective
// distortion or d estimation error) to match, while staying well below 0.5·d
// (the half-cell distance) so a slot cannot match a neighbouring marker.
// Note: d is overestimated ~1.4× (radial-histogram peak shift), so the
// effective tolerance in true-d units is ~0.56·d_true, safely below 0.707·d_true
// (half the diagonal — the ambiguity threshold). Increasing this beyond 0.40
// risks selecting wrong grid hypotheses (false matches inflate the score of
// an incorrect placement, displacing predicted positions by >1 cell).
constexpr float kMatchTolFrac = 0.40f;
// Ring radius as a fraction of the grid half-cell spacing d (R = 0.20·d).
// Must match circle_grid_display's kMarkerRadiusFrac. Used to estimate d from
// the detected ring radius — more robust than nearest-neighbour distance.
constexpr float kRingRadiusFrac = 0.20f;

// Temporary diagnostic logging (env GUI_SCREW_DEBUG=1). Removed after analysis.
bool screw_debug() {
    static const bool d = std::getenv("GUI_SCREW_DEBUG") != nullptr;
    return d;
}
#define SDBG(...) do { if (screw_debug()) std::fprintf(stderr, __VA_ARGS__); } while (0)

// Foreground = non-black. CV_8U 0/255.
cv::Mat foreground_mask(const cv::Mat& color) {
    cv::Mat fg(color.size(), CV_8U);
    for (int y = 0; y < color.rows; ++y) {
        const cv::Vec3b* c = color.ptr<cv::Vec3b>(y);
        uchar* f = fg.ptr<uchar>(y);
        for (int x = 0; x < color.cols; ++x) {
            const cv::Vec3b& v = c[x];
            f[x] = (v[0] | v[1] | v[2]) ? 255 : 0;
        }
    }
    return fg;
}

// Remove truly isolated foreground pixels. A dashed-cross dot has its nearest
// neighbour at distance (1+dot_gap) along the arm, so the denoise kernel radius
// must be (dot_gap+1) to preserve cross dots while removing lone noise that has
// no neighbour within that radius. (A 3×3 / 8-neighbour kernel would erase every
// cross dot — they are 1px at period ≥2 — leaving only the ring and making the
// density-peak centre localisation unreliable.)
cv::Mat denoise_isolated(const cv::Mat& fg, int dot_gap) {
    const int r = std::max(1, dot_gap + 1);
    // Ellipse kernel of radius r with the centre zeroed: dilate with this kernel
    // sets a pixel iff at least one neighbour within radius r (excluding self) is
    // set. bitwise_and keeps fg pixels that pass this test.
    cv::Mat kernel = cv::getStructuringElement(
        cv::MORPH_ELLIPSE, cv::Size(2 * r + 1, 2 * r + 1));
    kernel.at<uchar>(r, r) = 0;
    cv::Mat nb;
    cv::dilate(fg, nb, kernel);
    cv::Mat out;
    cv::bitwise_and(fg, nb, out);
    return out;
}

// Stage 2: candidate cross centres. Dilate bridges the dashed-cross dot gaps so
// each screw-head becomes one connected component; the distance-transform peak
// within each component localises the cross centre — the point furthest from the
// background, which is the thickest spot (the cross intersection = marker centre).
// Unlike the area centroid this is not biased toward whichever ring arc happened
// to fire, and unlike a Gaussian-blur density (which is flat across the blob and
// picks the first pixel in scan order) the distance transform has a real ridge.
struct Candidate {
    cv::Point2f pos;
    int max_r;  // half the component bounding-box diagonal — limits ring search
};

std::vector<Candidate> find_cross_candidates(const cv::Mat& fg_denoised, int dot_gap,
                                             int ring_search_margin) {
    const int r_dilate = std::max(2, dot_gap + 1);
    cv::Mat kernel = cv::getStructuringElement(
        cv::MORPH_ELLIPSE, cv::Size(2 * r_dilate + 1, 2 * r_dilate + 1));
    cv::Mat dil;
    cv::dilate(fg_denoised, dil, kernel);

    cv::Mat labels, stats, centroids;
    int n = cv::connectedComponentsWithStats(dil, labels, stats, centroids, 8, CV_32S);

    // Centre localisation via the distance transform of the dilated image.
    cv::Mat dist;
    cv::distanceTransform(dil, dist, cv::DIST_L2, 3);  // CV_32F output

    std::vector<Candidate> cands;
    const int frame_area = dil.cols * dil.rows;
    for (int i = 1; i < n; ++i) {
        const int area = stats.at<int>(i, cv::CC_STAT_AREA);
        // Loose area filter: drop speckle noise and anything huge (merged rows).
        if (area < 8 || area > frame_area / 4) continue;
        const int lx = stats.at<int>(i, cv::CC_STAT_LEFT);
        const int ly = stats.at<int>(i, cv::CC_STAT_TOP);
        const int lw = stats.at<int>(i, cv::CC_STAT_WIDTH);
        const int lh = stats.at<int>(i, cv::CC_STAT_HEIGHT);

        cv::Point best(lx, ly);
        float bestv = -1.f;
        for (int y = ly; y < ly + lh && y < dist.rows; ++y) {
            const float* drow = dist.ptr<float>(y);
            const int* lrow = labels.ptr<int>(y);
            for (int x = lx; x < lx + lw && x < dist.cols; ++x) {
                if (lrow[x] != i) continue;
                if (drow[x] > bestv) { bestv = drow[x]; best = cv::Point(x, y); }
            }
        }
        // Limit the ring-search radius so the radial histogram does not pick up
        // NEIGHBOURING markers' rings (which would give a wrong best_r and fail
        // the coverage test). The raw half-diagonal is too tight — sparse events
        // shrink the cross component — so add the (user-tunable) search margin
        // to reach the ring (R ≈ 0.20·d, typically larger than the cross arm).
        const int max_r = std::max(lw, lh) / 2 + ring_search_margin;
        cands.push_back({cv::Point2f(float(best.x), float(best.y)), max_r});
    }
    return cands;
}

// Stage 3: ring localisation. Searches a small neighbourhood (±kSeedSearch px)
// around the seed (cross candidate) for the centre that maximises the radial-
// histogram peak, then checks the angular coverage of the ring pixels. A real
// solid ring covers nearly all angle bins; a noise arc or a merged blob covers
// few.
//
// Returns the ring centre (which may differ from the seed by a few px), the
// ring radius, and the coverage fraction. The caller combines this with the
// cross centre into a weighted Marker — cross and ring are peers, each
// contributing kCrossConfidence/kRingConfidence to the centre's confidence.
struct RingInfo {
    bool found = false;
    cv::Point2f center;
    float radius = 0.f;
    float coverage = 0.f;
    int ring_px = 0;
};

RingInfo find_ring(const cv::Mat& fg, const cv::Point2f& seed, int max_r,
                   const RingParams& ring) {
    RingInfo info;
    if (max_r < 4) return info;

    // Local search: try the seed and its ±kSeedSearch px neighbourhood, pick
    // the centre with the strongest radial-histogram peak. This corrects
    // radial-histogram peak shifts caused by the cross candidate being
    // off-centre (e.g. when the distance-transform peak is biased toward a
    // thicker ring arc).
    cv::Point2f best_center = seed;
    int best_peak = 0;
    int best_r = 0;
    for (int dy = -kSeedSearch; dy <= kSeedSearch; ++dy) {
        for (int dx = -kSeedSearch; dx <= kSeedSearch; ++dx) {
            const cv::Point2f c = seed + cv::Point2f(float(dx), float(dy));
            const int x0 = std::max(0, int(c.x) - max_r);
            const int x1 = std::min(fg.cols, int(c.x) + max_r + 1);
            const int y0 = std::max(0, int(c.y) - max_r);
            const int y1 = std::min(fg.rows, int(c.y) + max_r + 1);
            std::vector<int> hist(max_r + 2, 0);
            for (int y = y0; y < y1; ++y) {
                const uchar* f = fg.ptr<uchar>(y);
                for (int x = x0; x < x1; ++x) {
                    if (!f[x]) continue;
                    const float ddx = x - c.x, ddy = y - c.y;
                    const int r = int(std::sqrt(ddx * ddx + ddy * ddy) + 0.5f);
                    if (r >= 1 && r <= max_r) hist[r]++;
                }
            }
            for (int r = 3; r <= max_r - 1; ++r) {
                const int cnt = hist[r - 1] + hist[r] + hist[r + 1];
                if (cnt > best_peak) {
                    best_peak = cnt;
                    best_center = c;
                    best_r = r;
                }
            }
        }
    }
    if (best_r < 3 || best_peak < ring.min_pixels) {
        SDBG("  seed(%.1f,%.1f) max_r=%d REJECT(best_r=%d peak=%d<%d)\n",
             seed.x, seed.y, max_r, best_r, best_peak, ring.min_pixels);
        return info;
    }

    // Collect ring pixels (distance within tol of best_r) and compute angular
    // coverage — the fraction of angle bins that contain at least one ring pixel.
    const float tol = 2.f;
    std::vector<char> bin_hit(kRingAngleBins, 0);
    int ring_px = 0;
    const int x0 = std::max(0, int(best_center.x) - max_r);
    const int x1 = std::min(fg.cols, int(best_center.x) + max_r + 1);
    const int y0 = std::max(0, int(best_center.y) - max_r);
    const int y1 = std::min(fg.rows, int(best_center.y) + max_r + 1);
    for (int y = y0; y < y1; ++y) {
        const uchar* f = fg.ptr<uchar>(y);
        for (int x = x0; x < x1; ++x) {
            if (!f[x]) continue;
            const float dx = x - best_center.x, dy = y - best_center.y;
            const float dist = std::sqrt(dx * dx + dy * dy);
            if (std::abs(dist - best_r) > tol) continue;
            ++ring_px;
            const float ang = std::atan2(dy, dx);
            int b = int((ang + CV_PI) / (2 * CV_PI) * kRingAngleBins);
            if (b < 0) b = 0; else if (b >= kRingAngleBins) b = kRingAngleBins - 1;
            bin_hit[b] = 1;
        }
    }
    if (ring_px < ring.min_pixels) {
        SDBG("  seed(%.1f,%.1f) best_r=%d REJECT(ring_px=%d<%d)\n",
             seed.x, seed.y, best_r, ring_px, ring.min_pixels);
        return info;
    }
    int covered = 0;
    for (char h : bin_hit) covered += h;
    const float coverage = float(covered) / float(kRingAngleBins);
    SDBG("  seed(%.1f,%.1f) ring_center=(%.1f,%.1f) r=%d px=%d cover=%.2f %s\n",
         seed.x, seed.y, best_center.x, best_center.y, best_r, ring_px, coverage,
         coverage >= ring.cover_frac ? "OK" : "REJECT(coverage)");
    if (coverage < ring.cover_frac) return info;

    info.found = true;
    info.center = best_center;
    info.radius = float(best_r);
    info.coverage = coverage;
    info.ring_px = ring_px;
    return info;
}

// A detected marker: cross centre (density peak) and/or ring centre (local
// Hough optimum). Cross and ring are PEER features — detecting either one means
// a possible centre was found. Each contributes kCrossConfidence/kRingConfidence
// (50%) to the centre's confidence; the 8-neighbourhood of the final centre adds
// kNeighborConfidence (20%). The grid fit is a weighted RANSAC over these
// confidences: centres with both features (weight ≈ 1.0–1.2) anchor the grid,
// single-feature centres (weight ≈ 0.5–0.7) fill gaps.
struct Marker {
    cv::Point2f cross_pos;
    cv::Point2f ring_pos;
    float ring_radius = 0.f;
    float coverage = 0.f;
    bool has_cross = false;
    bool has_ring = false;
    // True when ≥1 foreground pixel lies in the 8-neighbourhood of best_pos —
    // a centre surrounded by signal is more reliable than one in empty space.
    bool has_neighbor_support = false;

    // Weighted confidence for grid-fit voting. Cross and ring contribute equally
    // (peers); the 8-neighbourhood adds a small boost.
    float weight() const {
        return (has_cross ? kCrossConfidence : 0.f)
             + (has_ring  ? kRingConfidence  : 0.f)
             + (has_neighbor_support ? kNeighborConfidence : 0.f);
    }

    // Best estimate of the marker centre.
    // Ring centre (local Hough optimum) is more accurate than the cross centre
    // (distance-transform peak, biased by sparse events), so prefer it when
    // available; fall back to the cross centre for cross-only markers.
    cv::Point2f best_pos() const {
        if (has_ring) return ring_pos;
        return cross_pos;
    }
};

// Median of a vector (caller ensures non-empty).
float median(std::vector<float> v) {
    std::sort(v.begin(), v.end());
    return v[v.size() / 2];
}

// Nearest-neighbour distances within a point set (each point excludes itself).
std::vector<float> nn_distances(const std::vector<cv::Point2f>& pts) {
    std::vector<float> nn;
    const int K = int(pts.size());
    if (K < 2) return nn;
    nn.reserve(K);
    for (int i = 0; i < K; ++i) {
        float best = 1e9f;
        for (int j = 0; j < K; ++j) {
            if (i == j) continue;
            const float dx = pts[i].x - pts[j].x;
            const float dy = pts[i].y - pts[j].y;
            const float d = std::sqrt(dx * dx + dy * dy);
            if (d < best) best = d;
        }
        if (best < 1e8f) nn.push_back(best);
    }
    return nn;
}

// Stage 4: fit the asymmetric cols×rows grid to the detected markers using
// weighted RANSAC voting. Cross and ring are peers: a centre with both features
// (weight ≈ 1.0–1.2) anchors the grid; a single-feature centre (weight ≈ 0.5–0.7)
// fills gaps. Returns true only if ALL cols×rows positions have a matching
// marker — gaps are NOT synthesised and NOT recovered by a second pass, because
// a position with no detected feature (neither cross nor ring) would corrupt the
// calibration. An empty slot simply fails the fit.
bool fit_asymmetric_grid(const std::vector<Marker>& markers,
                         int cols, int rows,
                         std::vector<cv::Point2f>* ordered) {
    const int M = int(markers.size());
    const int need = cols * rows;
    if (M < need) return false;  // cannot fill the grid without features

    // Best-position + weight arrays for the voting.
    std::vector<cv::Point2f> pts(M);
    std::vector<float> wts(M);
    for (int i = 0; i < M; ++i) {
        pts[i] = markers[i].best_pos();
        wts[i] = markers[i].weight();
    }

    // Estimate the grid half-cell spacing d from markers that have BOTH cross
    // and ring (the most reliable centres). The ring radius is R = 0.20·d, but
    // the radial-histogram peak is shifted outward by the ring thickness +
    // smoothing, so d = r/0.20 overestimates. Instead, filter joint points by
    // radius consistency (removes noise with outlying radii like r=80) and use
    // their nearest-neighbour (diagonal) distance: d = median_NN / √2.
    //
    // When many false-positive joint markers are present (common in noisy event
    // data), the raw NN median is corrupted — clustered false positives create
    // small NN distances that pull the median far below the true grid diagonal.
    // To guard against this, use the radius-based d estimate (d_radius, robust
    // because r_med is dominated by real markers) as a band-pass filter on the
    // NN distances: only keep NN distances within [0.5, 2.0]× the expected
    // diagonal d_radius·√2. This rejects the clustered-false-positive distances
    // while preserving the real-marker diagonal distances. Fall back to the
    // unfiltered NN median, then to the radius estimate, if too few survive.
    std::vector<float> joint_radii;
    std::vector<cv::Point2f> joint_pts;
    for (const auto& m : markers) {
        if (m.has_cross && m.has_ring) {
            joint_radii.push_back(m.ring_radius);
            joint_pts.push_back(m.best_pos());
        }
    }
    float d = 0.f;
    float r_med = 0.f;  // median joint ring radius — used for the radius fallback
    if (joint_pts.size() >= 4) {
        // Filter by radius consistency: keep points within [0.5, 2.0]× median
        // radius. This removes gross noise (r=80 when true r≈14) while keeping
        // real markers whose radius is slightly shifted.
        r_med = median(joint_radii);
        std::vector<cv::Point2f> clean_pts;
        for (size_t i = 0; i < joint_pts.size(); ++i) {
            if (joint_radii[i] >= r_med * 0.5f &&
                joint_radii[i] <= r_med * 2.0f) {
                clean_pts.push_back(joint_pts[i]);
            }
        }
        SDBG("[screw] fit: joint=%zu r_med=%.1f clean=%zu\n",
             joint_pts.size(), r_med, clean_pts.size());
        if (clean_pts.size() >= 4) {
            const std::vector<float> nn = nn_distances(clean_pts);
            if (!nn.empty()) {
                // Radius-based d estimate (overestimates ~40% due to peak shift,
                // so apply an empirical correction factor). Used as a band-pass
                // filter on NN distances to reject clustered-false-positive
                // distances that corrupt the raw median.
                const float d_radius = (r_med / kRingRadiusFrac) * 0.72f;
                const float dexp = d_radius * std::sqrt(2.0f);  // expected diagonal
                // Band-pass: keep NN distances in [0.5, 1.5]× the expected
                // diagonal. The lower bound rejects clustered-false-positive
                // distances; the upper bound rejects 2-step distances (markers
                // whose nearest grid neighbour is 2 cells away — common at grid
                // edges) that would inflate the median. d_radius overestimates
                // ~25%, so the true diagonal sits at ~0.8×dexp, well inside the
                // band.
                std::vector<float> nn_filtered;
                nn_filtered.reserve(nn.size());
                for (float v : nn) {
                    if (v >= 0.5f * dexp && v <= 1.5f * dexp)
                        nn_filtered.push_back(v);
                }
                SDBG("[screw] fit: nn=%zu filtered=%zu (dexp=%.1f)\n",
                     nn.size(), nn_filtered.size(), dexp);
                if (!nn_filtered.empty()) {
                    d = median(nn_filtered) / std::sqrt(2.0f);
                } else {
                    // No NN distances in the expected band — fall back to the
                    // raw median (may be corrupted, but better than nothing).
                    d = median(nn) / std::sqrt(2.0f);
                }
            }
        }
        // Fallback: radius estimate (overestimates ~40% due to peak shift, so
        // apply an empirical correction factor).
        if (d < 3.f) d = (r_med / kRingRadiusFrac) * 0.72f;
    } else {
        const auto& ref_pts = joint_pts.empty() ? pts : joint_pts;
        const std::vector<float> nn = nn_distances(ref_pts);
        if (nn.empty()) return false;
        d = median(nn) / std::sqrt(2.0f);
    }
    if (d < 3.f) return false;
    const float match_tol = kMatchTolFrac * d;
    SDBG("[screw] fit: M=%d joint=%zu d=%.1f match_tol=%.1f\n",
         M, joint_pts.size(), d, match_tol);

    // Collect candidate diagonal steps (pairs at ~d·√2). Weighted by the sum of
    // the two endpoints' weights so high-confidence (cross+ring) pairs dominate.
    const float D_diag = d * std::sqrt(2.0f);  // expected diagonal step length
    struct Step { cv::Point2f from; cv::Point2f s; float weight; };
    std::vector<Step> steps;
    for (int i = 0; i < M; ++i) {
        for (int j = i + 1; j < M; ++j) {
            const float dx = pts[j].x - pts[i].x;
            const float dy = pts[j].y - pts[i].y;
            const float dist = std::sqrt(dx * dx + dy * dy);
            if (dist >= 0.7f * D_diag && dist <= 1.3f * D_diag) {
                steps.push_back({pts[i], cv::Point2f(dx, dy), wts[i] + wts[j]});
            }
        }
    }
    if (steps.empty()) return false;
    // Try the highest-weight steps first (they are most likely true diagonals).
    std::sort(steps.begin(), steps.end(),
              [](const Step& a, const Step& b) { return a.weight > b.weight; });
    SDBG("[screw] fit: steps=%zu top_weight=%.1f\n", steps.size(), steps.front().weight);

    // Weighted match: for a given (origin, ex, ey) grid hypothesis, slide a
    // cols×rows window over a wide grid and find the placement with the highest
    // total vote weight. Also records which marker (if any) matches each slot.
    auto match_weighted = [&](const cv::Point2f& origin,
                              const cv::Point2f& ex, const cv::Point2f& ey,
                              int* c0_out, int* r0_out,
                              float* score_out) -> void {
        const int gc0 = -cols, gc1 = 2 * cols;
        const int gr0 = -rows, gr1 = 2 * rows;
        const int gw = gc1 - gc0 + 1;
        const int gh = gr1 - gr0 + 1;
        std::vector<float> hit_w(gw * gh, 0.f);
        for (int r = gr0; r <= gr1; ++r) {
            for (int c = gc0; c <= gc1; ++c) {
                const cv::Point2f p = origin +
                    ((2 * c + (r & 1)) * d) * ex + (r * d) * ey;
                float best_d2 = match_tol * match_tol;
                int best_k = -1;
                for (int k = 0; k < M; ++k) {
                    const float dx = pts[k].x - p.x;
                    const float dy = pts[k].y - p.y;
                    const float d2 = dx * dx + dy * dy;
                    if (d2 < best_d2) { best_d2 = d2; best_k = k; }
                }
                if (best_k >= 0) hit_w[(r - gr0) * gw + (c - gc0)] = wts[best_k];
            }
        }
        float best = 0.f; int bc0 = gc0, br0 = gr0;
        for (int r = gr0; r + rows - 1 <= gr1; ++r) {
            for (int c = gc0; c + cols - 1 <= gc1; ++c) {
                float w = 0.f;
                for (int rr = 0; rr < rows; ++rr)
                    for (int cc = 0; cc < cols; ++cc)
                        w += hit_w[(r - gr0 + rr) * gw + (c - gc0 + cc)];
                if (w > best) { best = w; bc0 = c; br0 = r; }
            }
        }
        if (c0_out) *c0_out = bc0;
        if (r0_out) *r0_out = br0;
        if (score_out) *score_out = best;
    };

    // Try each diagonal step as a grid-axis hypothesis. The match_weighted slide
    // resolves the placement. Keep the highest-scoring hypothesis.
    float best_score = 0.f;
    cv::Point2f best_ex, best_ey, best_origin;
    int best_c0 = 0, best_r0 = 0;
    for (const Step& st : steps) {
        const cv::Point2f s = st.s;
        const float slen = std::sqrt(s.x * s.x + s.y * s.y);
        if (slen < 1e-3f) continue;
        // u = s/d, |u| = √2. v = u rotated ±90°. ex = (u-v)/2, ey = (u+v)/2 →
        // unit grid axes. Trying both signs of v covers both diagonal types and
        // both orientations; the match_weighted slide resolves the placement.
        const cv::Point2f u = s * (1.f / d);
        const cv::Point2f v_opt[2] = {
            cv::Point2f(-u.y, u.x),
            cv::Point2f(u.y, -u.x)
        };
        for (const cv::Point2f& v : v_opt) {
            const cv::Point2f ex = (u - v) * 0.5f;
            const cv::Point2f ey = (u + v) * 0.5f;
            int c0 = 0, r0 = 0;
            float score = 0.f;
            match_weighted(st.from, ex, ey, &c0, &r0, &score);
            if (score > best_score) {
                best_score = score;
                best_ex = ex; best_ey = ey; best_origin = st.from;
                best_c0 = c0; best_r0 = r0;
            }
        }
    }

    // Every slot must have at least one detected feature (cross or ring) — the
    // minimum weight for any real marker is min(kCrossConfidence, kRingConfidence)
    // = 0.5. No slot may be empty: a synthesised position has no detected feature
    // and would corrupt the calibration. There is no second-stage recovery — an
    // empty slot simply fails the fit.
    const float min_score = float(need) * std::min(kCrossConfidence, kRingConfidence);
    if (best_score < min_score) {
        SDBG("[screw] fit: best_score=%.1f < min_score=%.1f REJECT\n",
             best_score, min_score);
        return false;
    }
    SDBG("[screw] fit: best_score=%.1f (min=%.1f max=%.1f)\n",
         best_score, min_score,
         float(need) * (kCrossConfidence + kRingConfidence + kNeighborConfidence));

    // Assign each grid position its matched marker (nearest within tol). Collect
    // matched points, then sort by (y, x) for canonical row-major order. The
    // sort resolves the 8-fold directional ambiguity of (ex, ey) — the match
    // score is the same for all 8 orientations, but only one matches the
    // object-point formula (r increases → +y, c increases → +x). Same-row
    // threshold = d/2 (half the grid spacing) separates adjacent rows.
    ordered->clear();
    ordered->reserve(need);
    SDBG("[screw] grid slots (c,r = predicted pos, matched marker):\n");
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            const cv::Point2f p = best_origin +
                ((2 * (best_c0 + c) + ((best_r0 + r) & 1)) * d) * best_ex +
                ((best_r0 + r) * d) * best_ey;
            int best_k = -1;
            float best_d2 = match_tol * match_tol;
            for (int k = 0; k < M; ++k) {
                const float dx = pts[k].x - p.x;
                const float dy = pts[k].y - p.y;
                const float d2 = dx * dx + dy * dy;
                if (d2 < best_d2) { best_d2 = d2; best_k = k; }
            }
            if (best_k >= 0) {
                ordered->push_back(pts[best_k]);
                SDBG("  (%d,%d) pred=(%.0f,%.0f) match=#%d(%.0f,%.0f) d=%.1f w=%.1f %s%s\n",
                     c, r, p.x, p.y, best_k, pts[best_k].x, pts[best_k].y,
                     std::sqrt(best_d2), markers[best_k].weight(),
                     markers[best_k].has_cross ? "X" : "",
                     markers[best_k].has_ring ? "O" : "");
            } else {
                // No marker within tolerance and no second-stage recovery — the
                // slot is empty. The fit fails (see the size check below).
                // Diagnostic: nearest marker regardless of tol, for debugging.
                int nk = -1; float nd2 = 1e9f;
                for (int k = 0; k < M; ++k) {
                    const float dx = pts[k].x - p.x, dy = pts[k].y - p.y;
                    const float d2 = dx*dx + dy*dy;
                    if (d2 < nd2) { nd2 = d2; nk = k; }
                }
                SDBG("  (%d,%d) pred=(%.0f,%.0f) EMPTY", c, r, p.x, p.y);
                if (nk >= 0) {
                    SDBG(" nearest=#%d(%.0f,%.0f) d=%.1f %s%s\n",
                         nk, pts[nk].x, pts[nk].y, std::sqrt(nd2),
                         markers[nk].has_cross ? "X" : "",
                         markers[nk].has_ring ? "O" : "");
                } else {
                    SDBG("\n");
                }
            }
        }
    }
    if (int(ordered->size()) < need) return false;  // empty slots — reject
    std::sort(ordered->begin(), ordered->end(),
              [&](const cv::Point2f& a, const cv::Point2f& b) {
                  if (std::abs(a.y - b.y) > d * 0.5f) return a.y < b.y;
                  return a.x < b.x;
              });
    return true;
}

// Returns true if any of the 8 pixels surrounding @p pos (in the foreground
// mask @p fg) is non-zero — a centre surrounded by signal is more reliable than
// one sitting in empty space (e.g. inside a hollow ring with no cross). This is
// the 8-neighbourhood confidence boost (kNeighborConfidence = 0.2).
bool has_8neighbor_signal(const cv::Mat& fg, const cv::Point2f& pos) {
    const int cx = static_cast<int>(pos.x);
    const int cy = static_cast<int>(pos.y);
    for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
            if (dx == 0 && dy == 0) continue;
            const int x = cx + dx, y = cy + dy;
            if (x < 0 || x >= fg.cols || y < 0 || y >= fg.rows) continue;
            if (fg.at<uchar>(y, x)) return true;
        }
    }
    return false;
}

} // namespace

DetectionResult detect_screwheads(const cv::Mat& color_frame,
                                  int cols, int rows,
                                  int dot_gap,
                                  bool annotate,
                                  const RingParams& ring) {
    DetectionResult result;
    if (color_frame.empty() || color_frame.channels() != 3) return result;

    cv::Mat fg = foreground_mask(color_frame);
    cv::Mat denoised = denoise_isolated(fg, dot_gap);
    SDBG("[screw] frame %dx%d fg=%d denoised=%d\n",
         color_frame.cols, color_frame.rows, cv::countNonZero(fg), cv::countNonZero(denoised));

    // Stage 2: cross-centre candidates.
    std::vector<Candidate> cands = find_cross_candidates(denoised, dot_gap,
                                                         ring.search_margin);
    SDBG("[screw] stage2 candidates=%zu (need>=%d)\n", cands.size(), cols * rows);
    if (cands.size() < 9) return result;  // need at least ~a third of 30

    // Stage 3: joint cross + ring localisation. Cross and ring are PEER
    // features: each cross candidate always yields a cross centre, and a ring
    // is searched around it (found or not). Detecting EITHER feature means a
    // possible marker centre was found. Each contributes 50% confidence; the
    // 8-neighbourhood of the final centre adds 20%. The combined weight drives
    // the grid-fit RANSAC.
    //
    // NOTE: radius outliers (large merged arcs with r=40-169 when true r≈13)
    // are NOT downgraded here. At 20000µs these are often motion ghost markers
    // — their positions coincide with real grid positions (helping the grid fit
    // anchor), but their radii are inflated by the merged arc. The d estimation
    // in fit_asymmetric_grid filters by radius consistency for spacing, so the
    // grid axes are not corrupted by the outlier radii.
    //
    // Every detected cross/ring feature is reported in the result (cross_centers
    // / ring_centers / ring_radii) even if the grid fit later fails — the
    // capture-review dialog draws them (blue crosses, red circles) so the user
    // can see what WAS found and relax the ring parameters before re-detecting.
    std::vector<Marker> markers;
    int n_both = 0, n_cross_only = 0;
    for (const Candidate& c : cands) {
        Marker m;
        m.cross_pos = c.pos;
        m.has_cross = true;
        result.cross_centers.push_back(c.pos);
        const RingInfo rinfo = find_ring(fg, c.pos, c.max_r, ring);
        if (rinfo.found) {
            m.ring_pos = rinfo.center;
            m.ring_radius = rinfo.radius;
            m.coverage = rinfo.coverage;
            m.has_ring = true;
            result.ring_centers.push_back(rinfo.center);
            result.ring_radii.push_back(rinfo.radius);
        }
        m.has_neighbor_support = has_8neighbor_signal(fg, m.best_pos());
        if (m.has_ring) ++n_both; else ++n_cross_only;
        markers.push_back(m);
    }
    SDBG("[screw] stage3 markers=%zu cross+ring=%d cross-only=%d (need>=%d)\n",
         markers.size(), n_both, n_cross_only, cols * rows);
    if (int(markers.size()) < cols * rows) return result;

    // Stage 4: weighted RANSAC grid fit + order row-major.
    std::vector<cv::Point2f> ordered;
    const bool fit_ok = fit_asymmetric_grid(markers, cols, rows, &ordered);
    SDBG("[screw] stage4 fit=%d\n", fit_ok ? 1 : 0);
    if (!fit_ok) return result;

    result.found = true;
    result.points = std::move(ordered);

    if (annotate) {
        result.image = color_frame.clone();
        cv::drawChessboardCorners(result.image, cv::Size(cols, rows),
                                  result.points, true);
    }
    return result;
}

} // namespace gui_algo
