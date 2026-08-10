// algo/calibration/screwhead_detect.cpp — see header.

#include "screwhead_detect.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace gui_algo {

namespace {

// Three-valued frame colours (BGR). Must match CalibrationWizard::render_event_frame.
const cv::Vec3b kGold(0, 215, 255);    // ON polarity
const cv::Vec3b kWhite(255, 255, 255); // OFF polarity
// kBlend = (kGold + kWhite) / 2, computed in render_event_frame.

// Minimum fraction of ring pixels that must fit the half/half structure for a
// candidate to be accepted as a real marker. Tunable (field, C4).
constexpr float kHalfHalfFrac = 0.70f;
// Minimum ring pixels for a meaningful polarity test.
constexpr int kMinRingPixels = 8;
// Match tolerance (fraction of grid spacing d) for grid fitting.
constexpr float kMatchTolFrac = 0.35f;

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

// Polarity from colour: 1 = ON (gold, R-B large), 0 = OFF (white, R-B ~0).
// Blend (R-B = 128) is the midpoint; ties round to ON. The half/half test
// tolerates the few blend pixels, so the exact tie-break does not matter.
inline int polarity_of(const cv::Vec3b& v) {
    const int rb = int(v[2]) - int(v[0]);  // R - B (BGR ordering)
    return rb >= 128 ? 1 : 0;
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

std::vector<Candidate> find_cross_candidates(const cv::Mat& fg_denoised, int dot_gap) {
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
        // Limit the ring-search radius to the component bounding box so the
        // radial histogram does not pick up NEIGHBOURING markers' rings (which
        // would give a wrong best_r and fail the half/half test).
        const int max_r = std::max(lw, lh) / 2 + 2;
        cands.push_back({cv::Point2f(float(best.x), float(best.y)), max_r});
    }
    return cands;
}

// Stage 3: verify a candidate by finding its ring radius and checking the
// half/half polarity structure. Returns true and fills out_radius on success.
// @param max_r limits the radial histogram so only THIS marker's ring is counted.
bool verify_ring(const cv::Mat& fg, const cv::Mat& color,
                 const cv::Point2f& cand, int max_r, float* out_radius) {
    if (max_r < 4) return false;
    const int x0 = std::max(0, int(cand.x) - max_r);
    const int x1 = std::min(fg.cols, int(cand.x) + max_r + 1);
    const int y0 = std::max(0, int(cand.y) - max_r);
    const int y1 = std::min(fg.rows, int(cand.y) + max_r + 1);

    // Radial histogram of foreground pixels around the candidate.
    std::vector<int> hist(max_r + 2, 0);
    for (int y = y0; y < y1; ++y) {
        const uchar* f = fg.ptr<uchar>(y);
        for (int x = x0; x < x1; ++x) {
            if (!f[x]) continue;
            const float dx = x - cand.x, dy = y - cand.y;
            const int r = int(std::sqrt(dx * dx + dy * dy) + 0.5f);
            if (r >= 1 && r <= max_r) hist[r]++;
        }
    }
    // Smoothed peak (3-bin sum) — the ring shows as a clear maximum.
    int best_r = 0, best_count = 0;
    for (int r = 3; r <= max_r - 1; ++r) {
        const int c = hist[r - 1] + hist[r] + hist[r + 1];
        if (c > best_count) { best_count = c; best_r = r; }
    }
    if (best_r < 3) return false;

    // Collect ring pixels (distance within tol of best_r) with angle + polarity.
    const float tol = 2.f;
    std::vector<std::pair<float, int>> ring;  // (angle, polarity)
    for (int y = y0; y < y1; ++y) {
        const uchar* f = fg.ptr<uchar>(y);
        const cv::Vec3b* c = color.ptr<cv::Vec3b>(y);
        for (int x = x0; x < x1; ++x) {
            if (!f[x]) continue;
            const float dx = x - cand.x, dy = y - cand.y;
            const float dist = std::sqrt(dx * dx + dy * dy);
            if (std::abs(dist - best_r) > tol) continue;
            ring.emplace_back(std::atan2(dy, dx), polarity_of(c[x]));
        }
    }
    if (int(ring.size()) < kMinRingPixels) return false;

    // Half/half test: find the circular split that best separates the two
    // polarities into opposite semicircles. score = max over splits of
    //   (ON in first half + OFF in second) or (OFF in first + ON in second).
    // A real ring edge under translation gives ~2 transitions and a score near
    // N; random noise gives ~N/2. Require score/N >= kHalfHalfFrac.
    std::sort(ring.begin(), ring.end(),
              [](const std::pair<float, int>& a, const std::pair<float, int>& b) {
                  return a.first < b.first;
              });
    const int N = int(ring.size());
    std::vector<int> pol(2 * N);
    for (int i = 0; i < 2 * N; ++i) pol[i] = ring[i % N].second;
    const int half = N / 2;
    int best_score = 0;
    for (int i = 0; i < N; ++i) {
        int on1 = 0, off1 = 0;
        for (int j = i; j < i + half; ++j) (pol[j] ? ++on1 : ++off1);
        int on2 = 0, off2 = 0;
        for (int j = i + half; j < i + N; ++j) (pol[j] ? ++on2 : ++off2);
        best_score = std::max(best_score, std::max(on1 + off2, off1 + on2));
    }
    if (float(best_score) / float(N) < kHalfHalfFrac) return false;

    *out_radius = float(best_r);
    return true;
}

// Median of a vector (caller ensures non-empty).
float median(std::vector<float> v) {
    std::sort(v.begin(), v.end());
    return v[v.size() / 2];
}

// Stage 4: fit the asymmetric cols×rows grid to verified centres and order them
// row-major. Returns true only if ALL cols×rows positions matched.
bool fit_asymmetric_grid(const std::vector<cv::Point2f>& centers,
                         int cols, int rows,
                         std::vector<cv::Point2f>* ordered) {
    const int M = int(centers.size());
    const int need = cols * rows;
    if (M < need) return false;  // cannot fill the grid

    // Nearest-neighbour distance per centre → median is the diagonal step d·√2.
    std::vector<float> nn(M, 1e9f);
    for (int i = 0; i < M; ++i) {
        for (int j = 0; j < M; ++j) {
            if (i == j) continue;
            const float dx = centers[i].x - centers[j].x;
            const float dy = centers[i].y - centers[j].y;
            const float d = std::sqrt(dx * dx + dy * dy);
            if (d < nn[i]) nn[i] = d;
        }
    }
    const float D_nn = median(nn);
    const float d = D_nn / std::sqrt(2.0f);  // grid half-cell spacing
    if (d < 3.f) return false;
    const float match_tol = kMatchTolFrac * d;

    // Collect candidate diagonal steps (pairs at ~D_nn).
    struct Step { cv::Point2f from; cv::Point2f s; };
    std::vector<Step> steps;
    for (int i = 0; i < M; ++i) {
        for (int j = i + 1; j < M; ++j) {
            const float dx = centers[j].x - centers[i].x;
            const float dy = centers[j].y - centers[i].y;
            const float dist = std::sqrt(dx * dx + dy * dy);
            if (dist >= 0.7f * D_nn && dist <= 1.3f * D_nn) {
                steps.push_back({centers[i], cv::Point2f(dx, dy)});
            }
        }
    }
    if (steps.empty()) return false;

    auto match_count = [&](const cv::Point2f& origin,
                           const cv::Point2f& ex, const cv::Point2f& ey,
                           int* c0_out, int* r0_out) -> int {
        // Generate a wide grid around origin (c,r in [-cols, 2*cols] etc.) and
        // mark which positions have a matching centre. Then slide a cols×rows
        // window to find the densest block.
        const int gc0 = -cols, gc1 = 2 * cols;
        const int gr0 = -rows, gr1 = 2 * rows;
        const int gw = gc1 - gc0 + 1;
        const int gh = gr1 - gr0 + 1;
        std::vector<char> hit(gw * gh, 0);
        for (int r = gr0; r <= gr1; ++r) {
            for (int c = gc0; c <= gc1; ++c) {
                const cv::Point2f p = origin +
                    ((2 * c + (r & 1)) * d) * ex + (r * d) * ey;
                for (int k = 0; k < M; ++k) {
                    const float dx = centers[k].x - p.x;
                    const float dy = centers[k].y - p.y;
                    if (dx * dx + dy * dy <= match_tol * match_tol) {
                        hit[(r - gr0) * gw + (c - gc0)] = 1;
                        break;
                    }
                }
            }
        }
        int best = 0, bc0 = gc0, br0 = gr0;
        for (int r = gr0; r + rows - 1 <= gr1; ++r) {
            for (int c = gc0; c + cols - 1 <= gc1; ++c) {
                int cnt = 0;
                for (int rr = 0; rr < rows; ++rr)
                    for (int cc = 0; cc < cols; ++cc)
                        cnt += hit[(r - gr0 + rr) * gw + (c - gc0 + cc)];
                if (cnt > best) { best = cnt; bc0 = c; br0 = r; }
            }
        }
        if (c0_out) *c0_out = bc0;
        if (r0_out) *r0_out = br0;
        return best;
    };

    int best_matches = 0;
    cv::Point2f best_ex, best_ey, best_origin;
    int best_c0 = 0, best_r0 = 0;

    for (const Step& st : steps) {
        const cv::Point2f s = st.s;
        const float slen = std::sqrt(s.x * s.x + s.y * s.y);
        if (slen < 1e-3f) continue;
        // u = s/d, |u| = √2. v = u rotated ±90°. ex = (u-v)/2, ey = (u+v)/2 →
        // unit grid axes. Trying both signs of v covers both diagonal types and
        // both orientations; the match_count slide resolves the placement.
        const cv::Point2f u = s * (1.f / d);
        const cv::Point2f v_opt[2] = {
            cv::Point2f(-u.y, u.x),
            cv::Point2f(u.y, -u.x)
        };
        for (const cv::Point2f& v : v_opt) {
            const cv::Point2f ex = (u - v) * 0.5f;
            const cv::Point2f ey = (u + v) * 0.5f;
            int c0 = 0, r0 = 0;
            const int cnt = match_count(st.from, ex, ey, &c0, &r0);
            if (cnt > best_matches) {
                best_matches = cnt;
                best_ex = ex; best_ey = ey; best_origin = st.from;
                best_c0 = c0; best_r0 = r0;
            }
        }
    }

    if (best_matches < need) return false;  // require a fully-filled grid

    // Assign each grid position its matched centre (nearest within tol), then
    // emit row-major.
    ordered->assign(need, cv::Point2f(-1.f, -1.f));
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            const cv::Point2f p = best_origin +
                ((2 * (best_c0 + c) + ((best_r0 + r) & 1)) * d) * best_ex +
                ((best_r0 + r) * d) * best_ey;
            int best_k = -1;
            float best_d2 = match_tol * match_tol;
            for (int k = 0; k < M; ++k) {
                const float dx = centers[k].x - p.x;
                const float dy = centers[k].y - p.y;
                const float d2 = dx * dx + dy * dy;
                if (d2 < best_d2) { best_d2 = d2; best_k = k; }
            }
            if (best_k >= 0) (*ordered)[r * cols + c] = centers[best_k];
        }
    }
    // All slots must be filled.
    for (const auto& p : *ordered) {
        if (p.x < 0) return false;
    }
    return true;
}

} // namespace

DetectionResult detect_screwheads(const cv::Mat& color_frame,
                                  int cols, int rows,
                                  int dot_gap,
                                  bool annotate) {
    DetectionResult result;
    if (color_frame.empty() || color_frame.channels() != 3) return result;

    cv::Mat fg = foreground_mask(color_frame);
    cv::Mat denoised = denoise_isolated(fg, dot_gap);

    // Stage 2: cross-centre candidates.
    std::vector<Candidate> cands = find_cross_candidates(denoised, dot_gap);
    if (cands.size() < 9) return result;  // need at least ~a third of 30

    // Stage 3: verify each candidate via ring + polarity.
    std::vector<cv::Point2f> verified;
    for (const Candidate& c : cands) {
        float r = 0.f;
        if (verify_ring(fg, color_frame, c.pos, c.max_r, &r)) {
            verified.push_back(c.pos);
        }
    }
    if (int(verified.size()) < cols * rows) return result;

    // Stage 4: fit the asymmetric grid + order row-major.
    std::vector<cv::Point2f> ordered;
    if (!fit_asymmetric_grid(verified, cols, rows, &ordered)) return result;

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
