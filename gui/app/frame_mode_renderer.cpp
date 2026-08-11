// gui/app/frame_mode_renderer.cpp — see header.

#include "frame_mode_renderer.h"

#include <algorithm>

#include <opencv2/imgproc.hpp>

namespace gui {

void FrameModeRenderer::set_geometry(int width, int height) {
    std::lock_guard<std::mutex> lk(mutex_);
    if (width == width_ && height == height_) return;
    width_ = width;
    height_ = height;
    contrast_.reset();
    time_decay_.reset();
    integration_.reset();
    histo_pos_ = cv::Mat_<int>();
    histo_neg_ = cv::Mat_<int>();
    diff_ = cv::Mat_<int>();
}

void FrameModeRenderer::set_mode(FrameMode mode) {
    std::lock_guard<std::mutex> lk(mutex_);
    if (mode_ == mode) return;
    mode_ = mode;
    ensure_instances();
    reset_locked();
}

void FrameModeRenderer::set_decay_time_us(Metavision::timestamp us) {
    std::lock_guard<std::mutex> lk(mutex_);
    decay_us_ = us;
    if (time_decay_) time_decay_->set_exponential_decay_time_us(us);
    if (integration_) {
        integration_ = std::make_unique<Metavision::EventsIntegrationAlgorithm>(
            width_, height_, decay_us_);
    }
}

void FrameModeRenderer::set_palette(Metavision::ColorPalette palette) {
    std::lock_guard<std::mutex> lk(mutex_);
    palette_ = palette;
    if (time_decay_) time_decay_->set_color_palette(palette);
}

void FrameModeRenderer::reset() {
    std::lock_guard<std::mutex> lk(mutex_);
    reset_locked();
}

void FrameModeRenderer::reset_locked() {
    if (contrast_) contrast_->reset();
    if (time_decay_) time_decay_->reset();
    if (integration_) integration_->reset();
    if (!histo_pos_.empty()) histo_pos_.setTo(0);
    if (!histo_neg_.empty()) histo_neg_.setTo(0);
    if (!diff_.empty()) diff_.setTo(0);
}

void FrameModeRenderer::ensure_instances() {
    if (width_ <= 0 || height_ <= 0) return;
    switch (mode_) {
        case FrameMode::ContrastMap:
            if (!contrast_) {
                contrast_ = std::make_unique<Metavision::ContrastMapGenerationAlgorithm>(
                    static_cast<unsigned>(width_), static_cast<unsigned>(height_));
            }
            break;
        case FrameMode::TimeDecay:
            if (!time_decay_) {
                time_decay_ = std::make_unique<Metavision::TimeDecayFrameGenerationAlgorithm>(
                    width_, height_, decay_us_, palette_);
            }
            break;
        case FrameMode::EventsIntegration:
            if (!integration_) {
                integration_ = std::make_unique<Metavision::EventsIntegrationAlgorithm>(
                    static_cast<unsigned>(width_), static_cast<unsigned>(height_), decay_us_);
            }
            break;
        default:
            break;
    }
}

void FrameModeRenderer::add_events(const Metavision::EventCD* begin,
                                   const Metavision::EventCD* end) {
    if (begin == nullptr || end == nullptr || begin >= end) return;
    std::lock_guard<std::mutex> lk(mutex_);
    if (mode_ == FrameMode::Integration || width_ <= 0 || height_ <= 0) return;
    ensure_instances();
    switch (mode_) {
        case FrameMode::ContrastMap:
            contrast_->process_events(begin, end);
            break;
        case FrameMode::TimeDecay:
            time_decay_->process_events(begin, end);
            break;
        case FrameMode::EventsIntegration:
            integration_->process_events(begin, end);
            break;
        case FrameMode::Histo: {
            if (histo_pos_.empty()) {
                histo_pos_ = cv::Mat_<int>::zeros(height_, width_);
                histo_neg_ = cv::Mat_<int>::zeros(height_, width_);
            }
            for (const auto* ev = begin; ev != end; ++ev) {
                const int x = static_cast<int>(ev->x);
                const int y = static_cast<int>(ev->y);
                if (x < 0 || x >= width_ || y < 0 || y >= height_) continue;
                if (ev->p != 0) ++histo_pos_(y, x);
                else            ++histo_neg_(y, x);
            }
            break;
        }
        case FrameMode::Diff: {
            if (diff_.empty()) {
                diff_ = cv::Mat_<int>::zeros(height_, width_);
            }
            for (const auto* ev = begin; ev != end; ++ev) {
                const int x = static_cast<int>(ev->x);
                const int y = static_cast<int>(ev->y);
                if (x < 0 || x >= width_ || y < 0 || y >= height_) continue;
                if (ev->p != 0) ++diff_(y, x);
                else            --diff_(y, x);
            }
            break;
        }
        default:
            break;
    }
}

cv::Mat FrameModeRenderer::generate() {
    std::lock_guard<std::mutex> lk(mutex_);
    if (mode_ == FrameMode::Integration || width_ <= 0 || height_ <= 0) return {};
    switch (mode_) {
        case FrameMode::ContrastMap:   return generate_contrast();
        case FrameMode::Histo:         return generate_histo();
        case FrameMode::Diff:          return generate_diff();
        case FrameMode::TimeDecay:     return generate_time_decay();
        case FrameMode::EventsIntegration: return generate_integration();
        default:                       return {};
    }
}

cv::Mat FrameModeRenderer::generate_contrast() {
    ensure_instances();
    if (!contrast_) return {};
    // Fixed linear tonemap (no per-frame renormalization — a NORM_MINMAX per
    // window made the brightness flicker as the scene's event rate fluctuated).
    // Contrast 64 (1.2^23 ≈ 23 same-polarity events — the vendored
    // ContrastMapGenerationAlgorithm multiplies the per-pixel state by
    // contrast_on = 1.2 per ON event) maps to full white; lower event
    // densities render dimmer but stay stable.
    static constexpr float kContrastScale = 255.0f / 64.0f;
    cv::Mat_<uchar> u8;
    contrast_->generate(u8, kContrastScale, 0.0f);  // resets the internal state
    if (u8.empty()) return {};
    cv::Mat bgr;
    cv::cvtColor(u8, bgr, cv::COLOR_GRAY2BGR);
    return bgr;
}

cv::Mat FrameModeRenderer::generate_histo() {
    cv::Mat out(height_, width_, CV_8UC3, cv::Scalar(0, 0, 0));
    if (histo_pos_.empty()) {
        histo_pos_ = cv::Mat_<int>::zeros(height_, width_);
        histo_neg_ = cv::Mat_<int>::zeros(height_, width_);
    }
    // Log-compressed display: v = 255·log1p(c)/log1p(64). A raw count map is
    // nearly black because per-pixel counts in a window are tiny (1–20); the
    // log scale makes small counts visible while bright pixels saturate
    // gracefully. Fixed mapping → no flicker.
    static const double kLogRef = std::log1p(64.0);
    for (int y = 0; y < height_; ++y) {
        cv::Vec3b* row = out.ptr<cv::Vec3b>(y);
        const int* pos = histo_pos_.ptr<int>(y);
        const int* neg = histo_neg_.ptr<int>(y);
        for (int x = 0; x < width_; ++x) {
            // BGR: G = positive count, R = negative count.
            row[x][0] = 0;
            row[x][1] = static_cast<uchar>(std::lround(255.0 * std::log1p(
                static_cast<double>(std::max(0, pos[x]))) / kLogRef));
            row[x][2] = static_cast<uchar>(std::lround(255.0 * std::log1p(
                static_cast<double>(std::max(0, neg[x]))) / kLogRef));
        }
    }
    histo_pos_.setTo(0);
    histo_neg_.setTo(0);
    return out;
}

cv::Mat FrameModeRenderer::generate_diff() {
    cv::Mat out(height_, width_, CV_8UC3);
    if (diff_.empty()) {
        diff_ = cv::Mat_<int>::zeros(height_, width_);
    }
    // Amplify the small per-pixel signed sums: raw sums are ±1–5 (nearly
    // uniform gray); a fixed ×8 gain makes them clearly visible while large
    // sums saturate. Fixed mapping → no flicker.
    static constexpr double kDiffGain = 8.0;
    for (int y = 0; y < height_; ++y) {
        cv::Vec3b* row = out.ptr<cv::Vec3b>(y);
        const int* d = diff_.ptr<int>(y);
        for (int x = 0; x < width_; ++x) {
            // 128 = no net change; positive sum → brighter, negative → darker.
            const int v = std::clamp(static_cast<int>(std::lround(
                                        static_cast<double>(d[x]) * kDiffGain)),
                                     -127, 127) + 128;
            const uchar g = static_cast<uchar>(v);
            row[x] = cv::Vec3b(g, g, g);
        }
    }
    diff_.setTo(0);
    return out;
}

cv::Mat FrameModeRenderer::generate_time_decay() {
    ensure_instances();
    if (!time_decay_) return {};
    cv::Mat frame;
    time_decay_->generate(frame, true);  // allocates CV_8UC3 (colored palette) or CV_8UC1
    if (frame.empty()) return {};
    if (frame.channels() == 1) {
        cv::Mat bgr;
        cv::cvtColor(frame, bgr, cv::COLOR_GRAY2BGR);
        return bgr;
    }
    return frame;  // already BGR
}

cv::Mat FrameModeRenderer::generate_integration() {
    ensure_instances();
    if (!integration_) return {};
    cv::Mat gray;
    integration_->generate(gray);
    if (gray.empty()) return {};
    cv::Mat bgr;
    cv::cvtColor(gray, bgr, cv::COLOR_GRAY2BGR);
    return bgr;
}

} // namespace gui
