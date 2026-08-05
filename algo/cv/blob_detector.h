// algo/cv/blob_detector.h — connected-component blob detection with background model.
//
// Self-developed (design §4.3.10). Accumulates events into a count frame over
// an accumulation window, separates foreground from a learned background model
// (Histogram2DFilter-style EMA, approximating the rolling HxW event-count
// histogram), thresholds and runs cv::connectedComponents, then filters blobs
// by minimum area. Output is a vector of bounding boxes for overlay.
// Header-only.

#ifndef GUI_ALGO_CV_BLOB_DETECTOR_H
#define GUI_ALGO_CV_BLOB_DETECTOR_H

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <metavision/sdk/base/utils/timestamp.h>

#include "algo/common/event.h"
#include "algo/common/event_packet.h"

namespace gui_algo {

/// @brief A detected blob: bounding box, pixel area and component label.
struct Blob {
    cv::Rect bbox;
    float area{0.0F};
    int label{0};
};

/// @brief Event-accumulation blob detector with EMA background modelling.
class BlobDetector {
public:
    BlobDetector(int width, int height)
        : width_(width), height_(height) {
        reset();
    }

    // Parameters (defaults per design §4.3.10) ----------------------------
    void set_accumulation_ms(double v) { accumulation_ms_ = clamp_d(v, 1.0, 1000.0); }
    void set_threshold(int v) { threshold_ = clamp_i(v, 1, 254); }
    void set_min_area(int v) { min_area_ = clamp_i(v, 1, 100000); }
    void set_learning_rate(double v) { learning_rate_ = clamp_d(v, 1e-6, 1.0); }

    double accumulation_ms() const { return accumulation_ms_; }
    int threshold() const { return threshold_; }
    int min_area() const { return min_area_; }
    double learning_rate() const { return learning_rate_; }
    int width() const { return width_; }
    int height() const { return height_; }

    /// @brief Accumulates events; emits new blobs when the accumulation window
    ///        elapses (driven by event timestamps).
    void process(const Event* events, std::size_t count) {
        ensure_mats();
        for (std::size_t i = 0; i < count; ++i) {
            const Event& e = events[i];
            if (e.x < width_ && e.y < height_) {
                float& v = accum_.at<float>(static_cast<int>(e.y), static_cast<int>(e.x));
                if (v < 1e6F) v += 1.0F;
                if (e.t > last_event_t_) last_event_t_ = e.t;
            }
        }
        if (last_event_t_ - last_frame_t_ >= accum_us()) {
            generate();
            last_frame_t_ = last_event_t_;
        }
    }

    /// @brief Processes an event packet (accumulation + windowed emission).
    void process(EventPacket& events) {
        process(events.data(), events.size());
    }

    /// @brief Returns the most recently emitted blobs.
    const std::vector<Blob>& blobs() const { return blobs_; }

    /// @brief Returns the latest foreground mask (CV_8U), empty before first emit.
    const cv::Mat& foreground_mask() const { return fg_mask_; }

    void reset() {
        accum_ = cv::Mat();
        background_ = cv::Mat();
        fg_mask_ = cv::Mat();
        blobs_.clear();
        last_event_t_ = 0;
        last_frame_t_ = 0;
    }

private:
    static int clamp_i(int v, int lo, int hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }
    static double clamp_d(double v, double lo, double hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }

    Metavision::timestamp accum_us() const {
        return static_cast<Metavision::timestamp>(accumulation_ms_ * 1000.0);
    }

    void ensure_mats() {
        if (accum_.empty()) {
            accum_ = cv::Mat::zeros(height_, width_, CV_32FC1);
            background_ = cv::Mat::zeros(height_, width_, CV_32FC1);
        }
    }

    void generate() {
        // Foreground = current accumulation above the learned background.
        cv::Mat fg;
        cv::subtract(accum_, background_, fg);
        cv::threshold(fg, fg_mask_, static_cast<double>(threshold_), 255.0,
                      cv::THRESH_BINARY);
        if (fg_mask_.type() != CV_8U) {
            fg_mask_.convertTo(fg_mask_, CV_8U);
        }

        cv::Mat labels;
        const int n = cv::connectedComponents(fg_mask_, labels, 8, CV_32S);
        blobs_.clear();
        if (n > 1) {
            std::vector<int> area(n, 0);
            std::vector<int> minx(n, width_), miny(n, height_);
            std::vector<int> maxx(n, -1), maxy(n, -1);
            for (int y = 0; y < height_; ++y) {
                const std::int32_t* row = labels.ptr<std::int32_t>(y);
                for (int x = 0; x < width_; ++x) {
                    const int lbl = row[x];
                    if (lbl == 0) continue;
                    ++area[lbl];
                    if (x < minx[lbl]) minx[lbl] = x;
                    if (x > maxx[lbl]) maxx[lbl] = x;
                    if (y < miny[lbl]) miny[lbl] = y;
                    if (y > maxy[lbl]) maxy[lbl] = y;
                }
            }
            for (int lbl = 1; lbl < n; ++lbl) {
                if (area[lbl] >= min_area_) {
                    Blob b;
                    b.label = lbl;
                    b.area = static_cast<float>(area[lbl]);
                    b.bbox = cv::Rect(minx[lbl], miny[lbl],
                                      maxx[lbl] - minx[lbl] + 1,
                                      maxy[lbl] - miny[lbl] + 1);
                    blobs_.push_back(b);
                }
            }
        }

        // Update background EMA: bg = (1-lr)*bg + lr*accum.
        cv::Mat tmp;
        cv::addWeighted(background_, 1.0 - learning_rate_, accum_, learning_rate_,
                        0.0, tmp);
        background_ = tmp;
        accum_.setTo(0.0);
    }

    int width_;
    int height_;
    double accumulation_ms_{33.3};
    int threshold_{50};
    int min_area_{10};
    double learning_rate_{0.05};

    cv::Mat accum_;        // CV_32FC1 event-count accumulator
    cv::Mat background_;   // CV_32FC1 EMA background model
    cv::Mat fg_mask_;      // CV_8U foreground mask from last emit
    std::vector<Blob> blobs_;
    Metavision::timestamp last_event_t_{0};
    Metavision::timestamp last_frame_t_{0};
};

} // namespace gui_algo

#endif // GUI_ALGO_CV_BLOB_DETECTOR_H
