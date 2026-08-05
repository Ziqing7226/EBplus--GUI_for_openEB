// gui/algo_bridge/backends/backend_registry.h — declarations for per-category
// backend factory functions. Each category .cpp file implements one of these;
// the top-level create_algo_backend() in backend_factory.cpp dispatches to them
// in sequence, returning the first non-null result. This keeps backend class
// definitions private to their .cpp file (no headers exposing implementation).

#ifndef GUI_ALGO_BRIDGE_BACKENDS_BACKEND_REGISTRY_H
#define GUI_ALGO_BRIDGE_BACKENDS_BACKEND_REGISTRY_H

#include <memory>
#include <string>

namespace gui {

class AlgoBackend;

/// Tries to create a backend from the CV in-place-filter + overlay-detector
/// category (hot_pixel_filter, optical_gyro, object_tracker, corner_detector,
/// blob_detector, sparse_optical_flow).
/// Returns nullptr if @p name is not in this category.
std::unique_ptr<AlgoBackend> create_cv_backend(const std::string& name,
                                                int width, int height);

/// CV result-vector detectors (hough_line, hough_circle, line_segment,
/// orientation_cluster, cluster_lif).
std::unique_ptr<AlgoBackend> create_cv_vector_backend(const std::string& name,
                                                       int width, int height);

/// Analytics: event_to_video, flow_statistics, isi_analyzer,
/// freq_detector, active_marker.
std::unique_ptr<AlgoBackend> create_analytics_backend(const std::string& name,
                                                       int width, int height);

/// Analytics extras: particle_counter, auto_bias, trigger_synced.
std::unique_ptr<AlgoBackend> create_analytics_extra_backend(const std::string& name,
                                                             int width, int height);

/// Display: time_surface, ultra_slow_motion, xyt_visualizer, overlay.
std::unique_ptr<AlgoBackend> create_display_backend(const std::string& name,
                                                     int width, int height);

/// Filters: orientation_filter, direction_selective, background_mask,
/// bandpass_filter.
std::unique_ptr<AlgoBackend> create_filter_backend(const std::string& name,
                                                    int width, int height);

} // namespace gui

#endif // GUI_ALGO_BRIDGE_BACKENDS_BACKEND_REGISTRY_H
