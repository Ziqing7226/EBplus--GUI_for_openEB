# algo/tests/ — GTest-based unit tests for self-developed algorithm modules.

find_package(GTest REQUIRED)

# Test executable for Phase 6 (algo/common/) modules.
add_executable(test_phase6_common
    test_phase6_common.cpp
)

target_link_libraries(test_phase6_common PRIVATE
    gui_algo
    GTest::GTest
    GTest::Main
)

target_compile_options(test_phase6_common PRIVATE -Wall -Wextra -Werror)

include(GoogleTest)
gtest_discover_tests(test_phase6_common)

# Test executable for Phase 7-9 (algo/cv/) modules.
# Depends on algo/cv/ headers (noise_filter, hot_pixel_filter, orientation_filter,
# direction_selective_filter, sparse_optical_flow, blob_detector, object_tracker,
# corner_detector, cluster_interface, cluster_path_point).
add_executable(test_phase7_cv
    test_phase7_cv.cpp
)

target_link_libraries(test_phase7_cv PRIVATE
    gui_algo
    GTest::GTest
    GTest::Main
)

target_compile_options(test_phase7_cv PRIVATE -Wall -Wextra -Werror)

gtest_discover_tests(test_phase7_cv)

# Test executable for Phase 8-10 (algo/cv/ + algo/analytics/) modules.
# Depends on algo/cv/ and algo/analytics/ headers (line_segment_detector,
# hough_*, orientation_cluster, cluster_lif, background_mask_filter,
# perspective_undistort, trigger_synced_filter, bandpass_filter, optical_gyro,
# ultra_slow_motion, xyt_visualizer, overlay, time_surface, active_marker,
# event_to_video, flow_statistics, isi_analyzer, particle_counter,
# auto_bias_controller, freq_detector).
add_executable(test_phase8_10
    test_phase8_10.cpp
)

target_link_libraries(test_phase8_10 PRIVATE
    gui_algo
    GTest::GTest
    GTest::Main
)

target_compile_options(test_phase8_10 PRIVATE -Wall -Wextra -Werror)

gtest_discover_tests(test_phase8_10)

# Standalone diagnostic: feed .raw events to EventToVideo and report stats.
add_executable(test_raw_e2v
    test_raw_e2v.cpp
)
target_link_libraries(test_raw_e2v PRIVATE
    gui_algo
    MetavisionSDK::base
    MetavisionSDK::core
    MetavisionSDK::stream
    ${OpenCV_LIBS}
)

# Standalone diagnostic: open a .raw file with real_time_playback(true/false)
# and report event delivery + duration. Used to verify SDK behavior matches
# the GUI's default mode.
add_executable(test_raw_playback
    test_raw_playback.cpp
)
target_link_libraries(test_raw_playback PRIVATE
    MetavisionSDK::base
    MetavisionSDK::core
    MetavisionSDK::stream
)

# Standalone diagnostic: verify FileFrameGenerator produces frames at the
# correct playback rate (fps * window / 1e6). Tests slow-motion, real-time,
# fast-forward, EOF, and loop behavior.
add_executable(test_file_frame_generator
    test_file_frame_generator.cpp
    ${CMAKE_SOURCE_DIR}/gui/app/file_frame_generator.cpp
)
target_include_directories(test_file_frame_generator PRIVATE
    ${CMAKE_SOURCE_DIR}/gui
)
target_link_libraries(test_file_frame_generator PRIVATE
    Qt6::Core
    Qt6::Gui
    MetavisionSDK::base
    MetavisionSDK::core
    MetavisionSDK::stream
    ${OpenCV_LIBS}
)

