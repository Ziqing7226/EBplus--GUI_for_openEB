# algo/tests/ — GTest-based unit tests for self-developed algorithm modules.

find_package(GTest REQUIRED)

set(ALGO_TEST_COMPILE_OPTIONS -Wall -Wextra -Werror)
if(APPLE)
    # AppleClang diagnoses these existing header-only implementation details
    # more aggressively than the Linux toolchains used for the project.
    list(APPEND ALGO_TEST_COMPILE_OPTIONS
        -Wno-error=unused-private-field
        -Wno-error=unused-lambda-capture
        -Wno-error=unused-but-set-variable
    )
endif()

# Test executable for Phase 6 (algo/common/) modules.
add_executable(test_phase6_common
    test_phase6_common.cpp
)

target_link_libraries(test_phase6_common PRIVATE
    gui_algo
    GTest::GTest
    GTest::Main
)

target_compile_options(test_phase6_common PRIVATE ${ALGO_TEST_COMPILE_OPTIONS})

include(GoogleTest)
ebplus_gtest_discover_tests(test_phase6_common)

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

target_compile_options(test_phase7_cv PRIVATE ${ALGO_TEST_COMPILE_OPTIONS})

ebplus_gtest_discover_tests(test_phase7_cv)

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

target_compile_options(test_phase8_10 PRIVATE ${ALGO_TEST_COMPILE_OPTIONS})

ebplus_gtest_discover_tests(test_phase8_10)

# Raw-event integration tests: exercises every streaming algorithm against a
# real Metavision .raw recording (algo/tests/sparklers.raw, committed). Unlike
# the synthetic-event unit tests above, these assert real-world behaviour:
# non-flat reconstructed frames, no NaN divergence, sane filter rates, and
# in-bounds detected features. The .raw path is injected as an absolute
# compile definition so the test binary finds it regardless of CWD.
add_executable(test_raw_algos
    test_raw_algos.cpp
)

target_link_libraries(test_raw_algos PRIVATE
    gui_algo
    GTest::GTest
    GTest::Main
    MetavisionSDK::base
    MetavisionSDK::core
    MetavisionSDK::stream
    ${OpenCV_LIBS}
)

target_include_directories(test_raw_algos PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_compile_definitions(test_raw_algos PRIVATE
    GUI_TEST_RAW_FILE="${CMAKE_CURRENT_SOURCE_DIR}/sparklers.raw"
)

target_compile_options(test_raw_algos PRIVATE -Wall -Wextra)

ebplus_gtest_discover_tests(test_raw_algos)

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

# Standalone diagnostic: simulate GUI file-playback path for EventToVideo.
# Feeds events in small batches with scaled timestamps and reports frame
# statistics (min/max/mean) to diagnose black/flicker output during playback.
add_executable(test_playback_e2v
    test_playback_e2v.cpp
)
target_link_libraries(test_playback_e2v PRIVATE
    gui_algo
    MetavisionSDK::base
    MetavisionSDK::core
    MetavisionSDK::stream
    ${OpenCV_LIBS}
)

# Standalone diagnostic: verify FileFrameGenerator produces frames at the
# correct playback rate (fps * window / 1e6). Tests slow-motion, real-time,
# fast-forward, EOF, and loop behavior.
add_executable(test_file_frame_generator
    test_file_frame_generator.cpp
    ${CMAKE_SOURCE_DIR}/gui/app/file_frame_generator.cpp
    ${CMAKE_SOURCE_DIR}/gui/algo_bridge/filter_chain.cpp
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

# Diagnostic: verify FilterChain (flip_x) is applied in
# FileFrameGenerator::render_frame() so display reflects flip toggles
# immediately during file playback.
add_executable(test_filter_in_render
    test_filter_in_render.cpp
    ${CMAKE_SOURCE_DIR}/gui/app/file_frame_generator.cpp
    ${CMAKE_SOURCE_DIR}/gui/algo_bridge/filter_chain.cpp
)
target_include_directories(test_filter_in_render PRIVATE
    ${CMAKE_SOURCE_DIR}/gui
)
target_link_libraries(test_filter_in_render PRIVATE
    Qt6::Core
    Qt6::Gui
    MetavisionSDK::base
    MetavisionSDK::core
    ${OpenCV_LIBS}
)

# Integration test: full file-playback flow with FilterChain using a real
# .raw file. Verifies flip_x toggled DURING playback takes effect.
add_executable(test_filter_integration
    test_filter_integration.cpp
    ${CMAKE_SOURCE_DIR}/gui/app/file_frame_generator.cpp
    ${CMAKE_SOURCE_DIR}/gui/algo_bridge/filter_chain.cpp
)
target_include_directories(test_filter_integration PRIVATE
    ${CMAKE_SOURCE_DIR}/gui
)
target_link_libraries(test_filter_integration PRIVATE
    Qt6::Core
    Qt6::Gui
    MetavisionSDK::base
    MetavisionSDK::core
    MetavisionSDK::stream
    ${OpenCV_LIBS}
)

# Loop + flip test: verifies flip_x works during LOOP playback and survives
# across loop wraps. Simulates the exact GUI scenario.
add_executable(test_loop_flip
    test_loop_flip.cpp
    ${CMAKE_SOURCE_DIR}/gui/app/file_frame_generator.cpp
    ${CMAKE_SOURCE_DIR}/gui/algo_bridge/filter_chain.cpp
)
target_include_directories(test_loop_flip PRIVATE
    ${CMAKE_SOURCE_DIR}/gui
)
target_link_libraries(test_loop_flip PRIVATE
    Qt6::Core
    Qt6::Gui
    MetavisionSDK::base
    MetavisionSDK::core
    MetavisionSDK::stream
    ${OpenCV_LIBS}
)
