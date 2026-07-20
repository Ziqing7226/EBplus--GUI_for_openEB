# gui/tests/ — GTest-based unit tests for GUI core logic (design §3.11).
#
# Each test links against the gui_core static library (defined in the parent
# CMakeLists.txt) so it can exercise AlgoBridge, ConfigManager, LayoutManager
# and the display strategies without a live camera or a display. Tests that
# instantiate Qt widgets set QT_QPA_PLATFORM=offscreen (via a custom main() and
# the ENVIRONMENT test property) so they run headless.

find_package(GTest REQUIRED)
include(GoogleTest)

set(GUI_TEST_COMPILE_OPTS -Wall -Wextra)

# --- test_theme_tokens: header-only token contrast check (no gui_core link). ---
add_executable(test_theme_tokens
    test_theme_tokens.cpp
)
target_include_directories(test_theme_tokens PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/..)
target_link_libraries(test_theme_tokens PRIVATE
    Qt6::Core
    GTest::GTest
    GTest::Main
)
target_compile_options(test_theme_tokens PRIVATE ${GUI_TEST_COMPILE_OPTS})
gtest_discover_tests(test_theme_tokens)

# --- test_algo_bridge: registry, idempotent find_or_create, flood guard. ---
add_executable(test_algo_bridge
    test_algo_bridge.cpp
)
target_link_libraries(test_algo_bridge PRIVATE
    gui_core
    GTest::GTest
    GTest::Main
)
target_compile_options(test_algo_bridge PRIVATE ${GUI_TEST_COMPILE_OPTS})
gtest_discover_tests(test_algo_bridge)

# --- test_config_manager: algorithm-state serialization round-trip. ---
add_executable(test_config_manager
    test_config_manager.cpp
)
target_link_libraries(test_config_manager PRIVATE
    gui_core
    GTest::GTest
    GTest::Main
)
target_compile_options(test_config_manager PRIVATE ${GUI_TEST_COMPILE_OPTS})
gtest_discover_tests(test_config_manager)

# --- test_layout_manager: layout save/load symmetry (needs QApplication). ---
# Custom main() creates an offscreen QApplication before InitGoogleTest, so we
# link GTest::GTest (not GTest::Main) to avoid a duplicate main().
add_executable(test_layout_manager
    test_layout_manager.cpp
)
target_link_libraries(test_layout_manager PRIVATE
    gui_core
    GTest::GTest
)
target_compile_options(test_layout_manager PRIVATE ${GUI_TEST_COMPILE_OPTS})
gtest_discover_tests(test_layout_manager
    PROPERTIES ENVIRONMENT "QT_QPA_PLATFORM=offscreen"
)

# --- test_display_strategy: per-strategy apply correctness (needs QApplication). ---
add_executable(test_display_strategy
    test_display_strategy.cpp
)
target_link_libraries(test_display_strategy PRIVATE
    gui_core
    GTest::GTest
)
target_compile_options(test_display_strategy PRIVATE ${GUI_TEST_COMPILE_OPTS})
gtest_discover_tests(test_display_strategy
    PROPERTIES ENVIRONMENT "QT_QPA_PLATFORM=offscreen"
)

# --- test_sharpness_metrics: pure-metric checks (no Qt, links gui_core). ---
add_executable(test_sharpness_metrics
    test_sharpness_metrics.cpp
)
target_link_libraries(test_sharpness_metrics PRIVATE
    gui_core
    GTest::GTest
    GTest::Main
)
target_compile_options(test_sharpness_metrics PRIVATE ${GUI_TEST_COMPILE_OPTS})
gtest_discover_tests(test_sharpness_metrics)
