// gui/algo_bridge/backends/backend_factory.cpp — top-level factory function
// (design §3.4). Dispatches to per-category factory functions declared in
// backend_registry.h. Each category .cpp file implements its own factory;
// this file just calls them in sequence and returns the first non-null result.

#include "algo_bridge/algo_backend.h"
#include "algo_bridge/backends/backend_registry.h"

#include <memory>
#include <string>

namespace gui {

std::unique_ptr<AlgoBackend> create_algo_backend(const std::string& name,
                                                  int width, int height) {
    if (auto b = create_cv_backend(name, width, height))             return b;
    if (auto b = create_cv_vector_backend(name, width, height))      return b;
    if (auto b = create_analytics_backend(name, width, height))      return b;
    if (auto b = create_analytics_extra_backend(name, width, height)) return b;
    if (auto b = create_display_backend(name, width, height))        return b;
    if (auto b = create_filter_backend(name, width, height))         return b;
    // NOTE: the former openeb_* backend categories were removed (audit
    // §3-B6/7) — the remaining OpenEB event-transform stages are handled by
    // FilterChain, not by AlgoBackend instances.
    return nullptr;
}

} // namespace gui
