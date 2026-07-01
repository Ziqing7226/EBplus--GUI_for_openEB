// algo/common/freme.h — Freme<O>: generic 2D event-driven state-map container.
//
// ✅ 移植自 jAER Freme<O> (net/sf/jaer/eventprocessing/freme/Freme.java).
// A "freme" is the event-driven equivalent of a frame: a 2D set of values (or
// objects) that get updated on the arrival of an event. Each pixel (x, y) maps
// to a single state object of type O. This is a header-only template so the
// state type can be chosen by the consumer (e.g. float for orientation maps,
// or a small histogram struct for frequency representations).

#ifndef GUI_ALGO_COMMON_FREME_H
#define GUI_ALGO_COMMON_FREME_H

#include <algorithm>
#include <cstddef>
#include <vector>

namespace gui_algo {

/// @brief Generic 2D state-map container — the event-driven equivalent of a
/// frame. Maps each pixel (x, y) to a state object of type @p O.
///
/// Ported from jAER's `net.sf.jaer.eventprocessing.freme.Freme<O>`.
///
/// @tparam O State type stored per pixel. Must be default-constructible and
///         copy-assignable.
template <typename O>
class Freme {
public:
    /// @brief Constructs a freme of size @p width x @p height.
    /// @param width,height Sensor dimensions (non-negative).
    Freme(int width, int height)
        : width_(width), height_(height),
          data_(static_cast<std::size_t>(width) * static_cast<std::size_t>(height)) {}

    /// @brief Returns a reference to the state at pixel (x, y).
    O& get(int x, int y) {
        return data_[static_cast<std::size_t>(getIndex(x, y))];
    }

    /// @brief Returns a const reference to the state at pixel (x, y).
    const O& get(int x, int y) const {
        return data_[static_cast<std::size_t>(getIndex(x, y))];
    }

    /// @brief Replaces the state at pixel (x, y) with @p value.
    void set(int x, int y, const O& value) {
        data_[static_cast<std::size_t>(getIndex(x, y))] = value;
    }

    /// @brief Sets every pixel's state to @p value.
    void fill(const O& value) {
        std::fill(data_.begin(), data_.end(), value);
    }

    /// @brief Returns the flattened index of pixel (x, y): y * width + x.
    int getIndex(int x, int y) const {
        return y * width_ + x;
    }

    /// @brief Returns the total number of pixels.
    int size() const { return static_cast<int>(data_.size()); }

    /// @brief Returns the width (number of columns).
    int width() const { return width_; }

    /// @brief Returns the height (number of rows).
    int height() const { return height_; }

    /// @brief Pointer to the first element (enables range-based for).
    O* begin() { return data_.data(); }
    /// @brief Pointer one past the last element.
    O* end() { return data_.data() + data_.size(); }
    /// @brief Const pointer to the first element.
    const O* begin() const { return data_.data(); }
    /// @brief Const pointer one past the last element.
    const O* end() const { return data_.data() + data_.size(); }

private:
    int width_;
    int height_;
    std::vector<O> data_;
};

} // namespace gui_algo

#endif // GUI_ALGO_COMMON_FREME_H
