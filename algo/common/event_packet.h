// algo/common/event_packet.h — span-style zero-copy event packet.
//
// Wraps a contiguous range of events (Metavision::EventCD or gui_algo::Event)
// as a non-owning view, analogous to std::span. Algorithms receive packets
// rather than raw pointer/size pairs for safer, bounds-checked iteration.
// The view is immutable by default; mutable variants are provided for in-place
// filters.

#ifndef GUI_ALGO_COMMON_EVENT_PACKET_H
#define GUI_ALGO_COMMON_EVENT_PACKET_H

#include <algorithm>
#include <cstddef>
#include <iterator>
#include <type_traits>
#include <vector>

#include "event.h"

#include <metavision/sdk/base/events/event_cd.h>

namespace gui_algo {

/// @brief Non-owning view over a contiguous event range.
///
/// @tparam EventT Event type (must be layout-compatible with EventCD).
template <typename EventT>
class EventPacketBase {
public:
    using value_type = EventT;
    using pointer = EventT*;
    using const_pointer = const EventT*;
    using reference = EventT&;
    using const_reference = const EventT&;
    using iterator = pointer;
    using const_iterator = const_pointer;

    EventPacketBase() = default;

    EventPacketBase(pointer begin, std::size_t n) : begin_(begin), size_(n) {}

    EventPacketBase(pointer begin, pointer end)
        : begin_(begin), size_(static_cast<std::size_t>(end - begin)) {}

    // Iterators -------------------------------------------------------------
    iterator begin() { return begin_; }
    iterator end() { return begin_ + size_; }
    const_iterator begin() const { return begin_; }
    const_iterator end() const { return begin_ + size_; }
    const_iterator cbegin() const { return begin_; }
    const_iterator cend() const { return begin_ + size_; }

    // Element access --------------------------------------------------------
    reference operator[](std::size_t i) { return begin_[i]; }
    const_reference operator[](std::size_t i) const { return begin_[i]; }
    pointer data() { return begin_; }
    const_pointer data() const { return begin_; }

    // Capacity --------------------------------------------------------------
    std::size_t size() const { return size_; }
    bool empty() const { return size_ == 0; }

    /// @brief Returns a sub-packet [start, start+count).
    /// @note @p start is clamped to size_ to avoid unsigned underflow when it
    ///       exceeds the packet length (returns an empty packet in that case).
    EventPacketBase subpacket(std::size_t start, std::size_t count) const {
        if (start >= size_) return EventPacketBase();
        const std::size_t avail = size_ - start;
        return EventPacketBase(begin_ + start, count < avail ? count : avail);
    }

    /// @brief First @p n events.
    EventPacketBase first(std::size_t n) const {
        return subpacket(0, n);
    }

    /// @brief Last @p n events.
    EventPacketBase last(std::size_t n) const {
        return subpacket(n < size_ ? size_ - n : 0, n);
    }

private:
    pointer begin_{nullptr};
    std::size_t size_{0};
};

/// @brief Immutable event packet (most common usage).
using EventPacket = EventPacketBase<const Event>;

/// @brief Mutable event packet (for in-place filters that rewrite events).
using MutableEventPacket = EventPacketBase<Event>;

/// @brief Immutable packet over raw SDK events (zero-copy from HAL/decoders).
using EventCDPacket = EventPacketBase<const Metavision::EventCD>;

/// @brief Mutable packet over raw SDK events.
using MutableEventCDPacket = EventPacketBase<Metavision::EventCD>;

/// @brief In-place filtering wrapper around an immutable EventPacket.
///
/// Mirrors jAER's EventPacket.filteredOut semantics without enlarging the
/// layout-compatible Event POD (which must stay sizeof(EventCD)). A filter
/// marks events as filtered-out via mark_filtered(); the filtering iterator
/// pair begin_filtered()/end_filtered() skips them, and
/// size_not_filtered_out() reports the surviving count.
class FilteredEventPacket {
public:
    FilteredEventPacket() = default;
    explicit FilteredEventPacket(const EventPacket& packet)
        : packet_(packet), filtered_(packet.size(), false) {}

    std::size_t size() const { return packet_.size(); }
    bool empty() const { return packet_.empty(); }
    const Event& operator[](std::size_t i) const { return packet_[i]; }
    const EventPacket& packet() const { return packet_; }

    /// @brief Marks event @p i as filtered out (no-op if out of range).
    void mark_filtered(std::size_t i) {
        if (i < filtered_.size()) filtered_[i] = true;
    }
    /// @brief Returns true if event @p i has been marked filtered out.
    bool is_filtered(std::size_t i) const {
        return i < filtered_.size() && filtered_[i];
    }
    /// @brief Clears the filtered-out flags (keeps the underlying packet).
    void clear_filtered() { std::fill(filtered_.begin(), filtered_.end(), false); }

    /// @brief Number of events marked filtered out.
    std::size_t filtered_out_count() const {
        std::size_t c = 0;
        for (std::size_t i = 0; i < filtered_.size(); ++i) {
            if (filtered_[i]) ++c;
        }
        return c;
    }
    /// @brief size() - filtered_out_count(), matching jAER getSizeNotFilteredOut.
    std::size_t size_not_filtered_out() const {
        return packet_.size() - filtered_out_count();
    }

    /// @brief Forward iterator that skips filtered-out events.
    class const_filtered_iterator {
    public:
        using iterator_category = std::forward_iterator_tag;
        using value_type = Event;
        using difference_type = std::ptrdiff_t;
        using pointer = const Event*;
        using reference = const Event&;

        const_filtered_iterator() = default;
        const_filtered_iterator(const FilteredEventPacket* owner, std::size_t idx)
            : owner_(owner), idx_(idx) { advance_to_valid(); }

        reference operator*() const { return (*owner_)[idx_]; }
        pointer operator->() const { return &(*owner_)[idx_]; }
        const_filtered_iterator& operator++() { ++idx_; advance_to_valid(); return *this; }
        const_filtered_iterator operator++(int) {
            const_filtered_iterator tmp = *this; ++(*this); return tmp;
        }
        bool operator==(const const_filtered_iterator& o) const { return idx_ == o.idx_; }
        bool operator!=(const const_filtered_iterator& o) const { return idx_ != o.idx_; }

    private:
        void advance_to_valid() {
            while (owner_ != nullptr && idx_ < owner_->size() &&
                   owner_->is_filtered(idx_)) {
                ++idx_;
            }
        }
        const FilteredEventPacket* owner_{nullptr};
        std::size_t idx_{0};
    };

    const_filtered_iterator begin_filtered() const {
        return const_filtered_iterator(this, 0);
    }
    const_filtered_iterator end_filtered() const {
        return const_filtered_iterator(this, packet_.size());
    }

private:
    EventPacket packet_;
    std::vector<bool> filtered_;
};

} // namespace gui_algo

#endif // GUI_ALGO_COMMON_EVENT_PACKET_H
