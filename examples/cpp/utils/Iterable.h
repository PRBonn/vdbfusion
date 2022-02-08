#pragma once
#include <utility>

namespace utils {
template <typename T>
constexpr auto iterable(T&& indexable) {
    struct iterator {
        std::size_t idx{0};
        T indexable;
        bool operator!=(const iterator& other) const { return idx != other.idx; }
        void operator++() { ++idx; }
        auto operator*() const { return indexable[idx]; }
    };
    struct indexable_wrapper {
        T indexable;
        auto begin() { return iterator{0, indexable}; }
        auto end() { return iterator{indexable.size(), indexable}; }
    };
    return indexable_wrapper{std::forward<T>(indexable)};
}

}  // namespace utils
