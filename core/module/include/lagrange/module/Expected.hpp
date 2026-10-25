#pragma once

#include <optional>
#include <type_traits>
#include <utility>
#include <variant>

namespace lagrange {

/// The error side of an Expected, as in C++23's std::unexpected.
template <class E>
struct Unexpected {
    E error;
};

template <class E>
Unexpected(E) -> Unexpected<E>;

/// A value or an error: the subset of C++23's std::expected the core needs.
///
/// The project is limited to C++20 (CLAUDE.md §5); RFC 0001 chose this in-tree type over
/// tl::expected. Replace with std::expected when the language baseline allows.
template <class T, class E>
class Expected {
public:
    Expected(T value) : state_(std::in_place_index<0>, std::move(value)) {}  // NOLINT: implicit
    Expected(Unexpected<E> error)                                            // NOLINT: implicit
        : state_(std::in_place_index<1>, std::move(error.error)) {}

    [[nodiscard]] bool has_value() const { return state_.index() == 0; }
    explicit operator bool() const { return has_value(); }

    [[nodiscard]] T& value() & { return std::get<0>(state_); }
    [[nodiscard]] const T& value() const& { return std::get<0>(state_); }
    [[nodiscard]] T&& value() && { return std::get<0>(std::move(state_)); }
    [[nodiscard]] E& error() & { return std::get<1>(state_); }
    [[nodiscard]] const E& error() const& { return std::get<1>(state_); }

    T* operator->() { return &value(); }
    const T* operator->() const { return &value(); }
    T& operator*() & { return value(); }
    const T& operator*() const& { return value(); }

private:
    std::variant<T, E> state_;
};

/// Success with no value, or an error.
template <class E>
class Expected<void, E> {
public:
    Expected() = default;
    Expected(Unexpected<E> error) : error_(std::move(error.error)) {}  // NOLINT: implicit

    [[nodiscard]] bool has_value() const { return !error_.has_value(); }
    explicit operator bool() const { return has_value(); }

    [[nodiscard]] E& error() & { return *error_; }
    [[nodiscard]] const E& error() const& { return *error_; }

private:
    std::optional<E> error_;
};

}  // namespace lagrange
