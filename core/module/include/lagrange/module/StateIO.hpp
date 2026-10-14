#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

namespace lagrange {

/// Serialises a module's dynamic state for save/restore and state hashes (RFC 0001, "State").
/// Values are written in host byte order: a saved state is for the same build and machine,
/// which is the scope of the determinism promise (docs/design/architecture.md §12).
class StateWriter {
public:
    template <class T>
    void put(const T& value) {
        static_assert(std::is_trivially_copyable_v<T>);
        putBytes(&value, sizeof(T));
    }
    void putString(std::string_view text) {
        put<std::uint64_t>(text.size());
        putBytes(text.data(), text.size());
    }
    void putDoubles(std::span<const double> values) {
        put<std::uint64_t>(values.size());
        putBytes(values.data(), values.size_bytes());
    }
    void putBytes(const void* data, std::size_t size) {
        if (size == 0) {
            return;
        }
        const auto* first = static_cast<const std::byte*>(data);
        bytes_.insert(bytes_.end(), first, first + size);
    }

    [[nodiscard]] std::span<const std::byte> bytes() const { return bytes_; }

private:
    std::vector<std::byte> bytes_;
};

/// Reads what StateWriter wrote. Any short read sets failed() and yields zeros.
class StateReader {
public:
    explicit StateReader(std::span<const std::byte> bytes) : bytes_(bytes) {}

    template <class T>
    bool get(T& value) {
        static_assert(std::is_trivially_copyable_v<T>);
        return getBytes(&value, sizeof(T));
    }
    bool getString(std::string& text) {
        std::uint64_t size = 0;
        if (!get(size) || size > remaining()) {
            failed_ = true;
            return false;
        }
        text.assign(reinterpret_cast<const char*>(bytes_.data() + position_), size);
        position_ += size;
        return true;
    }
    /// Reads a double array that must have exactly `out.size()` elements.
    bool getDoubles(std::span<double> out) {
        std::uint64_t size = 0;
        if (!get(size) || size != out.size()) {
            failed_ = true;
            return false;
        }
        return getBytes(out.data(), out.size_bytes());
    }
    bool getBytes(void* data, std::size_t size) {
        if (size == 0) {
            return !failed_;  // memcpy and memset need valid pointers even for zero bytes
        }
        if (failed_ || size > remaining()) {
            failed_ = true;
            std::memset(data, 0, size);
            return false;
        }
        std::memcpy(data, bytes_.data() + position_, size);
        position_ += size;
        return true;
    }

    [[nodiscard]] std::size_t remaining() const { return bytes_.size() - position_; }
    [[nodiscard]] bool failed() const { return failed_; }

private:
    std::span<const std::byte> bytes_;
    std::size_t position_{0};
    bool failed_{false};
};

}  // namespace lagrange
