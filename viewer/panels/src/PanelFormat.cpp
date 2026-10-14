#include "PanelFormat.hpp"

#include <array>
#include <cstdio>

namespace lagrange::viewer::format {

void clock(std::int64_t simTimeNs, char* buffer, std::size_t size) {
    const std::int64_t ms = simTimeNs / 1'000'000;
    std::snprintf(buffer, size, "%02lld:%02lld.%03lld", static_cast<long long>(ms / 60'000),
                  static_cast<long long>(ms / 1000 % 60), static_cast<long long>(ms % 1000));
}

void step(std::int64_t dtNs, char* buffer, std::size_t size) {
    if (dtNs <= 0) {
        std::snprintf(buffer, size, "-");
    } else if (dtNs < 1'000) {
        std::snprintf(buffer, size, "%lld ns", static_cast<long long>(dtNs));
    } else if (dtNs < 1'000'000) {
        std::snprintf(buffer, size, "%g µs", static_cast<double>(dtNs) * 1e-3);
    } else if (dtNs < 1'000'000'000) {
        std::snprintf(buffer, size, "%g ms", static_cast<double>(dtNs) * 1e-6);
    } else {
        std::snprintf(buffer, size, "%g s", static_cast<double>(dtNs) * 1e-9);
    }
}

void domainDetail(const DomainView& view, char* buffer, std::size_t size) {
    const char* pending = view.pending ? " · pending" : "";
    if (!view.enabled) {
        std::snprintf(buffer, size, "→ %s%s", view.mode.empty() ? "off" : view.mode.c_str(),
                      pending);
        return;
    }
    char dt[24];
    step(view.dtNs, dt, sizeof(dt));
    if (view.mode.empty()) {
        std::snprintf(buffer, size, "%s%s", dt, pending);
    } else if (view.dtNs > 0) {
        std::snprintf(buffer, size, "%s · %s%s", view.mode.c_str(), dt, pending);
    } else {
        std::snprintf(buffer, size, "%s%s", view.mode.c_str(), pending);
    }
}

bool isPhysicsDomain(std::string_view name) {
    constexpr std::array<std::string_view, 8> kDomains{"rigid_body",   "electrical", "thermal",
                                                       "aerodynamics", "weather",    "emi",
                                                       "radiation",    "sensors"};
    for (const std::string_view domain : kDomains) {
        if (domain == name) {
            return true;
        }
    }
    return false;
}

}  // namespace lagrange::viewer::format
