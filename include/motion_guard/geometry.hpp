#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <vector>

namespace motion_guard {
struct Vec {
    double x{}, y{};
};
inline Vec operator+(Vec a, Vec b) { return {a.x + b.x, a.y + b.y}; }
inline Vec operator-(Vec a, Vec b) { return {a.x - b.x, a.y - b.y}; }
inline Vec operator*(Vec a, double s) { return {a.x * s, a.y * s}; }
inline double norm(Vec a) { return std::hypot(a.x, a.y); }
struct Sample {
    double t;
    Vec p;
};
struct Track {
    std::uint64_t id;
    double radius;
    std::vector<Sample> samples;
};
struct Segment {
    std::uint64_t id;
    std::size_t ordinal;
    double radius;
    Sample a, b;
    Vec at(double t) const { return a.p + (b.p - a.p) * ((t - a.t) / (b.t - a.t)); }
};
inline void validate(const Track &track, double horizon) {
    if (!track.id || !std::isfinite(track.radius) || track.radius <= 0 || track.radius > 100 ||
        track.samples.size() < 2)
        throw std::invalid_argument("invalid track id, radius, or sample count");
    if (track.samples.front().t != 0 || track.samples.back().t != horizon)
        throw std::invalid_argument("track must cover the complete [0,horizon] interval");
    double last = -1;
    for (const auto &s : track.samples) {
        if (!std::isfinite(s.t) || s.t <= last || s.t > horizon || !std::isfinite(s.p.x) ||
            !std::isfinite(s.p.y) || std::abs(s.p.x) > 1e6 || std::abs(s.p.y) > 1e6 ||
            (last >= 0 && s.t - last < 1e-6))
            throw std::invalid_argument("invalid coordinates or sample times");
        last = s.t;
    }
}
inline std::vector<Segment> segments(const Track &t) {
    std::vector<Segment> out;
    for (std::size_t i = 1; i < t.samples.size(); ++i)
        out.push_back({t.id, i - 1, t.radius, t.samples[i - 1], t.samples[i]});
    return out;
}
struct Box {
    double x0, y0, x1, y1, t0, t1;
    bool overlaps(const Box &b) const {
        return x0 <= b.x1 && b.x0 <= x1 && y0 <= b.y1 && b.y0 <= y1 && t0 <= b.t1 && b.t0 <= t1;
    }
};
inline Box bounds(const Segment &s, double margin = 0) {
    // Outward rounding protects broad-phase inclusiveness at touching boundaries.
    const double r = s.radius + margin;
    auto down = [](double x) { return std::nextafter(x, -INFINITY); };
    auto up = [](double x) { return std::nextafter(x, INFINITY); };
    return {down(std::min(s.a.p.x, s.b.p.x) - r),
            down(std::min(s.a.p.y, s.b.p.y) - r),
            up(std::max(s.a.p.x, s.b.p.x) + r),
            up(std::max(s.a.p.y, s.b.p.y) + r),
            s.a.t,
            s.b.t};
}
inline Box unite(Box a, Box b) {
    return {std::min(a.x0, b.x0), std::min(a.y0, b.y0), std::max(a.x1, b.x1),
            std::max(a.y1, b.y1), std::min(a.t0, b.t0), std::max(a.t1, b.t1)};
}
struct Contact {
    std::uint64_t obstacle;
    std::size_t ego_segment, obstacle_segment;
    double enter, exit, closest_time, clearance;
    bool operator==(const Contact &) const = default;
};
// Exact continuous-time solution for the piecewise-linear disc model, subject
// to floating-point rounding. No sampling interval can tunnel through a contact.
inline std::optional<Contact> contact(const Segment &a, const Segment &b, double margin = 0) {
    const double lo = std::max(a.a.t, b.a.t), hi = std::min(a.b.t, b.b.t);
    if (lo > hi)
        return std::nullopt;
    auto ap = a.at(lo), bp = b.at(lo);
    auto av = (a.b.p - a.a.p) * (1 / (a.b.t - a.a.t)), bv = (b.b.p - b.a.p) * (1 / (b.b.t - b.a.t));
    const long double px = static_cast<long double>(ap.x) - bp.x,
                      py = static_cast<long double>(ap.y) - bp.y;
    const long double vx = static_cast<long double>(av.x) - bv.x,
                      vy = static_cast<long double>(av.y) - bv.y;
    const long double radius = static_cast<long double>(a.radius) + b.radius + margin;
    const long double aa = vx * vx + vy * vy, bb = px * vx + py * vy,
                      cc = px * px + py * py - radius * radius;
    const long double duration = hi - lo;
    const long double closest = aa == 0 ? 0 : std::clamp(-bb / aa, 0.L, duration);
    const long double dx = px + vx * closest, dy = py + vy * closest;
    const double clearance =
        static_cast<double>(std::sqrt(dx * dx + dy * dy) - a.radius - b.radius);
    long double enter = 0, leave = duration;
    if (aa == 0) {
        if (cc > 0)
            return std::nullopt;
    } else {
        const long double discriminant = bb * bb - aa * cc;
        if (discriminant < 0)
            return std::nullopt;
        const long double root = std::sqrt(discriminant);
        // Stable quadratic roots for aa*t*t + 2*bb*t + cc = 0.
        const long double q = -bb - std::copysign(root, bb);
        long double first, second;
        if (q == 0)
            first = second = -bb / aa;
        else {
            first = q / aa;
            second = cc / q;
            if (first > second)
                std::swap(first, second);
        }
        enter = std::max(0.L, first);
        leave = std::min(duration, second);
        if (enter > leave)
            return std::nullopt;
    }
    return Contact{b.id,
                   a.ordinal,
                   b.ordinal,
                   lo + static_cast<double>(enter),
                   lo + static_cast<double>(leave),
                   lo + static_cast<double>(closest),
                   clearance};
}
} // namespace motion_guard
