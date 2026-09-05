#pragma once
#include "geometry.hpp"
#include <limits>
#include <numeric>
#include <set>

namespace motion_guard {
struct Audit {
    std::vector<Contact> contacts;
    std::size_t narrow_checks{}, node_visits{};
};
class World {
    struct Node {
        Box box;
        std::size_t begin, end;
        int left = -1, right = -1;
    };
    double horizon_;
    std::vector<Segment> obstacles_;
    std::vector<std::size_t> order_;
    std::vector<Node> nodes_;
    int build(std::size_t begin, std::size_t end) {
        Box box = bounds(obstacles_[order_[begin]]);
        for (auto i = begin + 1; i < end; ++i)
            box = unite(box, bounds(obstacles_[order_[i]]));
        int index = static_cast<int>(nodes_.size());
        nodes_.push_back({box, begin, end});
        if (end - begin > 8) {
            bool split_x = box.x1 - box.x0 >= box.y1 - box.y0;
            auto mid = begin + (end - begin) / 2;
            std::nth_element(order_.begin() + begin, order_.begin() + mid, order_.begin() + end,
                             [&](auto a, auto b) {
                                 const auto x = bounds(obstacles_[a]), y = bounds(obstacles_[b]);
                                 auto ca = split_x ? x.x0 + x.x1 : x.y0 + x.y1,
                                      cb = split_x ? y.x0 + y.x1 : y.y0 + y.y1;
                                 return ca == cb ? a < b : ca < cb;
                             });
            int left = build(begin, mid), right = build(mid, end);
            nodes_[index].left = left;
            nodes_[index].right = right;
        }
        return index;
    }
    void visit(int index, const Segment &ego, const Box &query, double margin, Audit &out) const {
        const auto &node = nodes_[index];
        ++out.node_visits;
        if (!node.box.overlaps(query))
            return;
        if (node.left >= 0) {
            visit(node.left, ego, query, margin, out);
            visit(node.right, ego, query, margin, out);
            return;
        }
        for (auto i = node.begin; i < node.end; ++i) {
            const auto &other = obstacles_[order_[i]];
            if (!bounds(other).overlaps(query))
                continue;
            ++out.narrow_checks;
            if (auto hit = contact(ego, other, margin))
                out.contacts.push_back(*hit);
        }
    }

  public:
    World(double horizon, const std::vector<Track> &tracks) : horizon_(horizon) {
        if (!std::isfinite(horizon) || horizon <= 0 || horizon > 3600)
            throw std::invalid_argument("invalid horizon");
        std::set<std::uint64_t> ids;
        for (const auto &t : tracks) {
            validate(t, horizon);
            if (!ids.insert(t.id).second)
                throw std::invalid_argument("duplicate obstacle id");
            auto seg = segments(t);
            obstacles_.insert(obstacles_.end(), seg.begin(), seg.end());
        }
        order_.resize(obstacles_.size());
        std::iota(order_.begin(), order_.end(), 0);
        if (!order_.empty())
            build(0, order_.size());
    }
    Audit audit(const Track &ego, double margin = 0, bool brute = false) const {
        validate(ego, horizon_);
        if (!std::isfinite(margin) || margin < 0 || margin > 100)
            throw std::invalid_argument("invalid margin");
        Audit out;
        for (const auto &s : segments(ego)) {
            if (brute)
                for (const auto &o : obstacles_) {
                    ++out.narrow_checks;
                    if (auto hit = contact(s, o, margin))
                        out.contacts.push_back(*hit);
                }
            else if (!nodes_.empty())
                visit(0, s, bounds(s, margin), margin, out);
        }
        std::sort(out.contacts.begin(), out.contacts.end(), [](const Contact &a, const Contact &b) {
            if (a.enter != b.enter)
                return a.enter < b.enter;
            if (a.obstacle != b.obstacle)
                return a.obstacle < b.obstacle;
            if (a.ego_segment != b.ego_segment)
                return a.ego_segment < b.ego_segment;
            return a.obstacle_segment < b.obstacle_segment;
        });
        return out;
    }
};
struct Limits {
    double speed = 20, acceleration = 8;
};
struct Evaluation {
    bool feasible;
    double max_speed, max_acceleration;
    Audit collision;
};
inline Evaluation evaluate(const World &world, const Track &candidate, Limits limits = {},
                           double margin = 0) {
    if (!std::isfinite(limits.speed) || !std::isfinite(limits.acceleration) || limits.speed <= 0 ||
        limits.acceleration <= 0)
        throw std::invalid_argument("invalid kinematic limits");
    auto audit = world.audit(candidate, margin); // validates before finite differences
    double max_speed = 0, max_acceleration = 0;
    auto seg = segments(candidate);
    Vec previous{};
    double previous_dt = 0;
    for (std::size_t i = 0; i < seg.size(); ++i) {
        double dt = seg[i].b.t - seg[i].a.t;
        auto v = (seg[i].b.p - seg[i].a.p) * (1 / dt);
        max_speed = std::max(max_speed, norm(v));
        if (i)
            max_acceleration =
                std::max(max_acceleration, norm(v - previous) / ((dt + previous_dt) / 2));
        previous = v;
        previous_dt = dt;
    }
    bool feasible = audit.contacts.empty() && max_speed <= limits.speed &&
                    max_acceleration <= limits.acceleration;
    return {feasible, max_speed, max_acceleration, std::move(audit)};
}
} // namespace motion_guard
