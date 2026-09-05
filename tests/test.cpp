#include "motion_guard/world.hpp"
#include <iostream>
#include <random>
#include <string>

using namespace motion_guard;
#define CHECK(x)                                                                                   \
    do {                                                                                           \
        if (!(x))                                                                                  \
            throw std::runtime_error(#x);                                                          \
    } while (false)
Track line(std::uint64_t id, Vec a, Vec b, double radius = 1) {
    return {id, radius, {{0, a}, {1, b}}};
}
template <class F> void rejects(F f) {
    bool bad = false;
    try {
        f();
    } catch (const std::invalid_argument &) {
        bad = true;
    }
    CHECK(bad);
}
int main() {
    try {
        auto a = segments(line(1, {-10, 0}, {10, 0}))[0], b = segments(line(2, {0, 0}, {0, 0}))[0];
        auto hit = contact(a, b);
        CHECK(hit);
        CHECK(std::abs(hit->enter - .4) < 1e-12);
        CHECK(std::abs(hit->exit - .6) < 1e-12);
        // Both sampled endpoints are clear: discrete endpoint-only checks miss this.
        CHECK(norm(a.a.p - b.a.p) > 2 && norm(a.b.p - b.b.p) > 2);
        auto tangent = segments(line(3, {0, 2}, {0, 2}))[0];
        CHECK(contact(a, tangent));
        CHECK(std::abs(contact(a, tangent)->enter - .5) < 1e-12);
        CHECK(!contact(a, segments(line(4, {0, 2.001}, {0, 2.001}))[0]));
        CHECK(contact(a, segments(line(4, {0, 2.001}, {0, 2.001}))[0], .01));
        CHECK(contact(b, b)->enter == 0);
        CHECK(contact(b, b)->exit == 1);
        auto later = b;
        later.a.t = 2;
        later.b.t = 3;
        CHECK(!contact(a, later));
        World empty(1, {});
        CHECK(empty.audit(line(1, {0, 0}, {1, 1})).contacts.empty());
        rejects([&] { World w(1, {line(1, {0, 0}, {0, 0}), line(1, {2, 2}, {2, 2})}); });
        rejects([&] { empty.audit({1, 1, {{0, {0, 0}}, {0, {1, 1}}, {1, {2, 2}}}}); });
        rejects([&] { empty.audit(line(1, {NAN, 0}, {0, 0})); });
        rejects([&] { empty.audit(line(1, {0, 0}, {1, 1}), -1); });
        CHECK(!evaluate(empty, line(1, {0, 0}, {100, 0})).feasible);
        Track irregular{2, 1, {{0, {0, 0}}, {.3, {0, 0}}, {1, {0, 0}}}};
        World split(1, {irregular});
        CHECK(std::abs(split.audit(line(1, {-10, 0}, {10, 0})).contacts.front().enter - .4) <
              1e-12);

        std::mt19937_64 rng(19);
        auto coord = [&] { return static_cast<double>(rng() % 20001) / 100 - 100; };
        for (int i = 0; i < 100000; ++i) {
            Vec p0{coord(), coord()}, p1{coord(), coord()}, q0{coord(), coord()},
                q1{coord(), coord()};
            double r1 = .1 + static_cast<double>(rng() % 100) / 10,
                   r2 = .1 + static_cast<double>(rng() % 100) / 10;
            // Independent oracle: distance from origin to relative-motion line segment.
            double x = p0.x - q0.x, y = p0.y - q0.y, dx = p1.x - q1.x - x, dy = p1.y - q1.y - y;
            double den = dx * dx + dy * dy,
                   t = den == 0 ? 0 : std::clamp(-(x * dx + y * dy) / den, 0., 1.);
            double gap = std::hypot(x + dx * t, y + dy * t) - r1 - r2;
            auto actual =
                contact(segments(line(1, p0, p1, r1))[0], segments(line(2, q0, q1, r2))[0]);
            if (std::abs(gap) > 1e-8)
                CHECK(actual.has_value() == (gap <= 0));
        }
        std::vector<Track> obstacles;
        for (std::uint64_t i = 1; i <= 500; ++i)
            obstacles.push_back(line(i, {coord(), coord()}, {coord(), coord()}, .5));
        World world(1, obstacles);
        for (int i = 0; i < 500; ++i) {
            auto ego = line(1000, {coord(), coord()}, {coord(), coord()}, .8);
            auto fast = world.audit(ego, .2), slow = world.audit(ego, .2, true);
            CHECK(fast.contacts == slow.contacts);
        }
        std::cout << "PASS: analytical edges, 100000 independent geometry cases, 250000 "
                     "broad-phase reference pairs\n";
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
