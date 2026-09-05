#include "motion_guard/world.hpp"
#include <charconv>
#include <chrono>
#include <fstream>
#include <iostream>
#include <map>
#include <random>
#include <sstream>
#include <string>

using namespace motion_guard;
std::vector<Track> read_csv(const std::string &path) {
    std::ifstream in(path);
    if (!in)
        throw std::runtime_error("cannot open CSV");
    std::map<std::uint64_t, Track> tracks;
    std::string line;
    std::size_t lineno = 0;
    while (std::getline(in, line)) {
        ++lineno;
        if (line.empty() || line[0] == '#')
            continue;
        std::replace(line.begin(), line.end(), ',', ' ');
        std::istringstream row(line);
        std::uint64_t id{};
        double r, t, x, y;
        std::string extra, id_text;
        if (!(row >> id_text >> r >> t >> x >> y) || row >> extra)
            throw std::runtime_error("bad CSV row " + std::to_string(lineno));
        auto parsed = std::from_chars(id_text.data(), id_text.data() + id_text.size(), id);
        if (parsed.ec != std::errc{} || parsed.ptr != id_text.data() + id_text.size() || id == 0)
            throw std::runtime_error("invalid positive track id on row " + std::to_string(lineno));
        auto [it, fresh] = tracks.try_emplace(id, Track{id, r, {}});
        if (!fresh && it->second.radius != r)
            throw std::runtime_error("radius changed within track");
        it->second.samples.push_back({t, {x, y}});
    }
    if (in.bad())
        throw std::runtime_error("CSV read failure");
    std::vector<Track> out;
    for (auto &[id, track] : tracks) {
        (void)id;
        out.push_back(std::move(track));
    }
    return out;
}
void report(const World &world, const Track &ego) {
    auto e = evaluate(world, ego);
    std::cout << "{\"candidate\":" << ego.id
              << ",\"accepted_by_model\":" << (e.feasible ? "true" : "false")
              << ",\"contact_pairs\":" << e.collision.contacts.size()
              << ",\"max_speed\":" << e.max_speed
              << ",\"acceleration_proxy\":" << e.max_acceleration;
    if (!e.collision.contacts.empty())
        std::cout << ",\"first_contact_s\":" << e.collision.contacts.front().enter;
    std::cout << "}\n";
}
int main(int argc, char **argv) {
    try {
        if (argc == 2 && std::string(argv[1]) == "--scenario") {
            World crossing(4, {{10, 1, {{0, {0, -8}}, {4, {0, 8}}}}});
            report(crossing, {1, 1, {{0, {-8, 0}}, {4, {8, 0}}}});
            report(crossing, {2, 1, {{0, {-8, 0}}, {2, {-8, 0}}, {4, {-4, 0}}}});
            return 0;
        }
        if (argc == 4 && std::string(argv[1]) == "--audit") {
            auto obstacles = read_csv(argv[2]), candidates = read_csv(argv[3]);
            if (candidates.empty())
                throw std::runtime_error("no candidates");
            World world(candidates.front().samples.back().t, obstacles);
            for (const auto &candidate : candidates)
                report(world, candidate);
            return 0;
        }
        if (argc != 1) {
            std::cerr << "usage: experiment [--scenario | --audit obstacles.csv candidates.csv]\n";
            return 2;
        }
        std::vector<Track> tracks;
        std::mt19937_64 rng(42);
        auto coord = [&] { return static_cast<double>(rng() % 100000) / 100 - 500; };
        for (std::uint64_t i = 1; i <= 10000; ++i) {
            double x = coord(), y = coord();
            tracks.push_back({i, 1, {{0, {x, y}}, {5, {x + 5, y + 2}}, {10, {x + 10, y + 4}}}});
        }
        using Clock = std::chrono::steady_clock;
        auto begin = Clock::now();
        World world(10, tracks);
        double build_ms = std::chrono::duration<double, std::milli>(Clock::now() - begin).count();
        std::vector<Track> candidates;
        for (std::uint64_t i = 0; i < 500; ++i) {
            double x = coord(), y = coord();
            candidates.push_back({20000 + i, 1, {{0, {x, y}}, {10, {x + 30, y}}}});
        }
        std::vector<std::vector<Contact>> baseline;
        for (bool brute : {true, false}) {
            std::size_t checks = 0, hits = 0;
            begin = Clock::now();
            std::size_t index = 0;
            for (const auto &candidate : candidates) {
                auto a = world.audit(candidate, 0, brute);
                checks += a.narrow_checks;
                hits += a.contacts.size();
                if (brute)
                    baseline.push_back(a.contacts);
                else if (a.contacts != baseline[index])
                    throw std::runtime_error("contact mismatch");
                ++index;
            }
            double ms = std::chrono::duration<double, std::milli>(Clock::now() - begin).count();
            std::cout << "{\"mode\":\"" << (brute ? "all-pairs" : "bvh")
                      << "\",\"obstacles\":10000,\"queries\":500,\"build_ms\":" << build_ms
                      << ",\"query_ms\":" << ms << ",\"narrow_checks\":" << checks
                      << ",\"contact_pairs\":" << hits << "}\n";
        }
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
