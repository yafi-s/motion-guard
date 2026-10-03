#include "motion_guard/world.hpp"
#include <chrono>
#include <charconv>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <tuple>

using namespace motion_guard;
std::vector<Track> load(const std::string &path) {
    std::ifstream file(path);
    if (!file) throw std::runtime_error("cannot open scenario CSV");
    std::map<std::uint64_t,Track> tracks;
    std::string line;
    std::size_t rows=0;
    while (std::getline(file,line)) {
        if (++rows>100000) throw std::runtime_error("scenario row limit");
        std::replace(line.begin(),line.end(),',',' ');
        std::istringstream in(line);
        std::uint64_t id{};
        double radius,t,x,y;
        std::string extra,id_text;
        if (!(in>>id_text>>radius>>t>>x>>y) || in>>extra) throw std::runtime_error("bad scenario row");
        const auto parsed=std::from_chars(id_text.data(),id_text.data()+id_text.size(),id);
        if (parsed.ec!=std::errc{} || parsed.ptr!=id_text.data()+id_text.size() || id==0)
            throw std::runtime_error("invalid positive track id");
        auto [it,inserted]=tracks.try_emplace(id,Track{id,radius,{}});
        if (!inserted && it->second.radius!=radius) throw std::runtime_error("inconsistent radius");
        it->second.samples.push_back({t,{x,y}});
    }
    if (file.bad()) throw std::runtime_error("scenario read error");
    std::vector<Track> result;
    for (auto &[id,track]:tracks) { (void)id; result.push_back(std::move(track)); }
    return result;
}
bool parity(const Audit &a,const Audit &b) {
    if (a.contacts.size()!=b.contacts.size()) return false;
    for (std::size_t i=0;i<a.contacts.size();++i) {
        const auto &x=a.contacts[i], &y=b.contacts[i];
        if (std::tie(x.obstacle,x.ego_segment,x.obstacle_segment,x.enter,x.exit)!=
            std::tie(y.obstacle,y.ego_segment,y.obstacle_segment,y.enter,y.exit)) return false;
    }
    return true;
}
int main(int argc,char **argv) {
    try {
        if (argc!=3) throw std::runtime_error("usage: scenario_eval tracks.csv margin");
        auto tracks=load(argv[1]);
        std::size_t parsed=0;
        const double margin=std::stod(argv[2],&parsed);
        if (parsed!=std::string(argv[2]).size() || !std::isfinite(margin) || margin<0 || margin>100)
            throw std::runtime_error("invalid margin");
        if (tracks.size()<2 || tracks.size()>128) throw std::runtime_error("invalid track count");
        const double horizon=tracks.front().samples.back().t;
        using Clock=std::chrono::steady_clock;
        std::cout<<std::setprecision(17);
        for (const auto &ego:tracks) {
            std::vector<Track> others;
            for (const auto &track:tracks) if (track.id!=ego.id) others.push_back(track);
            const auto begin=Clock::now();
            World world(horizon,others);
            const auto built=Clock::now();
            auto fast=world.audit(ego,margin);
            const auto audited=Clock::now();
            auto brute=world.audit(ego,margin,true);
            const auto done=Clock::now();
            if (!parity(fast,brute)) throw std::runtime_error("BVH/brute interval mismatch");
            std::set<std::uint64_t> actor_ids;
            for (const auto &hit:fast.contacts) actor_ids.insert(hit.obstacle);
            std::cout<<"{\"track\":"<<ego.id<<",\"margin\":"<<margin
                     <<",\"narrow_checks\":"<<fast.narrow_checks<<",\"brute_checks\":"<<brute.narrow_checks
                     <<",\"segment_contacts\":"<<fast.contacts.size()<<",\"build_ms\":"
                     <<std::chrono::duration<double,std::milli>(built-begin).count()
                     <<",\"audit_ms\":"<<std::chrono::duration<double,std::milli>(audited-built).count()
                     <<",\"brute_ms\":"<<std::chrono::duration<double,std::milli>(done-audited).count()
                     <<",\"contacts\":[";
            bool first=true;
            for (auto id:actor_ids) { if (!first) std::cout<<','; std::cout<<id; first=false; }
            std::cout<<"]}\n";
        }
    } catch (const std::exception &error) { std::cerr<<error.what()<<'\n'; return 2; }
}
