// Active regions as separate runs (RegionHost.hpp, huge-worlds.md §3).
#include "lagrange/server/RegionHost.hpp"

#include <algorithm>
#include <utility>

namespace lagrange::server {

namespace {

constexpr std::int64_t kRegionStepNs = 100'000'000;  // regions meet every 100 ms

}  // namespace

RegionHost::RegionHost(RegionHostOptions options)
    : options_(std::move(options)), planner_(options_.regions) {}

RegionHost::~RegionHost() = default;

Status RegionHost::load() {
    if (options_.robots.empty()) {
        return Status::error(StatusCode::InvalidConfig, "regions: no robots");
    }
    if (!(options_.regions.radius > 0.0)) {
        return Status::error(StatusCode::InvalidConfig,
                             "regions: world.streaming.active_region_radius must be given");
    }
    std::vector<std::array<double, 3>> starts;
    for (const ServerOptions::ExtraRobot& r : options_.robots) {
        starts.push_back(r.spawn.position);
    }
    assignment_ = planner_.plan(starts);
    servers_.clear();
    for (const std::vector<std::uint32_t>& members : assignment_) {
        ServerOptions options = options_.base;
        options.ros = false;
        const ServerOptions::ExtraRobot& lead = options_.robots.at(members.front());
        options.robotPath = lead.path;
        options.spawn = lead.spawn;
        options.extraRobots.clear();
        for (std::size_t k = 1; k < members.size(); ++k) {
            options.extraRobots.push_back(options_.robots.at(members[k]));
        }
        auto server = std::make_unique<Server>(options);
        if (Status status = server->load(); !status.ok()) {
            return Status::error(status.code, lead.name + "'s region: " + status.message);
        }
        servers_.push_back(std::move(server));
    }
    return {};
}

Status RegionHost::stepUntil(SimTime t) {
    if (servers_.empty()) {
        return Status::error(StatusCode::Internal, "regions: stepUntil before load()");
    }
    while (servers_.front()->now() < t) {
        const SimTime next{std::min(t.ns, servers_.front()->now().ns + kRegionStepNs)};
        for (const std::unique_ptr<Server>& server : servers_) {
            if (Status status = server->stepUntil(next); !status.ok()) {
                return status;
            }
        }
        std::vector<std::array<double, 3>> positions;
        for (std::uint32_t r = 0; r < options_.robots.size(); ++r) {
            positions.push_back(robotPosition(r));
        }
        if (planner_.plan(positions, assignment_) != assignment_) {
            return Status::error(StatusCode::Unsupported,
                                 "regions: robots met or parted, and moving a robot between "
                                 "regions is not built yet");
        }
    }
    return {};
}

std::array<double, 3> RegionHost::robotPosition(std::uint32_t robot) const {
    for (std::size_t g = 0; g < assignment_.size(); ++g) {
        const std::vector<std::uint32_t>& members = assignment_[g];
        const auto it = std::find(members.begin(), members.end(), robot);
        if (it == members.end()) {
            continue;
        }
        const Server& server = *servers_.at(g);
        // The region's robot is unprefixed; extras' bodies carry their name.
        const World& world = server.world();
        std::string body;
        if (it == members.begin()) {
            body = world.bodies.names.empty() ? "" : world.bodies.names.front();
        } else {
            const std::string front = options_.robots.at(robot).name + "_";
            for (const std::string& name : world.bodies.names) {
                if (name.rfind(front, 0) == 0) {
                    body = name;
                    break;
                }
            }
        }
        std::vector<double> pose = server.read("body/" + body + "/pose");
        std::array<double, 3> p{};
        for (std::size_t i = 0; i < 3 && i < pose.size(); ++i) {
            p.at(i) = pose[i] + world.region.worldEnu.at(i);
        }
        return p;
    }
    return {};
}

}  // namespace lagrange::server
