#include "App.hpp"
#include "lagrange/gpu/Device.hpp"
#include "lagrange/gpu/Resources.hpp"
#include "lagrange/proto/Endpoint.hpp"
#include "lagrange/viewer/Fonts.hpp"
#include "lagrange/viewer/Panel.hpp"
#include "lagrange/viewer/SceneRenderer.hpp"
#include "lagrange/viewer/ViewerClient.hpp"
#include "lagrange/viewer/lagrange_theme.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <imgui_impl_sdl3.h>
#include <imgui_impl_vulkan.h>
#include <map>
#include <memory>
#include <mutex>
#include <numbers>
#include <random>
#include <signal.h>
#include <spawn.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
#include <vector>

#include <SDL3/SDL.h>
#include <SDL3/SDL_vulkan.h>
#include <imgui.h>
#include <implot.h>

namespace lagrange::viewer {

namespace {

// The lidar bird's-eye view's two tones (design 06-sensors: the sensor colour, strong and
// attenuated), and the intensity between them; the viewport overlay draws in the same.
const ImVec4 kLidarStrong = theme::kDomainColor[static_cast<int>(theme::Domain::Sensors)];
const ImVec4 kLidarAttenuated = theme::hex(0x2A8F76);
constexpr float kLidarStrongIntensity = 0.12F;

using Clock = std::chrono::steady_clock;

/// Where the bundled fonts are: an explicit directory, the installed share directory next to
/// the program, or the source tree (when run from a build directory).
std::string fontDirectory() {
    // NOLINTNEXTLINE(concurrency-mt-unsafe): start-up, single-threaded.
    if (const char* override = std::getenv("LAGRANGE_VIEWER_FONTS");
        override != nullptr && *override != '\0') {
        return override;
    }
    std::error_code ec;
    if (const char* base = SDL_GetBasePath(); base != nullptr) {
        // Installed layout: <prefix>/lib/lagrange_viewer/lagrange_viewer.
        const std::filesystem::path installed =
            std::filesystem::path(base) / "../../share/lagrange_viewer/fonts";
        if (std::filesystem::is_directory(installed, ec)) {
            return installed.string();
        }
    }
    return LAGRANGE_VIEWER_SOURCE_FONTS;
}

std::string environment(const char* name) {
    // NOLINTNEXTLINE(concurrency-mt-unsafe): start-up, single-threaded.
    const char* value = std::getenv(name);
    return value != nullptr ? value : "";
}

/// "2026-10-14 23:12" in local time.
std::string localTime() {
    const std::time_t now = std::time(nullptr);
    std::tm parts{};
    localtime_r(&now, &parts);
    char text[32];
    std::strftime(text, sizeof(text), "%Y-%m-%d %H:%M", &parts);
    return text;
}

/// What this machine offers, for the launcher's system check. Probed locally.
/// Where the viewer keeps its files, made if it is not there.
std::string configDirectory() {
    const std::string configHome = environment("XDG_CONFIG_HOME");
    const std::string home = environment("HOME");
    std::string directory =
        (configHome.empty() ? (home.empty() ? std::string(".") : home) + "/.config" : configHome) +
        "/lagrange";
    std::error_code ec;
    std::filesystem::create_directories(directory, ec);
    return directory;
}

SystemInfo systemInfo(const gpu::Device& device, float displayScale) {
    SystemInfo info;
    const gpu::DeviceInfo& gpuInfo = device.info();
    info.gpu = gpuInfo.name;
    char detail[160];
    std::snprintf(detail, sizeof(detail), "Vulkan %u.%u · ray query %s · %.0f GB", gpuInfo.apiMajor,
                  gpuInfo.apiMinor, gpuInfo.rayQuery ? "yes" : "no",
                  static_cast<double>(gpuInfo.deviceMemoryBytes) / (1024.0 * 1024.0 * 1024.0));
    info.gpuDetail = detail;
    info.tier = gpu::toString(device.tier());
    info.tier = info.tier.substr(0, info.tier.find(" ("));
    info.softwareGpu = gpuInfo.isSoftware;
    // PRETTY_NAME="Ubuntu 26.04 LTS" in /etc/os-release.
    std::ifstream release("/etc/os-release");
    for (std::string line; std::getline(release, line);) {
        constexpr std::string_view kKey = "PRETTY_NAME=";
        if (line.starts_with(kKey)) {
            info.os = line.substr(kKey.size());
            std::erase(info.os, '"');
        }
    }
    info.session = environment("XDG_SESSION_TYPE");
    info.scale = displayScale;
    info.rosDistro = environment("ROS_DISTRO");
    info.rmw = environment("RMW_IMPLEMENTATION");
    info.rosDomain = environment("ROS_DOMAIN_ID");
    info.configDirectory = configDirectory();
    return info;
}

// Everything both modes share: ImGui, the panels, the camera and the scene renderer.
/// lagrange_server in the ROS install the viewer runs from, or empty.
std::string findLocalServer() {
    const char* prefixes = std::getenv("AMENT_PREFIX_PATH");
    if (prefixes == nullptr) {
        return {};
    }
    std::string list = prefixes;
    std::size_t start = 0;
    while (start <= list.size()) {
        const std::size_t end = std::min(list.find(':', start), list.size());
        const std::filesystem::path candidate =
            std::filesystem::path(list.substr(start, end - start)) /
            "lib/lagrange_server/lagrange_server";
        std::error_code ec;
        if (end > start && std::filesystem::is_regular_file(candidate, ec)) {
            return candidate.string();
        }
        start = end + 1;
    }
    return {};
}

/// A simulation server started by the viewer from the launcher. Stopped with the viewer.
class LocalServer {
public:
    LocalServer() = default;
    LocalServer(const LocalServer&) = delete;
    LocalServer& operator=(const LocalServer&) = delete;
    ~LocalServer() { stop(); }

    /// Starts `program --listen <endpoint>`. Returns an error message, or empty on success.
    std::string start(const std::string& program, const std::string& endpoint) {
        stop();
        std::vector<std::string> args{program, "--listen", endpoint};
        std::vector<char*> argv;
        for (std::string& arg : args) {
            argv.push_back(arg.data());
        }
        argv.push_back(nullptr);
        const int error =
            ::posix_spawn(&pid_, program.c_str(), nullptr, nullptr, argv.data(), environ);
        if (error != 0) {
            pid_ = -1;
            return "could not start " + program + ": " + std::strerror(error);
        }
        return {};
    }

    /// Asks the server to finish (SIGTERM) and waits for it.
    void stop() {
        if (pid_ > 0) {
            ::kill(pid_, SIGTERM);
            int status = 0;
            ::waitpid(pid_, &status, 0);
            pid_ = -1;
        }
    }

private:
    pid_t pid_{-1};
};

/// Sensor images as textures for the panels. An image is copied into a host-visible staging
/// buffer when a panel asks for it and copied to its texture in the frame's command buffer,
/// before the UI pass. The viewer has one frame in flight, so both are free by the next frame.
class TextureCache final : public SensorTextures {
public:
    explicit TextureCache(const gpu::Device& device) : device_(device) {}
    ~TextureCache() override { clear(); }
    TextureCache(const TextureCache&) = delete;
    TextureCache& operator=(const TextureCache&) = delete;

    std::uint64_t texture(const std::string& key,
                          const std::shared_ptr<const SensorImage>& image) override {
        if (!image || image->width == 0 || image->height == 0) {
            return 0;
        }
        Entry& entry = entries_[key];
        if (entry.image.extent.width != image->width ||
            entry.image.extent.height != image->height) {
            if (entry.descriptor != VK_NULL_HANDLE) {
                vkDeviceWaitIdle(device_.device());
            }
            release(entry);
            entry.image =
                gpu::createImage2D(device_, {image->width, image->height}, VK_FORMAT_R8G8B8A8_UNORM,
                                   VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT,
                                   VK_IMAGE_ASPECT_COLOR_BIT);
            entry.staging = gpu::createBuffer(device_, image->rgba.size(),
                                              VK_BUFFER_USAGE_TRANSFER_SRC_BIT, true);
            entry.descriptor = ImGui_ImplVulkan_AddTexture(
                entry.image.view, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
            entry.initialised = false;
            entry.shown.reset();
        }
        if (entry.shown != image) {
            std::memcpy(entry.staging.mapped, image->rgba.data(), image->rgba.size());
            gpu::flush(device_, entry.staging);
            entry.shown = image;
            entry.pending = true;
        }
        return static_cast<std::uint64_t>(reinterpret_cast<std::uintptr_t>(entry.descriptor));
    }

    /// Records the copies of the images asked for since the last frame.
    void record(VkCommandBuffer commands) {
        for (auto& [key, entry] : entries_) {
            if (!entry.pending) {
                continue;
            }
            gpu::transitionImage(commands, entry.image.image, VK_IMAGE_ASPECT_COLOR_BIT,
                                 entry.initialised ? VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL
                                                   : VK_IMAGE_LAYOUT_UNDEFINED,
                                 VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);
            VkBufferImageCopy region{};
            region.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
            region.imageExtent = {entry.image.extent.width, entry.image.extent.height, 1};
            vkCmdCopyBufferToImage(commands, entry.staging.buffer, entry.image.image,
                                   VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &region);
            gpu::transitionImage(commands, entry.image.image, VK_IMAGE_ASPECT_COLOR_BIT,
                                 VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                                 VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
            entry.pending = false;
            entry.initialised = true;
        }
    }

    /// Frees every texture; call before the ImGui Vulkan backend shuts down.
    void clear() {
        if (entries_.empty()) {
            return;
        }
        vkDeviceWaitIdle(device_.device());
        for (auto& [key, entry] : entries_) {
            release(entry);
        }
        entries_.clear();
    }

private:
    struct Entry {
        gpu::Image image;
        gpu::Buffer staging;
        VkDescriptorSet descriptor{VK_NULL_HANDLE};
        std::shared_ptr<const SensorImage> shown;
        bool pending{false};
        bool initialised{false};
    };

    void release(Entry& entry) {
        if (entry.descriptor != VK_NULL_HANDLE) {
            ImGui_ImplVulkan_RemoveTexture(entry.descriptor);
            entry.descriptor = VK_NULL_HANDLE;
        }
        gpu::destroy(device_, entry.image);
        gpu::destroy(device_, entry.staging);
    }

    const gpu::Device& device_;
    std::map<std::string, Entry, std::less<>> entries_;
};

class Ui {
public:
    Ui(const gpu::Device& device, ViewerClient& client, VkFormat targetFormat,
       std::uint32_t imageCount, float dpiScale, bool persistLayout,
       std::vector<std::string> scopeSignals)
        : device_(device)
        , client_(client)
        , renderer_(device)
        , panels_(createDefaultPanels())
        , scopeSignals_(std::move(scopeSignals)) {
        renderOptions_.wireframeAvailable = renderer_.wireframeSupported();
        IMGUI_CHECKVERSION();
        ImGui::CreateContext();
        ImPlot::CreateContext();
        ImGuiIO& io = ImGui::GetIO();
        io.ConfigFlags |= ImGuiConfigFlags_DockingEnable | ImGuiConfigFlags_NavEnableKeyboard;
        if (persistLayout) {
            // NOLINTNEXTLINE(concurrency-mt-unsafe): start-up, single-threaded.
            const char* configHome = std::getenv("XDG_CONFIG_HOME");
            // NOLINTNEXTLINE(concurrency-mt-unsafe)
            const char* home = std::getenv("HOME");
            std::filesystem::path directory =
                configHome != nullptr && *configHome != '\0'
                    ? std::filesystem::path(configHome)
                    : std::filesystem::path(home != nullptr ? home : ".") / ".config";
            directory /= "lagrange/viewer";
            std::error_code ec;
            std::filesystem::create_directories(directory, ec);
            iniPath_ = (directory / "layout.ini").string();
            io.IniFilename = iniPath_.c_str();
        } else {
            io.IniFilename = nullptr;
        }
        // The whole look comes from the design system (docs/design/ui/00-design-system).
        theme::apply(dpiScale);
        if (!theme::loadFonts(fontDirectory()).bundled) {
            std::fputs(
                "warning: IBM Plex fonts not found; using the built-in font. Set "
                "LAGRANGE_VIEWER_FONTS to the directory that holds them.\n",
                stderr);
        }

        ImGui_ImplVulkan_InitInfo info{};
        info.ApiVersion = VK_API_VERSION_1_3;
        info.Instance = device.instance();
        info.PhysicalDevice = device.physicalDevice();
        info.Device = device.device();
        info.QueueFamily = device.queueFamily();
        info.Queue = device.queue();
        info.DescriptorPoolSize = 32;
        info.MinImageCount = std::max<std::uint32_t>(imageCount, 2);
        info.ImageCount = std::max<std::uint32_t>(imageCount, 2);
        info.UseDynamicRendering = true;
        targetFormat_ = targetFormat;
        info.PipelineInfoMain.PipelineRenderingCreateInfo.sType =
            VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO;
        info.PipelineInfoMain.PipelineRenderingCreateInfo.colorAttachmentCount = 1;
        info.PipelineInfoMain.PipelineRenderingCreateInfo.pColorAttachmentFormats = &targetFormat_;
        ImGui_ImplVulkan_Init(&info);

        (void)renderer_.resize({640, 360});
        registerTexture();
    }

    ~Ui() {
        vkDeviceWaitIdle(device_.device());
        textures_.clear();
        if (texture_ != VK_NULL_HANDLE) {
            ImGui_ImplVulkan_RemoveTexture(texture_);
        }
        ImGui_ImplVulkan_Shutdown();
        ImPlot::DestroyContext();
        ImGui::DestroyContext();
    }
    Ui(const Ui&) = delete;
    Ui& operator=(const Ui&) = delete;

    /// Call before the platform's NewFrame. Applies the viewport size the panel asked for.
    void prepare() {
        if (requestedWidth_ != 0 && requestedHeight_ != 0 &&
            renderer_.resize({requestedWidth_, requestedHeight_})) {
            ImGui_ImplVulkan_RemoveTexture(texture_);
            registerTexture();
        }
    }

    /// Call between ImGui::NewFrame and ImGui::Render.
    void build(float frameMs) {
        scene_ = client_.scene();
        // Re-aim the camera whenever a different stage is shown (first scene, or a file was
        // opened), but not when the same stage is merely re-sent after a seek.
        if (scene_ && scene_->statics->stagePath == framedStage_ &&
            scene_->statics->rebaseCount != framedRebase_) {
            // The floating origin moved: keep looking at the same place in the world.
            for (std::size_t k = 0; k < 3; ++k) {
                camera_.target.at(k) += framedRegion_.at(k) - scene_->statics->regionEnu.at(k);
            }
            framedRegion_ = scene_->statics->regionEnu;
            framedRebase_ = scene_->statics->rebaseCount;
        }
        if (scene_ && scene_->statics->stagePath != framedStage_) {
            frameScene(*scene_);
            framedStage_ = scene_->statics->stagePath;
            framedRegion_ = scene_->statics->regionEnu;
            framedRebase_ = scene_->statics->rebaseCount;
            selectedPrim_ = kNone;
            // A prim asked for on the command line is selected in the first stage shown.
            for (std::uint32_t i = 0; !selectPath_.empty() && i < scene_->statics->prims.size();
                 ++i) {
                if (scene_->statics->prims[i].path == selectPath_) {
                    selectedPrim_ = i;
                }
            }
            selectPath_.clear();
            if (recordRecent_ && !recentPath_.empty()) {
                RecentEntry entry;
                entry.stage = scene_->statics->stagePath;
                entry.robot =
                    scene_->statics->prims.empty() ? "" : scene_->statics->prims.front().name;
                entry.profile = "applied";
                entry.endpoint = proto::toString(client_.endpoint());
                entry.opened = localTime();
                launcher_.recent.add(std::move(entry));
                launcher_.recent.save(recentPath_);  // best effort: the list is a convenience
            }
        }
        // Cursors asked for on the command line wait for the first samples, so that they
        // are placed inside a real time window.
        if (cursorsWanted_ && !scopeSession_.traces().empty() &&
            scopeSession_.traces().front().buffer->newestTimeNs() > 0) {
            scopeSession_.showCursors(true);
            cursorsWanted_ = false;
        }
        // Edits asked for on the command line wait for the server's profile.
        if (!domainEdits_.empty() && profileSession_.profile()) {
            for (const auto& [domain, enabled] : domainEdits_) {
                if (!profileSession_.setEnabled(domain, enabled)) {
                    std::fprintf(stderr, "warning: cannot switch domain '%s'\n", domain.c_str());
                }
            }
            domainEdits_.clear();
        } else if (applyProfile_ && domainEdits_.empty() && profileSession_.canApply()) {
            profileSession_.apply(client_);
            applyProfile_ = false;
        }
        if (scene_) {
            client_.setCameraInterest(camera_.eye());  // for streamed worlds; rate-limited
        }
        PanelContext context{client_,         scene_,  client_.status(), client_.connection(),
                             client_.stats(), camera_, selectedPrim_};
        context.catalog = client_.catalog();
        context.notice = client_.notice();
        context.canOpenFiles = canOpenFiles_;
        context.initialScopeSignals = &scopeSignals_;
        context.workspace = &workspace_;
        context.focusPanel = &focusPanel_;
        context.fieldSlice = &slice_;
        if (configDirectory_.empty()) {
            configDirectory_ = configDirectory();
        }
        context.configDirectory = configDirectory_;
        context.exportNetlist = &exportNetlist_;
        context.profileSession = &profileSession_;
        context.scopeSession = &scopeSession_;
        context.renderOptions = &renderOptions_;
        context.bookmarks = &bookmarks_;
        context.launcher = &launcher_;
        context.viewportTexture =
            static_cast<std::uint64_t>(reinterpret_cast<std::uintptr_t>(texture_));
        context.sensorTextures = &textures_;
        context.viewportWidth = renderer_.extent().width;
        context.viewportHeight = renderer_.extent().height;
        context.deviceName = device_.info().name;
        context.tierName = gpu::toString(device_.tier());
        context.frameMs = frameMs;
        context.drawCalls = renderer_.lastDrawCount();
        if (launcher_.active && ImGui::GetTime() - socketChecked_ > 1.0) {
            // Is a local server listening? Looked at once a second, not every frame.
            socketChecked_ = ImGui::GetTime();
            std::error_code ec;
            launcher_.defaultAvailable =
                std::filesystem::is_socket(proto::defaultUnixSocketPath(), ec);
        }
        drawPanels(panels_, context);
        if (launcher_.connectRequested) {
            launcher_.connectRequested = false;
            const auto endpoint = proto::parseEndpoint(launcher_.connectEndpoint);
            if (!endpoint) {
                launcher_.error = "'" + launcher_.connectEndpoint +
                                  "' is not a server address (unix:/path or tcp://host:port)";
            } else {
                launcher_.error.clear();
                if (launcher_.startLocalRequested) {
                    // The client keeps retrying until the new server has bound its socket.
                    launcher_.error =
                        localServer_.start(launcher_.localServer, launcher_.connectEndpoint);
                }
                launcher_.startLocalRequested = false;
            }
            if (endpoint && launcher_.error.empty()) {
                client_.stop();
                client_.setEndpoint(*endpoint);
                client_.start();
                if (!launcher_.openTarget.empty()) {
                    client_.openFile(launcher_.openTarget);
                    launcher_.openTarget.clear();
                }
                launcher_.active = false;
                recordRecent_ = true;
            }
        }
        if (launcher_.disconnectRequested) {
            launcher_.disconnectRequested = false;
            client_.stop();
            if (recentPath_.empty()) {
                showLauncher(ImGui::GetStyle().FontScaleDpi);
            }
            launcher_.active = true;
        }
        openFileRequested_ = context.openFileRequested;
        requestedWidth_ = context.requestedViewportWidth;
        requestedHeight_ = context.requestedViewportHeight;
    }

    /// Records the scene pass. The UI pass (ImGui draw data) is recorded by the caller.
    void renderScene(VkCommandBuffer commands) {
        textures_.record(commands);
        buildSensorOverlays();
        buildAirflow();
        if (const auto environment = client_.environment()) {
            renderOptions_.windSpeed = static_cast<float>(environment->windSpeed);
        }
        overlays_.clear();
        overlays_.insert(overlays_.end(), sensorOverlays_.begin(), sensorOverlays_.end());
        overlays_.insert(overlays_.end(), airflowOverlays_.begin(), airflowOverlays_.end());
        renderer_.render(commands, scene_.get(), camera_, renderOptions_, overlays_);
    }

    /// Airflow particles (rotor_aero.md §5): kAirParticles points moved through the server's
    /// box of the air's velocity around the robot (FieldVolume of aero/wind_speed) and drawn as
    /// streaks, one batch per colour of the design's field map (viridis) by speed. New ones are
    /// placed where the air moves, so they gather in the downwash and outwash.
    void buildAirflow() {
        constexpr std::size_t kAirParticles = 1500;
        constexpr float kLifeS = 2.5F;      // s before a particle starts again
        constexpr float kTrailS = 0.06F;    // s of motion a streak shows
        constexpr float kStillMps = 0.05F;  // slower than this: placed again
        constexpr int kColours = 6;
        static const std::array<ImVec4, 5> kStops{theme::hex(0x440154), theme::hex(0x3B528B),
                                                  theme::hex(0x21918C), theme::hex(0x5EC962),
                                                  theme::hex(0xFDE725)};
        if (!renderOptions_.airflow) {
            if (airStream_) {
                (void)client_.watchField(kAirField, 0.0, SliceOrientation::VerticalXZ, false);
                airStream_.reset();
            }
            airflowOverlays_.clear();
            particles_.clear();
            renderOptions_.airflowMax = 0.0F;
            return;
        }
        if (!airStream_) {
            airStream_ = client_.watchField(kAirField, 0.0, SliceOrientation::VerticalXZ, true);
        }
        const std::shared_ptr<const FieldVolumeData> volume = airStream_->latestVolume();
        if (!volume) {
            airflowOverlays_.clear();
            return;
        }
        const float top = std::max(volume->valueMax, 0.5F);
        renderOptions_.airflowMax = top;
        std::uniform_real_distribution<double> unit(0.0, 1.0);
        const std::array<double, 3> span{volume->cellM * (volume->nx - 1),
                                         volume->cellM * (volume->ny - 1),
                                         volume->cellM * (volume->nz - 1)};
        const auto place = [&](Particle& p) {
            std::array<float, 3> v{};
            for (int attempt = 0; attempt < 12; ++attempt) {
                for (std::size_t k = 0; k < 3; ++k) {
                    p.at[k] = volume->origin[k] + unit(airRandom_) * span[k];
                }
                if (volume->sample(p.at, v) && std::hypot(v[0], v[1], v[2]) > 0.25F * top) {
                    break;
                }
            }
            p.age = static_cast<float>(unit(airRandom_)) * kLifeS;
        };
        if (particles_.size() != kAirParticles) {
            particles_.resize(kAirParticles);
            for (Particle& p : particles_) {
                place(p);
            }
        }
        const float dt = std::clamp(ImGui::GetIO().DeltaTime, 0.0F, 0.1F);
        airflowOverlays_.resize(kColours);
        for (int c = 0; c < kColours; ++c) {
            const float t = (static_cast<float>(c) + 0.5F) / kColours * 4.0F;
            const auto index = std::min(static_cast<std::size_t>(t), std::size_t{3});
            const float blend = t - static_cast<float>(index);
            const ImVec4& a = kStops.at(index);
            const ImVec4& b = kStops.at(index + 1);
            OverlayLines& lines = airflowOverlays_[static_cast<std::size_t>(c)];
            lines.origin = volume->origin;
            lines.color = {a.x + (b.x - a.x) * blend, a.y + (b.y - a.y) * blend,
                           a.z + (b.z - a.z) * blend, 1.0F};
            lines.vertices.clear();
        }
        for (Particle& p : particles_) {
            std::array<float, 3> v{};
            p.age += dt;
            if (!volume->sample(p.at, v) || p.age > kLifeS) {
                place(p);
                continue;
            }
            const float speed = std::hypot(v[0], v[1], v[2]);
            if (speed < kStillMps) {
                place(p);
                continue;
            }
            for (std::size_t k = 0; k < 3; ++k) {
                p.at[k] += v[k] * dt;
            }
            const int c = std::clamp(static_cast<int>(speed / top * kColours), 0, kColours - 1);
            std::vector<float>& out = airflowOverlays_[static_cast<std::size_t>(c)].vertices;
            const std::array<float, 3> head{static_cast<float>(p.at[0] - volume->origin[0]),
                                            static_cast<float>(p.at[1] - volume->origin[1]),
                                            static_cast<float>(p.at[2] - volume->origin[2])};
            out.insert(out.end(), {head[0] - v[0] * kTrailS, head[1] - v[1] * kTrailS,
                                   head[2] - v[2] * kTrailS, head[0], head[1], head[2]});
        }
    }

    /// Sensor frustums overlay: for each lidar of the scene, its coverage fan (spanning the
    /// returns, at kFanRadiusM) and a small cross at every return of its latest scan, in the
    /// two tones the lidar bird's-eye view uses. Points arrive in the sensor frame and are
    /// placed with the sensor's current pose.
    void buildSensorOverlays() {
        constexpr float kCrossHalfM = 0.025F;  // half the size of a return's marker
        constexpr float kFanRadiusM = 0.6F;
        constexpr int kFanSegments = 24;
        std::size_t used = 0;
        const SceneFrame* frame = scene_.get();
        if (!renderOptions_.sensors || frame == nullptr || !frame->statics) {
            sensorOverlays_.clear();
            return;
        }
        const SceneStatics& statics = *frame->statics;
        if (statics.generation != lidarGeneration_) {
            lidarGeneration_ = statics.generation;
            lidars_.clear();
            for (std::size_t i = 0; i < statics.prims.size(); ++i) {
                if (statics.prims[i].typeName == "Lidar") {
                    lidars_.push_back({static_cast<std::uint32_t>(i),
                                       client_.watchSensor(statics.prims[i].path)});
                }
            }
        }
        // Returns the index of a cleared batch: references into sensorOverlays_ are taken only
        // after every batch of a lidar exists, since adding one may move the others.
        const auto batch = [&](const std::array<double, 3>& origin, const ImVec4& color) {
            if (used == sensorOverlays_.size()) {
                sensorOverlays_.emplace_back();
            }
            OverlayLines& lines = sensorOverlays_[used];
            lines.origin = origin;
            lines.color = {color.x, color.y, color.z, color.w};
            lines.vertices.clear();  // keeps its capacity: no allocation once warmed up
            return used++;
        };
        for (const auto& [prim, stream] : lidars_) {
            const std::shared_ptr<const PointCloud> cloud = stream->latest();
            if (!cloud || cloud->count == 0 || prim >= frame->transforms.size()) {
                continue;
            }
            const Transform& pose = frame->transforms[prim];
            const std::array<float, 4>& q = pose.rotation;
            const auto rotate = [&q](float x, float y, float z) {
                // v' = v + 2w (u x v) + 2 u x (u x v), u = (q.x, q.y, q.z)
                const float tx = 2.0F * (q[2] * z - q[3] * y);
                const float ty = 2.0F * (q[3] * x - q[1] * z);
                const float tz = 2.0F * (q[1] * y - q[2] * x);
                return std::array<float, 3>{x + q[0] * tx + (q[2] * tz - q[3] * ty),
                                            y + q[0] * ty + (q[3] * tx - q[1] * tz),
                                            z + q[0] * tz + (q[1] * ty - q[2] * tx)};
            };
            const std::size_t strongBatch = batch(pose.position, kLidarStrong);
            const std::size_t weakBatch = batch(pose.position, kLidarAttenuated);
            std::vector<float>& strong = sensorOverlays_[strongBatch].vertices;
            std::vector<float>& weak = sensorOverlays_[weakBatch].vertices;
            float lowest = 1e9F;
            float highest = -1e9F;
            for (std::uint32_t i = 0; i < cloud->count; ++i) {
                const float* p = &cloud->points[static_cast<std::size_t>(i) * 4];
                const float angle = std::atan2(p[1], p[0]);
                lowest = std::min(lowest, angle);
                highest = std::max(highest, angle);
                const std::array<float, 3> at = rotate(p[0], p[1], p[2]);
                std::vector<float>& out = p[3] >= kLidarStrongIntensity ? strong : weak;
                for (int axis = 0; axis < 3; ++axis) {
                    std::array<float, 3> a = at;
                    std::array<float, 3> b = at;
                    a.at(axis) -= kCrossHalfM;
                    b.at(axis) += kCrossHalfM;
                    out.insert(out.end(), {a[0], a[1], a[2], b[0], b[1], b[2]});
                }
            }
            std::vector<float>& fan = weak;
            const auto edge = [&](float angle) {
                const std::array<float, 3> end =
                    rotate(kFanRadiusM * std::cos(angle), kFanRadiusM * std::sin(angle), 0.0F);
                fan.insert(fan.end(), {0.0F, 0.0F, 0.0F, end[0], end[1], end[2]});
            };
            edge(lowest);
            edge(highest);
            for (int k = 0; k < kFanSegments; ++k) {
                const float a0 = lowest + (highest - lowest) * static_cast<float>(k) / kFanSegments;
                const float a1 =
                    lowest + (highest - lowest) * static_cast<float>(k + 1) / kFanSegments;
                const std::array<float, 3> p0 =
                    rotate(kFanRadiusM * std::cos(a0), kFanRadiusM * std::sin(a0), 0.0F);
                const std::array<float, 3> p1 =
                    rotate(kFanRadiusM * std::cos(a1), kFanRadiusM * std::sin(a1), 0.0F);
                fan.insert(fan.end(), {p0[0], p0[1], p0[2], p1[0], p1[1], p1[2]});
            }
        }
        sensorOverlays_.resize(used);
    }

    [[nodiscard]] const std::shared_ptr<const SceneFrame>& scene() const { return scene_; }
    [[nodiscard]] std::uint32_t drawCalls() const { return renderer_.lastDrawCount(); }
    void setCanOpenFiles(bool value) { canOpenFiles_ = value; }
    void setWorkspace(Workspace workspace) { workspace_ = workspace; }

    /// Shows the launcher first: the client is started when the user chooses a server.
    void showLauncher(float displayScale) {
        launcher_.active = true;
        launcher_.system = systemInfo(device_, displayScale);
        launcher_.defaultEndpoint =
            proto::toString(proto::Endpoint::unixSocket(proto::defaultUnixSocketPath()));
        launcher_.localServer = findLocalServer();
        launcher_.server = launcher_.localServer.empty() ? 1 : 0;
        recentPath_ = launcher_.system.configDirectory + "/recent.json";
        launcher_.recent.load(recentPath_);
    }
    [[nodiscard]] bool launcherActive() const { return launcher_.active; }
    /// As if the user chose Start local simulator and pressed Connect (--start-local).
    void requestStartLocal(const std::string& endpoint) {
        launcher_.server = 0;
        launcher_.connectRequested = true;
        launcher_.startLocalRequested = true;
        launcher_.connectEndpoint = endpoint;
        if (launcher_.localServer.empty()) {
            launcher_.error = "lagrange_server is not installed";
        }
    }
    /// A file chosen or dropped while the launcher is shown: opened after connecting.
    void setOpenTarget(const std::string& path) {
        std::error_code ec;
        const std::filesystem::path absolute = std::filesystem::absolute(path, ec);
        launcher_.openTarget = ec ? path : absolute.lexically_normal().string();
    }
    void setInitialSelection(std::string path) { selectPath_ = std::move(path); }
    void setFocusPanel(std::string title) { focusPanel_ = std::move(title); }
    void setExportNetlist(std::string path) { exportNetlist_ = std::move(path); }
    void setCursors(bool on) { cursorsWanted_ = on; }
    [[nodiscard]] Shading shading() const { return renderOptions_.shading; }
    /// Start with the follow camera at this distance (m) from the robot; 0: frame the scene.
    /// Where the camera looks from at the start: target (m), distance (m), yaw and pitch (deg).
    void setCamera(const std::optional<std::array<double, 6>>& view) { startView_ = view; }
    void setFollow(double distance, double pitchDeg) {
        followDistance_ = distance;
        followPitchDeg_ = pitchDeg;
    }
    /// Overlays to show from the start, whatever the shading turned off ("grid,axes,sensors").
    void setOverlays(const std::string& names) {
        renderOptions_.grid = renderOptions_.grid || names.find("grid") != std::string::npos;
        renderOptions_.axes = renderOptions_.axes || names.find("axes") != std::string::npos;
        renderOptions_.sensors =
            renderOptions_.sensors || names.find("sensors") != std::string::npos;
        renderOptions_.airflow =
            renderOptions_.airflow || names.find("airflow") != std::string::npos;
    }
    /// The field and plane the Slice 2D panel starts on ("aero/wind_speed:xz").
    void setSlice(std::string slice) { slice_ = std::move(slice); }
    void setSeason(const std::string& name) {
        if (name == "fall") {
            renderOptions_.season = Season::Fall;
        } else if (name == "winter") {
            renderOptions_.season = Season::Winter;
        } else if (name == "rainy") {
            renderOptions_.season = Season::Rainy;
        }
    }
    void setShading(const std::string& name) {
        renderOptions_.realisticAvailable = device_.tier() != gpu::CapabilityTier::Tier0;
        // A GPU shows the world as it would look, a software device keeps the quick view.
        const bool realistic =
            name == "realistic" || (name.empty() && renderOptions_.realisticAvailable);
        if (realistic) {
            renderOptions_.shading = Shading::Realistic;
            renderOptions_.grid = false;
            renderOptions_.axes = false;
            renderOptions_.sensors = false;
        } else if (name == "lit") {
            renderOptions_.shading = Shading::Lit;
        } else if (name == "unlit") {
            renderOptions_.shading = Shading::Unlit;
        } else if (name == "collision") {
            renderOptions_.shading = Shading::Collision;
        } else if (name == "wireframe") {
            renderOptions_.shading = Shading::Wireframe;
        } else if (name == "thermal") {
            renderOptions_.shading = Shading::Thermal;  // falls back to Lit without thermal nodes
        }
    }
    void setProfileEdits(std::vector<std::pair<std::string, bool>> edits, bool apply) {
        domainEdits_ = std::move(edits);
        applyProfile_ = apply;
    }
    [[nodiscard]] const ProfileSession& profileSession() const { return profileSession_; }
    /// True once after the user chose "Open..." in the menu.
    [[nodiscard]] bool takeOpenFileRequest() { return std::exchange(openFileRequested_, false); }

private:
    void registerTexture() {
        texture_ = ImGui_ImplVulkan_AddTexture(renderer_.colorView(),
                                               VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    }

    // Aim the camera at the scene: the bounds of everything with geometry, leaving out
    // very large things (floors) so that they do not push the robot into the distance.
    void frameScene(const SceneFrame& frame) {
        const SceneStatics& statics = *frame.statics;
        std::array<double, 3> low{1e300, 1e300, 1e300};
        std::array<double, 3> high{-1e300, -1e300, -1e300};
        bool any = false;
        for (std::size_t i = 0; i < statics.prims.size(); ++i) {
            const PrimData& prim = statics.prims[i];
            if (prim.mesh == kNone) {
                continue;
            }
            const std::array<float, 6>& box = statics.meshes[prim.mesh].aabb;
            const Transform& transform = frame.transforms[i];
            double radius = 0.0;  // of a sphere around the prim's origin that holds the mesh
            for (std::size_t k = 0; k < 3; ++k) {
                const double extent = std::max(std::abs(static_cast<double>(box.at(k))),
                                               std::abs(static_cast<double>(box.at(k + 3)))) *
                                      static_cast<double>(std::abs(transform.scale.at(k)));
                radius = std::max(radius, extent);
            }
            radius *= 1.7320508;  // corner of the box, not just its face
            if (radius > 5.0) {
                continue;
            }
            for (std::size_t k = 0; k < 3; ++k) {
                low.at(k) = std::min(low.at(k), transform.position.at(k) - radius);
                high.at(k) = std::max(high.at(k), transform.position.at(k) + radius);
            }
            any = true;
        }
        if (!any) {
            return;
        }
        double size = 0.0;
        for (std::size_t k = 0; k < 3; ++k) {
            camera_.target.at(k) = (low.at(k) + high.at(k)) / 2.0;
            size = std::max(size, high.at(k) - low.at(k));
        }
        camera_.distance = std::clamp(size * 1.6, 0.3, 500.0);
        if (followDistance_ > 0.0) {
            camera_.distance = followDistance_;
            if (followPitchDeg_ != 0.0) {
                camera_.pitch = followPitchDeg_ * std::numbers::pi / 180.0;
            }
            renderOptions_.follow = true;
        }
        if (startView_) {
            camera_.target = {(*startView_)[0], (*startView_)[1], (*startView_)[2]};
            camera_.distance = (*startView_)[3];
            camera_.yaw = (*startView_)[4] * std::numbers::pi / 180.0;
            camera_.pitch = (*startView_)[5] * std::numbers::pi / 180.0;
        }
    }

    std::optional<std::array<double, 6>> startView_;
    const gpu::Device& device_;
    ViewerClient& client_;
    SceneRenderer renderer_;
    std::vector<std::unique_ptr<Panel>> panels_;
    Camera camera_;
    double followDistance_{0.0};
    std::array<double, 3> framedRegion_{};  // the region origin the camera is placed in
    std::uint64_t framedRebase_{0};
    double followPitchDeg_{0.0};
    std::uint32_t selectedPrim_{kNone};
    std::shared_ptr<const SceneFrame> scene_;
    VkDescriptorSet texture_{VK_NULL_HANDLE};
    TextureCache textures_{device_};
    VkFormat targetFormat_{VK_FORMAT_UNDEFINED};
    std::uint32_t requestedWidth_{0};
    std::uint32_t requestedHeight_{0};
    std::string iniPath_;
    std::vector<std::string> scopeSignals_;
    bool canOpenFiles_{false};
    Workspace workspace_{Workspace::Scene};
    ProfileSession profileSession_;
    ScopeSession scopeSession_;
    RenderOptions renderOptions_;
    std::uint64_t lidarGeneration_{0};  // statics generation lidars_ was built from
    std::vector<std::pair<std::uint32_t, std::shared_ptr<SensorStream>>> lidars_;  // prim, frames
    std::vector<OverlayLines> sensorOverlays_;
    std::vector<OverlayLines> airflowOverlays_;
    std::vector<OverlayLines> overlays_;  // both, as drawn
    static constexpr const char* kAirField = "aero/wind_speed";
    struct Particle {
        std::array<double, 3> at{};  // region frame, m
        float age{0.0F};             // s
    };
    std::shared_ptr<FieldStream> airStream_;
    std::vector<Particle> particles_;
    std::mt19937 airRandom_{20261102U};
    std::vector<std::int64_t> bookmarks_;
    LauncherState launcher_;
    LocalServer localServer_;
    bool recordRecent_{false};
    std::string recentPath_;
    double socketChecked_{-10.0};
    std::vector<std::pair<std::string, bool>> domainEdits_;
    bool applyProfile_{false};
    std::string selectPath_;
    std::string focusPanel_;
    std::string slice_;
    std::string configDirectory_;
    std::string exportNetlist_;
    bool cursorsWanted_{false};
    bool openFileRequested_{false};
    std::string framedStage_{"\x01"};  // never a real stage path, so the first scene is framed
};

void beginUiPass(VkCommandBuffer commands, VkImageView view, VkExtent2D extent) {
    VkRenderingAttachmentInfo color{VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO};
    color.imageView = view;
    color.imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
    color.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    color.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
    color.clearValue.color = {{0.06F, 0.06F, 0.07F, 1.0F}};
    VkRenderingInfo rendering{VK_STRUCTURE_TYPE_RENDERING_INFO};
    rendering.renderArea = {{0, 0}, extent};
    rendering.layerCount = 1;
    rendering.colorAttachmentCount = 1;
    rendering.pColorAttachments = &color;
    vkCmdBeginRendering(commands, &rendering);
}

// The server reads the file itself, so it needs a path that means the same to it.
void openFile(ViewerClient& client, const std::string& path) {
    std::error_code ec;
    const std::filesystem::path absolute = std::filesystem::absolute(path, ec);
    client.openFile(ec ? path : absolute.lexically_normal().string());
}

proto::Endpoint resolveEndpoint(const Options& options, bool& ok) {
    ok = true;
    if (options.endpoint.empty()) {
        return proto::Endpoint::unixSocket(proto::defaultUnixSocketPath());
    }
    const auto parsed = proto::parseEndpoint(options.endpoint);
    if (!parsed) {
        std::fprintf(stderr, "error: '%s' is not an endpoint (unix:/path or tcp://host:port)\n",
                     options.endpoint.c_str());
        ok = false;
        return {};
    }
    return *parsed;
}

std::string jsonEscape(const std::string& text) {
    std::string out;
    for (const char c : text) {
        if (c == '"' || c == '\\') {
            out += '\\';
            out += c;
        } else if (static_cast<unsigned char>(c) < 0x20) {
            out += ' ';
        } else {
            out += c;
        }
    }
    return out;
}

// --- headless ---------------------------------------------------------------------------

/// --input: mouse and keyboard at given frames, so that a headless run can press what a user
/// presses. "FRAME:click:X,Y" (down and up on the next frames), "FRAME:move:X,Y", "FRAME:down",
/// "FRAME:up", "FRAME:text:WORDS", "FRAME:wheel:N", ";" between them; pixels of the window.
struct InputEvent {
    std::uint32_t frame{0};
    std::string kind;
    float x{0.0F};
    float y{0.0F};
    std::string text;
};

std::vector<InputEvent> parseInput(const std::string& script) {
    std::vector<InputEvent> events;
    std::size_t from = 0;
    while (from < script.size()) {
        std::size_t end = script.find(';', from);
        end = end == std::string::npos ? script.size() : end;
        const std::string item = script.substr(from, end - from);
        from = end + 1;
        const std::size_t a = item.find(':');
        if (a == std::string::npos) {
            continue;
        }
        InputEvent event;
        event.frame = static_cast<std::uint32_t>(std::strtoul(item.c_str(), nullptr, 10));
        const std::size_t b = item.find(':', a + 1);
        event.kind = item.substr(a + 1, b == std::string::npos ? b : b - a - 1);
        const std::string rest = b == std::string::npos ? std::string{} : item.substr(b + 1);
        event.text = rest;
        std::sscanf(rest.c_str(), "%f,%f", &event.x, &event.y);
        if (event.kind == "click") {
            InputEvent move = event;
            move.kind = "move";
            events.push_back(move);
            InputEvent down = event;
            down.kind = "down";
            down.frame = event.frame + 1;
            events.push_back(down);
            InputEvent up = event;
            up.kind = "up";
            up.frame = event.frame + 2;
            events.push_back(up);
        } else if (event.kind == "key") {
            // A key pressed on this frame and let go on the next: enter, tab or escape.
            InputEvent down = event;
            down.kind = "keydown";
            events.push_back(down);
            InputEvent up = event;
            up.kind = "keyup";
            up.frame = event.frame + 1;
            events.push_back(up);
        } else {
            events.push_back(std::move(event));
        }
    }
    return events;
}

void applyInput(const std::vector<InputEvent>& events, std::uint32_t frame, ImGuiIO& io) {
    for (const InputEvent& event : events) {
        if (event.frame != frame) {
            continue;
        }
        if (event.kind == "move") {
            io.AddMousePosEvent(event.x, event.y);
        } else if (event.kind == "down") {
            io.AddMouseButtonEvent(ImGuiMouseButton_Left, true);
        } else if (event.kind == "up") {
            io.AddMouseButtonEvent(ImGuiMouseButton_Left, false);
        } else if (event.kind == "wheel") {
            io.AddMouseWheelEvent(0.0F, event.x);
        } else if (event.kind == "text") {
            io.AddInputCharactersUTF8(event.text.c_str());
        } else if (event.kind == "keydown" || event.kind == "keyup") {
            const ImGuiKey key = event.text == "enter"    ? ImGuiKey_Enter
                                 : event.text == "tab"    ? ImGuiKey_Tab
                                 : event.text == "escape" ? ImGuiKey_Escape
                                                          : ImGuiKey_None;
            if (key != ImGuiKey_None) {
                io.AddKeyEvent(key, event.kind == "keydown");
            }
        }
    }
}

int runHeadless(const Options& options, const proto::Endpoint& endpoint, Workspace workspace) {
    gpu::DeviceDesc desc;
    desc.applicationName = "lagrange_viewer";
    desc.validation = options.validation;
    desc.preferredDevice = options.gpu;
    std::string error;
    const std::unique_ptr<gpu::Device> device = gpu::Device::create(desc, error);
    if (!device) {
        std::fprintf(stderr, "error: %s\n", error.c_str());
        return 3;
    }

    ViewerClient client(endpoint);
    client.setSecurity(options.tlsCa, options.authToken);
    if (!options.launcher && !options.startLocal) {
        client.start();
        if (!options.openPath.empty()) {
            openFile(client, options.openPath);
        }
    }

    const VkExtent2D extent{options.width, options.height};
    const VkFormat format = VK_FORMAT_R8G8B8A8_UNORM;
    gpu::Image target =
        gpu::createImage2D(*device, extent, format,
                           VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT,
                           VK_IMAGE_ASPECT_COLOR_BIT);
    int exitCode = 0;
    std::uint32_t rendered = 0;
    {
        Ui ui(*device, client, format, 2, 1.0F, false, options.scopeSignals);
        ui.setWorkspace(workspace);
        if (options.launcher || options.startLocal) {
            ui.showLauncher(1.0F);
        }
        if (options.startLocal) {
            ui.requestStartLocal(proto::toString(endpoint));
        }
        ui.setInitialSelection(options.selectPath);
        ui.setFocusPanel(options.focusPanel);
        ui.setSlice(options.slice);
        ui.setExportNetlist(options.exportNetlist);
        ui.setCursors(options.cursors);
        ui.setShading(options.shading);
        ui.setSeason(options.season);
        ui.setOverlays(options.overlays);
        ui.setFollow(options.followDistance, options.followPitchDeg);
        ui.setCamera(options.camera);
        ui.setProfileEdits(options.domainEdits, options.applyProfile);
        ImGuiIO& io = ImGui::GetIO();
        io.DisplaySize =
            ImVec2(static_cast<float>(extent.width), static_cast<float>(extent.height));

        const auto deadline = Clock::now() + std::chrono::milliseconds(options.waitMs);
        bool first = true;
        // Frames are counted from the moment the scene is there; before that we keep
        // drawing (the "not connected" state) until the deadline.
        const std::uint32_t wanted = options.frames != 0 ? options.frames : 120;
        // --record-frames writes a frame at a fixed wall-clock rate once the scene is there;
        // --frames then counts the frames written, so the clip's length is frames / fps.
        const bool recording = !options.recordFrames.empty();
        if (recording) {
            std::error_code ignored;
            std::filesystem::create_directories(options.recordFrames, ignored);
        }
        const auto framePeriod = std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double>(1.0 / std::max(options.recordFps, 1U)));
        const std::vector<InputEvent> input = parseInput(options.input);
        auto nextFrame = Clock::time_point{};
        std::uint32_t written = 0;
        while ((recording ? written : rendered) < wanted) {
            // With --open, wait for the requested file's scene, not the one served before it.
            const auto current = client.scene();
            // The launcher needs no scene: it is what is shown before connecting.
            const bool haveScene =
                options.launcher ||
                (current != nullptr &&
                 (options.openPath.empty() || client.notice().text.rfind("Loading", 0) != 0));
            if (!haveScene && Clock::now() > deadline) {
                std::fprintf(stderr, "error: no scene from %s within %u ms (%s)\n",
                             proto::toString(endpoint).c_str(), options.waitMs,
                             client.connection().message.c_str());
                exitCode = 4;
                break;
            }
            ui.prepare();
            applyInput(input, rendered, io);
            io.DeltaTime = 1.0F / 60.0F;
            ImGui_ImplVulkan_NewFrame();
            ImGui::NewFrame();
            ui.build(0.0F);
            ImGui::Render();
            device->submitAndWait([&](VkCommandBuffer commands) {
                ui.renderScene(commands);
                gpu::transitionImage(
                    commands, target.image, VK_IMAGE_ASPECT_COLOR_BIT,
                    first ? VK_IMAGE_LAYOUT_UNDEFINED : VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                    VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL);
                beginUiPass(commands, target.view, extent);
                ImGui_ImplVulkan_RenderDrawData(ImGui::GetDrawData(), commands);
                vkCmdEndRendering(commands);
            });
            first = false;
            if (haveScene) {
                ++rendered;
                if (recording && Clock::now() >= nextFrame) {
                    nextFrame =
                        (nextFrame == Clock::time_point{} ? Clock::now() : nextFrame) + framePeriod;
                    char name[32];
                    std::snprintf(name, sizeof(name), "frame_%05u.png", written);
                    const std::string file =
                        (std::filesystem::path(options.recordFrames) / name).string();
                    const std::vector<std::uint8_t> pixels = gpu::readbackRgba8(
                        *device, target, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL);
                    if (!gpu::writePng(file, extent.width, extent.height, pixels)) {
                        std::fprintf(stderr, "error: cannot write %s\n", file.c_str());
                        exitCode = 5;
                        break;
                    }
                    ++written;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(8));
        }

        if (exitCode == 0 && !options.screenshot.empty()) {
            const std::vector<std::uint8_t> pixels =
                gpu::readbackRgba8(*device, target, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL);
            if (!gpu::writePng(options.screenshot, extent.width, extent.height, pixels)) {
                std::fprintf(stderr, "error: cannot write %s\n", options.screenshot.c_str());
                exitCode = 5;
            }
        }

        // What the server runs (applied), and what it says about the profile being edited.
        std::string enabledDomains;
        if (const auto profile = client.profile()) {
            for (const DomainInfo& domain : profile->domains) {
                if (domain.enabled) {
                    enabledDomains += enabledDomains.empty() ? "" : ",";
                    enabledDomains += domain.name;
                }
            }
        }
        const char* profileVerdict = "none";
        switch (ui.profileSession().validity()) {
            case ProfileSession::Validity::Valid:
                profileVerdict = "valid";
                break;
            case ProfileSession::Validity::Invalid:
                profileVerdict = "invalid";
                break;
            case ProfileSession::Validity::Checking:
                profileVerdict = "checking";
                break;
            case ProfileSession::Validity::None:
                break;
        }
        const ClientStats stats = client.stats();
        const auto scene = client.scene();
        std::printf(
            "{\"notice\": \"%s\", \"notice_is_error\": %s, \"frames\": %u, \"snapshots\": %llu, "
            "\"deltas\": %llu, \"resyncs\": %llu, "
            "\"prims\": %zu, \"meshes\": %zu, \"draw_calls\": %u, \"sim_time_ns\": %lld, "
            "\"signal_blocks\": %llu, \"samples\": %llu, \"invalid_runs\": %llu, "
            "\"dropped_samples\": %llu, "
            "\"device\": \"%s\", \"tier\": %d, \"validation_errors\": %u, \"connected\": %s, "
            "\"profile\": \"%s\", \"profile_changes\": %zu, \"domains_enabled\": \"%s\", "
            "\"log_lines\": %llu, \"attributes\": %zu, \"wiring_nodes\": %zu, "
            "\"wiring_wires\": %zu, \"events\": %zu, \"server_stats\": %s, "
            "\"environment_fields\": %zu, \"ros_topics\": %zu, \"ros_nodes\": %zu, "
            "\"world_tiles\": %zu, \"world_agents\": %zu, \"shading\": %d}\n",
            jsonEscape(client.notice().text).c_str(), client.notice().error ? "true" : "false",
            rendered, static_cast<unsigned long long>(stats.snapshots),
            static_cast<unsigned long long>(stats.deltas),
            static_cast<unsigned long long>(stats.resyncs),
            scene ? scene->statics->prims.size() : std::size_t{0},
            scene ? scene->statics->meshes.size() : std::size_t{0}, ui.drawCalls(),
            static_cast<long long>(scene ? scene->simTimeNs : 0),
            static_cast<unsigned long long>(stats.signalBlocks),
            static_cast<unsigned long long>(stats.samples),
            static_cast<unsigned long long>(stats.invalidRuns),
            static_cast<unsigned long long>(stats.droppedSamples), device->info().name.c_str(),
            static_cast<int>(device->tier()), device->validationErrorCount(),
            client.connection().state == ConnectionState::Ready ? "true" : "false", profileVerdict,
            ui.profileSession().draft().changedLineCount(), enabledDomains.c_str(),
            static_cast<unsigned long long>(client.logs()->total),
            client.attributes() ? client.attributes()->attributes.size() : std::size_t{0},
            ui.profileSession().wiring().nodes.size(), ui.profileSession().wiring().wires.size(),
            client.events()->events.size(), client.statistics() ? "true" : "false",
            client.environment() ? client.environment()->fields.size() : std::size_t{0},
            client.rosGraph() ? client.rosGraph()->topics.size() : std::size_t{0},
            client.rosGraph() ? client.rosGraph()->nodes.size() : std::size_t{0},
            client.world() ? client.world()->tiles.size() : std::size_t{0},
            client.world() ? client.world()->agents.size() : std::size_t{0},
            static_cast<int>(ui.shading()));
        if (device->validationErrorCount() != 0 && exitCode == 0) {
            exitCode = 6;
        }
    }
    client.stop();
    gpu::destroy(*device, target);
    return exitCode;
}

// --- windowed ---------------------------------------------------------------------------

struct Swapchain {
    VkSwapchainKHR handle{VK_NULL_HANDLE};
    VkFormat format{VK_FORMAT_UNDEFINED};
    VkExtent2D extent{};
    std::vector<VkImage> images;
    std::vector<VkImageView> views;
};

void destroySwapchain(const gpu::Device& device, Swapchain& swapchain) {
    for (VkImageView view : swapchain.views) {
        vkDestroyImageView(device.device(), view, nullptr);
    }
    if (swapchain.handle != VK_NULL_HANDLE) {
        vkDestroySwapchainKHR(device.device(), swapchain.handle, nullptr);
    }
    swapchain = {};
}

bool createSwapchain(const gpu::Device& device, SDL_Window* window, Swapchain& swapchain) {
    VkSurfaceCapabilitiesKHR caps{};
    vkGetPhysicalDeviceSurfaceCapabilitiesKHR(device.physicalDevice(), device.surface(), &caps);

    VkExtent2D extent = caps.currentExtent;
    if (extent.width == UINT32_MAX) {  // Wayland: the surface takes the size we choose
        int width = 0;
        int height = 0;
        SDL_GetWindowSizeInPixels(window, &width, &height);
        extent.width = std::clamp(static_cast<std::uint32_t>(std::max(width, 1)),
                                  caps.minImageExtent.width, caps.maxImageExtent.width);
        extent.height = std::clamp(static_cast<std::uint32_t>(std::max(height, 1)),
                                   caps.minImageExtent.height, caps.maxImageExtent.height);
    }
    if (extent.width == 0 || extent.height == 0) {
        return false;  // minimised
    }

    std::uint32_t formatCount = 0;
    vkGetPhysicalDeviceSurfaceFormatsKHR(device.physicalDevice(), device.surface(), &formatCount,
                                         nullptr);
    std::vector<VkSurfaceFormatKHR> formats(formatCount);
    vkGetPhysicalDeviceSurfaceFormatsKHR(device.physicalDevice(), device.surface(), &formatCount,
                                         formats.data());
    if (formats.empty()) {
        return false;
    }
    VkSurfaceFormatKHR chosen = formats.front();
    for (const VkSurfaceFormatKHR& candidate : formats) {
        if ((candidate.format == VK_FORMAT_B8G8R8A8_UNORM ||
             candidate.format == VK_FORMAT_R8G8B8A8_UNORM) &&
            candidate.colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR) {
            chosen = candidate;
            break;
        }
    }

    VkSwapchainCreateInfoKHR info{VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR};
    info.surface = device.surface();
    info.minImageCount = std::max<std::uint32_t>(caps.minImageCount + 1, 2);
    if (caps.maxImageCount != 0) {
        info.minImageCount = std::min(info.minImageCount, caps.maxImageCount);
    }
    info.imageFormat = chosen.format;
    info.imageColorSpace = chosen.colorSpace;
    info.imageExtent = extent;
    info.imageArrayLayers = 1;
    info.imageUsage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
    info.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;
    info.preTransform = caps.currentTransform;
    info.compositeAlpha = (caps.supportedCompositeAlpha & VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR) != 0
                              ? VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR
                              : VK_COMPOSITE_ALPHA_INHERIT_BIT_KHR;
    info.presentMode = VK_PRESENT_MODE_FIFO_KHR;  // vsync; always available
    info.clipped = VK_TRUE;
    info.oldSwapchain = swapchain.handle;

    VkSwapchainKHR created = VK_NULL_HANDLE;
    if (vkCreateSwapchainKHR(device.device(), &info, nullptr, &created) != VK_SUCCESS) {
        return false;
    }
    destroySwapchain(device, swapchain);
    swapchain.handle = created;
    swapchain.format = chosen.format;
    swapchain.extent = extent;

    std::uint32_t imageCount = 0;
    vkGetSwapchainImagesKHR(device.device(), created, &imageCount, nullptr);
    swapchain.images.resize(imageCount);
    vkGetSwapchainImagesKHR(device.device(), created, &imageCount, swapchain.images.data());
    for (VkImage image : swapchain.images) {
        VkImageViewCreateInfo viewInfo{VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO};
        viewInfo.image = image;
        viewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
        viewInfo.format = chosen.format;
        viewInfo.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
        VkImageView view = VK_NULL_HANDLE;
        vkCreateImageView(device.device(), &viewInfo, nullptr, &view);
        swapchain.views.push_back(view);
    }
    return true;
}

int runWindowed(const Options& options, const proto::Endpoint& endpoint, Workspace workspace) {
    if (!SDL_Init(SDL_INIT_VIDEO)) {
        std::fprintf(stderr, "error: cannot initialise the window system: %s\n", SDL_GetError());
        return 3;
    }
    SDL_Window* window = SDL_CreateWindow(
        "Lagrange", static_cast<int>(options.width), static_cast<int>(options.height),
        SDL_WINDOW_VULKAN | SDL_WINDOW_RESIZABLE | SDL_WINDOW_HIGH_PIXEL_DENSITY);
    if (window == nullptr) {
        std::fprintf(stderr, "error: cannot create a window: %s\n", SDL_GetError());
        SDL_Quit();
        return 3;
    }

    gpu::DeviceDesc desc;
    desc.applicationName = "lagrange_viewer";
    desc.validation = options.validation;
    desc.preferredDevice = options.gpu;
    Uint32 extensionCount = 0;
    const char* const* extensions = SDL_Vulkan_GetInstanceExtensions(&extensionCount);
    for (Uint32 i = 0; i < extensionCount; ++i) {
        desc.instanceExtensions.emplace_back(extensions[i]);
    }
    desc.createSurface = [window](VkInstance instance) {
        VkSurfaceKHR surface = VK_NULL_HANDLE;
        if (!SDL_Vulkan_CreateSurface(window, instance, nullptr, &surface)) {
            return VkSurfaceKHR{VK_NULL_HANDLE};
        }
        return surface;
    };
    std::string error;
    std::unique_ptr<gpu::Device> device = gpu::Device::create(desc, error);
    if (!device) {
        std::fprintf(stderr, "error: %s\n", error.c_str());
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 3;
    }
    std::printf("device: %s, %s\n", device->info().name.c_str(), gpu::toString(device->tier()));

    Swapchain swapchain;
    if (!createSwapchain(*device, window, swapchain)) {
        std::fputs("error: cannot create a swapchain\n", stderr);
        device.reset();
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 3;
    }

    // Without a server named on the command line the viewer starts on the launcher, and the
    // client is started when the user picks a server there (design: 01-launcher).
    const bool launcher = options.launcher || options.startLocal ||
                          (options.endpoint.empty() && options.openPath.empty());
    ViewerClient client(endpoint);
    client.setSecurity(options.tlsCa, options.authToken);
    if (!launcher) {
        client.start();
        if (!options.openPath.empty()) {
            openFile(client, options.openPath);
        }
    }

    VkCommandBufferAllocateInfo allocateInfo{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
    allocateInfo.commandPool = device->commandPool();
    allocateInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    allocateInfo.commandBufferCount = 1;
    VkCommandBuffer commands = VK_NULL_HANDLE;
    vkAllocateCommandBuffers(device->device(), &allocateInfo, &commands);
    VkSemaphoreCreateInfo semaphoreInfo{VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO};
    VkSemaphore acquired = VK_NULL_HANDLE;
    VkSemaphore renderedSemaphore = VK_NULL_HANDLE;
    vkCreateSemaphore(device->device(), &semaphoreInfo, nullptr, &acquired);
    vkCreateSemaphore(device->device(), &semaphoreInfo, nullptr, &renderedSemaphore);
    VkFenceCreateInfo fenceInfo{VK_STRUCTURE_TYPE_FENCE_CREATE_INFO};
    fenceInfo.flags = VK_FENCE_CREATE_SIGNALED_BIT;
    VkFence frameDone = VK_NULL_HANDLE;
    vkCreateFence(device->device(), &fenceInfo, nullptr, &frameDone);

    {
        const float dpiScale = std::max(SDL_GetWindowDisplayScale(window), 1.0F);
        Ui ui(*device, client, swapchain.format,
              static_cast<std::uint32_t>(swapchain.images.size()), dpiScale, true,
              options.scopeSignals);
        ImGui_ImplSDL3_InitForVulkan(window);
        ui.setCanOpenFiles(true);
        if (launcher) {
            ui.showLauncher(dpiScale);
        }
        if (options.startLocal) {
            ui.requestStartLocal(proto::toString(endpoint));
        }
        ui.setWorkspace(workspace);
        ui.setInitialSelection(options.selectPath);
        ui.setFocusPanel(options.focusPanel);
        ui.setSlice(options.slice);
        ui.setExportNetlist(options.exportNetlist);
        ui.setCursors(options.cursors);
        ui.setShading(options.shading);
        ui.setOverlays(options.overlays);
        ui.setFollow(options.followDistance, options.followPitchDeg);
        ui.setCamera(options.camera);
        ui.setProfileEdits(options.domainEdits, options.applyProfile);
        // The file dialog reports its result on a callback that may run on another thread.
        struct Chosen {
            std::mutex mutex;
            std::vector<std::string> paths;
        } chosen;

        bool running = true;
        bool rebuild = false;
        std::uint32_t presentedFrames = 0;
        float frameMs = 0.0F;
        auto last = Clock::now();
        while (running) {
            SDL_Event event;
            while (SDL_PollEvent(&event)) {
                ImGui_ImplSDL3_ProcessEvent(&event);
                if (event.type == SDL_EVENT_QUIT ||
                    (event.type == SDL_EVENT_WINDOW_CLOSE_REQUESTED &&
                     event.window.windowID == SDL_GetWindowID(window))) {
                    running = false;
                }
                if (event.type == SDL_EVENT_WINDOW_PIXEL_SIZE_CHANGED) {
                    rebuild = true;
                }
                if (event.type == SDL_EVENT_DROP_FILE && event.drop.data != nullptr) {
                    // A file dropped on the window: opened now, or after connecting.
                    if (ui.launcherActive()) {
                        ui.setOpenTarget(event.drop.data);
                    } else {
                        openFile(client, event.drop.data);
                    }
                }
            }
            if ((SDL_GetWindowFlags(window) & SDL_WINDOW_MINIMIZED) != 0) {
                SDL_Delay(100);  // the client thread keeps receiving
                continue;
            }

            vkWaitForFences(device->device(), 1, &frameDone, VK_TRUE, UINT64_MAX);
            if (rebuild) {
                vkDeviceWaitIdle(device->device());
                if (!createSwapchain(*device, window, swapchain)) {
                    SDL_Delay(50);
                    continue;
                }
                rebuild = false;
            }

            ui.prepare();
            ImGui_ImplVulkan_NewFrame();
            ImGui_ImplSDL3_NewFrame();
            ImGui::NewFrame();
            ui.build(frameMs);
            ImGui::Render();
            if (ui.takeOpenFileRequest()) {
                static const SDL_DialogFileFilter kFilters[] = {
                    {"Robots and recordings", "urdf;sdf;xml;usd;usda;usdc;mcap"},
                    {"All files", "*"}};
                SDL_ShowOpenFileDialog(
                    [](void* user, const char* const* files, int /*filter*/) {
                        auto* result = static_cast<Chosen*>(user);
                        if (files != nullptr && files[0] != nullptr) {
                            const std::lock_guard lock(result->mutex);
                            result->paths.emplace_back(files[0]);
                        }
                    },
                    &chosen, window, kFilters, 2, nullptr, false);
            }
            {
                const std::lock_guard lock(chosen.mutex);
                for (const std::string& path : chosen.paths) {
                    if (ui.launcherActive()) {
                        ui.setOpenTarget(path);
                    } else {
                        openFile(client, path);
                    }
                }
                chosen.paths.clear();
            }

            std::uint32_t imageIndex = 0;
            const VkResult acquire =
                vkAcquireNextImageKHR(device->device(), swapchain.handle, UINT64_MAX, acquired,
                                      VK_NULL_HANDLE, &imageIndex);
            if (acquire == VK_ERROR_OUT_OF_DATE_KHR) {
                rebuild = true;
                continue;
            }
            if (acquire != VK_SUCCESS && acquire != VK_SUBOPTIMAL_KHR) {
                std::fputs("error: cannot acquire a swapchain image\n", stderr);
                break;
            }
            vkResetFences(device->device(), 1, &frameDone);

            vkResetCommandBuffer(commands, 0);
            VkCommandBufferBeginInfo beginInfo{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
            beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
            vkBeginCommandBuffer(commands, &beginInfo);
            ui.renderScene(commands);
            gpu::transitionImage(commands, swapchain.images[imageIndex], VK_IMAGE_ASPECT_COLOR_BIT,
                                 VK_IMAGE_LAYOUT_UNDEFINED,
                                 VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL);
            beginUiPass(commands, swapchain.views[imageIndex], swapchain.extent);
            ImGui_ImplVulkan_RenderDrawData(ImGui::GetDrawData(), commands);
            vkCmdEndRendering(commands);
            gpu::transitionImage(commands, swapchain.images[imageIndex], VK_IMAGE_ASPECT_COLOR_BIT,
                                 VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                                 VK_IMAGE_LAYOUT_PRESENT_SRC_KHR);
            vkEndCommandBuffer(commands);

            VkSemaphoreSubmitInfo waitInfo{VK_STRUCTURE_TYPE_SEMAPHORE_SUBMIT_INFO};
            waitInfo.semaphore = acquired;
            waitInfo.stageMask = VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT;
            VkSemaphoreSubmitInfo signalInfo{VK_STRUCTURE_TYPE_SEMAPHORE_SUBMIT_INFO};
            signalInfo.semaphore = renderedSemaphore;
            signalInfo.stageMask = VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT;
            VkCommandBufferSubmitInfo commandInfo{VK_STRUCTURE_TYPE_COMMAND_BUFFER_SUBMIT_INFO};
            commandInfo.commandBuffer = commands;
            VkSubmitInfo2 submit{VK_STRUCTURE_TYPE_SUBMIT_INFO_2};
            submit.waitSemaphoreInfoCount = 1;
            submit.pWaitSemaphoreInfos = &waitInfo;
            submit.commandBufferInfoCount = 1;
            submit.pCommandBufferInfos = &commandInfo;
            submit.signalSemaphoreInfoCount = 1;
            submit.pSignalSemaphoreInfos = &signalInfo;
            vkQueueSubmit2(device->queue(), 1, &submit, frameDone);

            VkPresentInfoKHR present{VK_STRUCTURE_TYPE_PRESENT_INFO_KHR};
            present.waitSemaphoreCount = 1;
            present.pWaitSemaphores = &renderedSemaphore;
            present.swapchainCount = 1;
            present.pSwapchains = &swapchain.handle;
            present.pImageIndices = &imageIndex;
            const VkResult presented = vkQueuePresentKHR(device->queue(), &present);
            if (presented == VK_ERROR_OUT_OF_DATE_KHR || presented == VK_SUBOPTIMAL_KHR) {
                rebuild = true;
            }

            if (options.frames != 0 && ++presentedFrames >= options.frames) {
                running = false;  // smoke-test mode
            }
            const auto now = Clock::now();
            frameMs = std::chrono::duration<float, std::milli>(now - last).count();
            last = now;
        }

        vkDeviceWaitIdle(device->device());
        ImGui_ImplSDL3_Shutdown();
    }

    client.stop();
    vkDestroyFence(device->device(), frameDone, nullptr);
    vkDestroySemaphore(device->device(), acquired, nullptr);
    vkDestroySemaphore(device->device(), renderedSemaphore, nullptr);
    destroySwapchain(*device, swapchain);
    const std::uint32_t validationErrors = device->validationErrorCount();
    device.reset();
    SDL_DestroyWindow(window);
    SDL_Quit();
    return validationErrors == 0 ? 0 : 6;
}

}  // namespace

int runViewer(const Options& options) {
    bool ok = false;
    const proto::Endpoint endpoint = resolveEndpoint(options, ok);
    if (!ok) {
        return 2;
    }
    // Signals asked for on the command line are plotted in the Signals workspace.
    Workspace workspace = options.scopeSignals.empty() ? Workspace::Scene : Workspace::Signals;
    if (!options.workspace.empty()) {
        const std::optional<Workspace> parsed = parseWorkspace(options.workspace);
        if (!parsed) {
            std::fprintf(stderr, "error: unknown workspace '%s' (scene, domains, signals)\n",
                         options.workspace.c_str());
            return 2;
        }
        workspace = *parsed;
    }
    return options.headless ? runHeadless(options, endpoint, workspace)
                            : runWindowed(options, endpoint, workspace);
}

}  // namespace lagrange::viewer
