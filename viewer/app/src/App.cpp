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
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <imgui_impl_sdl3.h>
#include <imgui_impl_vulkan.h>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include <SDL3/SDL.h>
#include <SDL3/SDL_vulkan.h>
#include <imgui.h>
#include <implot.h>

namespace lagrange::viewer {

namespace {

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

// Everything both modes share: ImGui, the panels, the camera and the scene renderer.
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
        if (scene_ && scene_->statics->stagePath != framedStage_) {
            frameScene(*scene_);
            framedStage_ = scene_->statics->stagePath;
            selectedPrim_ = kNone;
            // A prim asked for on the command line is selected in the first stage shown.
            for (std::uint32_t i = 0; !selectPath_.empty() && i < scene_->statics->prims.size();
                 ++i) {
                if (scene_->statics->prims[i].path == selectPath_) {
                    selectedPrim_ = i;
                }
            }
            selectPath_.clear();
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
        PanelContext context{client_,         scene_,  client_.status(), client_.connection(),
                             client_.stats(), camera_, selectedPrim_};
        context.catalog = client_.catalog();
        context.notice = client_.notice();
        context.canOpenFiles = canOpenFiles_;
        context.initialScopeSignals = &scopeSignals_;
        context.workspace = &workspace_;
        context.profileSession = &profileSession_;
        context.scopeSession = &scopeSession_;
        context.viewportTexture =
            static_cast<std::uint64_t>(reinterpret_cast<std::uintptr_t>(texture_));
        context.viewportWidth = renderer_.extent().width;
        context.viewportHeight = renderer_.extent().height;
        context.deviceName = device_.info().name;
        context.tierName = gpu::toString(device_.tier());
        context.frameMs = frameMs;
        context.drawCalls = renderer_.lastDrawCount();
        drawPanels(panels_, context);
        openFileRequested_ = context.openFileRequested;
        requestedWidth_ = context.requestedViewportWidth;
        requestedHeight_ = context.requestedViewportHeight;
    }

    /// Records the scene pass. The UI pass (ImGui draw data) is recorded by the caller.
    void renderScene(VkCommandBuffer commands) {
        renderer_.render(commands, scene_.get(), camera_);
    }

    [[nodiscard]] const std::shared_ptr<const SceneFrame>& scene() const { return scene_; }
    [[nodiscard]] std::uint32_t drawCalls() const { return renderer_.lastDrawCount(); }
    void setCanOpenFiles(bool value) { canOpenFiles_ = value; }
    void setWorkspace(Workspace workspace) { workspace_ = workspace; }
    void setInitialSelection(std::string path) { selectPath_ = std::move(path); }
    void setCursors(bool on) { cursorsWanted_ = on; }
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
    }

    const gpu::Device& device_;
    ViewerClient& client_;
    SceneRenderer renderer_;
    std::vector<std::unique_ptr<Panel>> panels_;
    Camera camera_;
    std::uint32_t selectedPrim_{kNone};
    std::shared_ptr<const SceneFrame> scene_;
    VkDescriptorSet texture_{VK_NULL_HANDLE};
    VkFormat targetFormat_{VK_FORMAT_UNDEFINED};
    std::uint32_t requestedWidth_{0};
    std::uint32_t requestedHeight_{0};
    std::string iniPath_;
    std::vector<std::string> scopeSignals_;
    bool canOpenFiles_{false};
    Workspace workspace_{Workspace::Scene};
    ProfileSession profileSession_;
    ScopeSession scopeSession_;
    std::vector<std::pair<std::string, bool>> domainEdits_;
    bool applyProfile_{false};
    std::string selectPath_;
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
    client.start();
    if (!options.openPath.empty()) {
        openFile(client, options.openPath);
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
        ui.setInitialSelection(options.selectPath);
        ui.setCursors(options.cursors);
        ui.setProfileEdits(options.domainEdits, options.applyProfile);
        ImGuiIO& io = ImGui::GetIO();
        io.DisplaySize =
            ImVec2(static_cast<float>(extent.width), static_cast<float>(extent.height));

        const auto deadline = Clock::now() + std::chrono::milliseconds(options.waitMs);
        bool first = true;
        // Frames are counted from the moment the scene is there; before that we keep
        // drawing (the "not connected" state) until the deadline.
        const std::uint32_t wanted = options.frames != 0 ? options.frames : 120;
        while (rendered < wanted) {
            // With --open, wait for the requested file's scene, not the one served before it.
            const auto current = client.scene();
            const bool haveScene =
                current != nullptr &&
                (options.openPath.empty() || client.notice().text.rfind("Loading", 0) != 0);
            if (!haveScene && Clock::now() > deadline) {
                std::fprintf(stderr, "error: no scene from %s within %u ms (%s)\n",
                             proto::toString(endpoint).c_str(), options.waitMs,
                             client.connection().message.c_str());
                exitCode = 4;
                break;
            }
            ui.prepare();
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
            "\"log_lines\": %llu, \"attributes\": %zu}\n",
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
            client.attributes() ? client.attributes()->attributes.size() : std::size_t{0});
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

    ViewerClient client(endpoint);
    client.start();
    if (!options.openPath.empty()) {
        openFile(client, options.openPath);
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
        ui.setWorkspace(workspace);
        ui.setInitialSelection(options.selectPath);
        ui.setCursors(options.cursors);
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
                    openFile(client, event.drop.data);  // a file dropped on the window
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
                    {"Robots and recordings", "urdf;xacro;mcap"}, {"All files", "*"}};
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
                    openFile(client, path);
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
