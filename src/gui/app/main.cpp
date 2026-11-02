#include <cstdio>
#include <memory>

#include <glad/gl.h>

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <implot.h>

namespace pendulum::gui {

namespace {

constexpr int kWindowWidthPixels = 1280;
constexpr int kWindowHeightPixels = 720;
constexpr int kExitSuccess = 0;
constexpr int kExitFailure = 1;
constexpr float kClearColorRed = 0.10F;
constexpr float kClearColorGreen = 0.10F;
constexpr float kClearColorBlue = 0.12F;
constexpr float kClearColorAlpha = 1.0F;
constexpr const char* kWindowTitle = "Pendulum Stabilizer";
constexpr const char* kGlslVersion = "#version 330 core";

struct GlfwWindowDeleter {
    void operator()(GLFWwindow* window) const {
        if (window != nullptr) {
            glfwDestroyWindow(window);
        }
    }
};

using GlfwWindow = std::unique_ptr<GLFWwindow, GlfwWindowDeleter>;

struct DemoWindows {
    bool is_imgui_visible = false;
    bool is_implot_visible = false;
};

void report_glfw_error(int error_code, const char* description) {
    std::fprintf(stderr, "GLFW error %d: %s\n", error_code, description);
}

void draw_control_panel(DemoWindows& demo_windows) {
    if (ImGui::Begin("Control Panel")) {
        ImGui::Checkbox("ImGui demo", &demo_windows.is_imgui_visible);
        ImGui::Checkbox("ImPlot demo", &demo_windows.is_implot_visible);
    }
    ImGui::End();
}

void draw_telemetry_panel() {
    if (ImGui::Begin("Telemetry")) {
        ImGui::TextUnformatted("Waiting for telemetry from the microcontroller.");
    }
    ImGui::End();
}

void run_frame_loop(GLFWwindow* window) {
    DemoWindows demo_windows{};

    while (glfwWindowShouldClose(window) == GLFW_FALSE) {
        glfwPollEvents();

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGui::DockSpaceOverViewport(0, ImGui::GetMainViewport());
        draw_control_panel(demo_windows);
        draw_telemetry_panel();
        if (demo_windows.is_imgui_visible) {
            ImGui::ShowDemoWindow(&demo_windows.is_imgui_visible);
        }
        if (demo_windows.is_implot_visible) {
            ImPlot::ShowDemoWindow(&demo_windows.is_implot_visible);
        }

        ImGui::Render();

        int framebuffer_width_pixels = 0;
        int framebuffer_height_pixels = 0;
        glfwGetFramebufferSize(window, &framebuffer_width_pixels, &framebuffer_height_pixels);
        glViewport(0, 0, framebuffer_width_pixels, framebuffer_height_pixels);
        glClearColor(kClearColorRed, kClearColorGreen, kClearColorBlue, kClearColorAlpha);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
    }
}

int run() {
    glfwSetErrorCallback(report_glfw_error);
    if (glfwInit() == GLFW_FALSE) {
        return kExitFailure;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);

    GlfwWindow window{
        glfwCreateWindow(kWindowWidthPixels, kWindowHeightPixels, kWindowTitle, nullptr, nullptr)};
    if (!window) {
        std::fprintf(stderr, "Failed to create the GLFW window\n");
        glfwTerminate();
        return kExitFailure;
    }

    glfwMakeContextCurrent(window.get());
    glfwSwapInterval(1);

    if (gladLoadGL(reinterpret_cast<GLADloadfunc>(glfwGetProcAddress)) == 0) {
        std::fprintf(stderr, "Failed to load OpenGL\n");
        window.reset();
        glfwTerminate();
        return kExitFailure;
    }

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    // ImGui flags are a signed bit mask; the check has nothing to fix here.
    ImGui::GetIO().ConfigFlags |= ImGuiConfigFlags_DockingEnable; // NOLINT(bugprone-signed-bitwise)
    ImGui::StyleColorsDark();

    ImGui_ImplGlfw_InitForOpenGL(window.get(), true);
    ImGui_ImplOpenGL3_Init(kGlslVersion);

    run_frame_loop(window.get());

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();

    window.reset();
    glfwTerminate();
    return kExitSuccess;
}

} // namespace

} // namespace pendulum::gui

int main() {
    return pendulum::gui::run();
}
