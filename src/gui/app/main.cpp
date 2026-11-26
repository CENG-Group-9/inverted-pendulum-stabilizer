#include "digital_twin.h"
#include "system_status.h"
#include <X11/Xlib.h>
#include <cassert>
#include <cstddef>
#include <cstdio>
#include <gl.h>
#include <iostream>

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include "control.h"
#include "plots.h"

namespace pendulum::gui {
int width = 1920;
int height = 1080;
GLFWwindow* window = NULL;

namespace {
// Callback functions for GLFW
void error_callback(int error, const char* description) {
    fprintf(stderr, "Error: %s\n", description);
}

static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
        glfwSetWindowShouldClose(window, GLFW_TRUE);
    }
}

void setFrameBufferSizeCallback(GLFWwindow* window, int width, int height) {
    glViewport(0, 0, width, height);
}

// Initialization of the context of GLFW & imgui
ImGuiIO& initialize() {
    // glfw
    if (!glfwInit()) {
        std::cerr << "ERROR::GLFW_INITIALIZE!\n";
    }
    glfwSetErrorCallback(error_callback);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    // Load OpenGL function pointers
    window = glfwCreateWindow(width, height, "Inverted Pendulum", NULL, NULL);
    if (!window) {
        std::cerr << "ERROR::WINDOW_INITIALIZE!\n";
    }

    glfwSetKeyCallback(window, key_callback);
    glfwMakeContextCurrent(window);
    gladLoadGL(glfwGetProcAddress);

    glfwSetFramebufferSizeCallback(window, setFrameBufferSizeCallback);

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard; // Enable Keyboard Controls
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;  // Enable Gamepad Controls
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;     // Enable Docking
    io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;   // Enable multi viewports

    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init();

    return io;
}

void terminate();

int run() {
    initialize();
    DigitalTwin digital_twin;
    StatusController status_controller;

    while (!glfwWindowShouldClose(window)) {
        // main render loop
        int framebuffer_width_pixels = 0;
        int framebuffer_height_pixels = 0;
        glfwGetFramebufferSize(window, &framebuffer_width_pixels, &framebuffer_height_pixels);

        glViewport(0, 0, framebuffer_width_pixels, framebuffer_height_pixels);
        glClearColor(0.08F, 0.09F, 0.11F, 1.0F);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // update system status
        status_controller.update_status();
        // Render digital twin
        // digital_twin.update_pose(status_controller.current_status());
        digital_twin.render(framebuffer_width_pixels, framebuffer_height_pixels, status_controller.current_status());

        // Render Dear ImGui
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        pendulum::gui::ControlWindow::show_control_window();

        // TODO: if demo is active
        // otherwise read actual data here
        // PlotsWindow::demo_status_data();
        PlotsWindow::show_plots_window(status_controller.current_status());

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    terminate();
    return 0;
}

void terminate() {
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();
}
} // namespace
} // namespace pendulum::gui

int main() {
    return pendulum::gui::run();
}
