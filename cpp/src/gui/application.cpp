/*
 * Copyright (c) 2011-2021, The DART development contributors:
 * https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "dart/gui/application.hpp"

#include <thread>

#include "dart/common/all.hpp"
#include "dart/gui/all.hpp"
#include "dart/simulation/all.hpp"

namespace dart::gui {

//==============================================================================
struct Application::Implementation
{
  ApplicationConfigs configs;

  std::shared_ptr<gui::Scene> scene{nullptr};

  int world_step_count_per_render = 20;

  Implementation()
  {
    // Do nothing
  }
};

//==============================================================================
Application::Application(const ApplicationConfigs& configs)
  : m_impl(std::make_unique<Implementation>())
{
  raylib::SetTraceLogLevel(raylib::LOG_NONE);

  // Store configs
  m_impl->configs = configs;

  raylib::SetConfigFlags(
      raylib::FLAG_MSAA_4X_HINT); // Enable Multi Sampling Anti Aliasing 4x (if
                                  // available)

  // Create main window
  if (m_impl->configs.headless) {
    raylib::SetWindowState(raylib::FLAG_WINDOW_HIDDEN);
  } else {
    //    m_impl->main_window = gui::MainWindow::Create();
  }
  raylib::InitWindow(800, 600, "DART GUI Application");
  raylib::SetWindowState(raylib::FLAG_WINDOW_RESIZABLE);
  raylib::SetExitKey(raylib::KEY_NULL);
  raylib::SetTargetFPS(60);

  // Create an empty scene
  m_impl->scene = EmptyScene::Create();
}

//==============================================================================
Application::~Application()
{
  // Clear resources
  m_impl->scene.reset();
  raylib::CloseWindow(); // Close window and OpenGL context
}

//==============================================================================
void Application::run(long num_steps)
{
  using namespace std::chrono_literals;

  long scene_update_steps = 0;

  while (!raylib::WindowShouldClose()) {
    raylib::PollInputEvents();

    //    raylib::BeginDrawing();
    {
      //      raylib::ClearBackground(raylib::RAYWHITE);

      if (num_steps == 0 || num_steps > scene_update_steps) {
        for (auto i = 0; i < m_impl->world_step_count_per_render; ++i) {
          m_impl->scene->update();
          scene_update_steps++;
        }
      }
      m_impl->scene->render();
    }
    //    raylib::EndDrawing();

    if (m_impl->configs.headless) {
      if (num_steps > 0 && num_steps <= scene_update_steps) {
        break;
      }
    }

    std::this_thread::sleep_for(10ms);
  }
}

//==============================================================================
void Application::set_scene(std::unique_ptr<Scene> scene)
{
  m_impl->scene = std::move(scene);
}

} // namespace dart::gui
