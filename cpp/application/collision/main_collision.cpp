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

#include "dart/all.hpp"

#include "collision_scene_node.hpp"
#include "collision_scene_viewer.hpp"

using namespace dart;

int main()
{
  auto collision_engine = collision::get_default_engine();
  auto collision_scene = collision_engine->create_scene();

  auto viewer_config = gui::ViewerConfig();
  viewer_config.title = "DART collision testbed";
  viewer_config.offscreen = false;

  auto viewer = CollisionSceneViewer(viewer_config);
  viewer.set_collision_scene(collision_scene);
  viewer.set_clear_color(gui::White());
  viewer.set_key_callback([&](int key, int scancode, int action, int mods) {
    (void)key;
    (void)scancode;
    (void)action;
    (void)mods;
    static bool toggle = true;
    if (toggle) {
      viewer.grid_on();
    } else {
      viewer.grid_off();
    }
    toggle = !toggle;
  });
  viewer.run();

  return 0;
}
