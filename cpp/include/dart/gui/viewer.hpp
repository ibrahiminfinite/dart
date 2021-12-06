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

#pragma once

#include <memory>

#include "dart/gui/color.hpp"
#include "dart/gui/export.hpp"
#include "dart/gui/node/node.hpp"
#include "dart/gui/osg_include.hpp"

namespace dart::gui {

struct DART_GUI_API ViewerConfig
{
  int x = 0;
  int y = 0;
  int width = 1366;
  int height = 768;
  std::string title = "notitle";
  bool offscreen = false;
  double target_fps = 60;
};

class DART_GUI_API Viewer
{
public:
  Viewer(const ViewerConfig& config = ViewerConfig());

  virtual ~Viewer();

  void set_window(
      int x, int y, int width, int height, unsigned int screenNum = 0);

  void set_clear_color(const Color4d& color);

  void set_target_fps(double fps);

  int run();

  void set_key_callback(
      std::function<void(int key, int scancode, int action, int mods)>&&
          callback);

  std::function<void(int key, int scancode, int action, int mods)>&
  get_key_callback()
  {
    return m_key_callback;
  }

protected:
  void set_offscreen();

  ViewerConfig m_config;

  osg::ref_ptr<osgViewer::Viewer> m_osg_viewer;

  std::unique_ptr<Node> m_root_node;

  std::function<void(int key, int scancode, int action, int mods)>
      m_key_callback;
};

} // namespace dart::gui
