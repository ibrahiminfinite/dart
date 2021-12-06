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

#include "dart/gui/viewer.hpp"

#include "dart/common/logging.hpp"

namespace dart::gui {

//==============================================================================
class InputHandler : public ::osgGA::GUIEventHandler
{
public:
  InputHandler(Viewer* viewer) : m_viewer(viewer)
  {
    // Do nothing
  }

  bool handle(
      const ::osgGA::GUIEventAdapter& ea, ::osgGA::GUIActionAdapter&) override
  {
    (void)ea;
    (void)m_viewer;

    if (osgGA::GUIEventAdapter::KEYDOWN == ea.getEventType()) {
      auto& key_callback = m_viewer->get_key_callback();
      if (key_callback) {
        key_callback(ea.getKey(), 0, 0, 0);
      }
    }

    return false;
  }

protected:
private:
  Viewer* m_viewer;
};

//==============================================================================
Viewer::Viewer(const ViewerConfig& config) : m_config(config)
{
  m_root_node = std::make_unique<Node>();

  m_osg_viewer = new osgViewer::Viewer();
  m_osg_viewer->setSceneData(m_root_node->get_mutable_osg_node());

  m_osg_viewer->addEventHandler(new InputHandler(this));

  if (m_config.offscreen) {
    set_offscreen();
  } else {
    set_window(0, 0, m_config.width, m_config.height);
  }
}

//==============================================================================
Viewer::~Viewer()
{
  // Do nothing
}

//==============================================================================
void Viewer::set_window(
    int x, int y, int width, int height, unsigned int screen_num)
{
  m_osg_viewer->setUpViewInWindow(x, y, width, height, screen_num);
}

//==============================================================================
void Viewer::set_clear_color(const Color4d& color)
{
  osg::Vec4 osg_color;
  osg_color.r() = color[0];
  osg_color.g() = color[1];
  osg_color.b() = color[2];
  osg_color.a() = color[3];
  m_osg_viewer->getCamera()->setClearColor(osg_color);
}

//==============================================================================
void Viewer::set_target_fps(double fps)
{
  (void)fps;
}

//==============================================================================
int Viewer::run()
{
  return m_osg_viewer->run();
}

//==============================================================================
void Viewer::set_key_callback(
    std::function<void(int, int, int, int)>&& callback)
{
  m_key_callback = std::move(callback);
}

//==============================================================================
void Viewer::set_offscreen()
{
  osg::GraphicsContext::ScreenIdentifier main_screen_id;
  main_screen_id.readDISPLAY();
  main_screen_id.setUndefinedScreenDetailsToDefaultScreen();
  std::cout << "\n[DEBUG] " << main_screen_id.displayName() << std::endl;
  std::cout << "[DEBUG] " << main_screen_id.hostName << std::endl;
  std::cout << "[DEBUG] " << main_screen_id.displayNum << std::endl;
  std::cout << "[DEBUG] " << main_screen_id.screenNum << std::endl;

  osg::GraphicsContext::WindowingSystemInterface* wsi
      = osg::GraphicsContext::getWindowingSystemInterface();
  if (!wsi) {
    DART_ERROR("No WindowSystemInterface available, cannot create windows.");
    return;
  }
  // wsi->getScreenResolution(main_screen_id, m_impl->width, m_impl->height);

  DART_INFO("{} screen(s) detected", wsi->getNumScreens(main_screen_id));

  // Create graphics context
  osg::ref_ptr<osg::DisplaySettings> ds = new osg::DisplaySettings;
  osg::ref_ptr<osg::GraphicsContext::Traits> osg_traits
      = new osg::GraphicsContext::Traits(ds);
  osg_traits->x = 0;
  osg_traits->y = 0;
  osg_traits->width = m_config.width;
  osg_traits->height = m_config.height;
  osg_traits->red = 8;
  osg_traits->green = 8;
  osg_traits->blue = 8;
  osg_traits->alpha = 8;
  osg_traits->depth = 24;
  osg_traits->windowDecoration = false;
  osg_traits->pbuffer = true;
  osg_traits->doubleBuffer = true;
  std::cout << "\n[DEBUG] " << osg_traits->displayName() << std::endl;
  std::cout << "[DEBUG] " << osg_traits->hostName << std::endl;
  std::cout << "[DEBUG] " << osg_traits->displayNum << std::endl;
  std::cout << "[DEBUG] " << osg_traits->screenNum << std::endl;
  osg_traits->readDISPLAY();
  std::cout << "\n[DEBUG] " << osg_traits->displayName() << std::endl;
  std::cout << "[DEBUG] " << osg_traits->hostName << std::endl;
  std::cout << "[DEBUG] " << osg_traits->displayNum << std::endl;
  std::cout << "[DEBUG] " << osg_traits->screenNum << std::endl;
  osg_traits->setUndefinedScreenDetailsToDefaultScreen();
  std::cout << "\n[DEBUG] " << osg_traits->displayName() << std::endl;
  std::cout << "[DEBUG] " << osg_traits->hostName << std::endl;
  std::cout << "[DEBUG] " << osg_traits->displayNum << std::endl;
  std::cout << "[DEBUG] " << osg_traits->screenNum << std::endl;
  osg::ref_ptr<osg::GraphicsContext> gc
      = osg::GraphicsContext::createGraphicsContext(osg_traits);
  if (!gc || !gc->valid()) {
    DART_ERROR("GraphicsWindow has not been created successfully.");
  }

  // Set OSG camera
  auto osg_camera = m_osg_viewer->getCamera();
  osg_camera->setGraphicsContext(gc);
}

} // namespace dart::gui
