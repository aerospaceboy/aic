/*
 * Copyright 2026 YZBot contributors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <algorithm>
#include <cstdlib>
#include <string>

#include <QTimer>
#include <QWidget>

#include <gazebo/common/Console.hh>
#include <gazebo/gui/GuiEvents.hh>
#include <gazebo/gui/GuiIface.hh>
#include <gazebo/gui/GuiPlugin.hh>
#include <gazebo/gui/MainWindow.hh>
#include <gazebo/rendering/Camera.hh>
#include <gazebo/rendering/Scene.hh>
#include <gazebo/rendering/rendering.hh>

namespace gazebo
{
namespace
{
double EnvironmentDouble(const char *_name, const double _fallback)
{
  const char *raw = std::getenv(_name);
  if (!raw)
    return _fallback;

  try
  {
    return std::stod(raw);
  }
  catch (...)
  {
    return _fallback;
  }
}

int EnvironmentInt(const char *_name, const int _fallback)
{
  return static_cast<int>(EnvironmentDouble(_name, _fallback));
}
}  // namespace

/// Keep the Gazebo client useful for observation without letting it render at
/// the desktop refresh rate. This plugin changes only the client-side scene;
/// gzserver physics and server-side RGB-D / lidar sensors are untouched.
class GazeboDebugGuiPlugin : public GUIPlugin
{
  public: GazeboDebugGuiPlugin()
      : renderRate(std::max(1.0, EnvironmentDouble(
            "YZBOT_GAZEBO_GUI_RENDER_RATE", 60.0))),
        windowWidth(std::max(640, EnvironmentInt(
            "YZBOT_GAZEBO_GUI_WIDTH", 1100))),
        windowHeight(std::max(480, EnvironmentInt(
            "YZBOT_GAZEBO_GUI_HEIGHT", 700)))
  {
    this->setVisible(false);
    this->timer = new QTimer(this);
    QObject::connect(this->timer, &QTimer::timeout, [this]()
    {
      this->ApplyLowLoadSettings();
    });
    this->timer->start(250);
  }

  public: void Load(sdf::ElementPtr /*_sdf*/) override
  {
    gzmsg << "YZBot low-load GUI requested: " << this->renderRate
          << " Hz, " << this->windowWidth << "x" << this->windowHeight
          << std::endl;
  }

  private: void ApplyLowLoadSettings()
  {
    auto *window = gui::get_main_window();
    auto camera = gui::get_active_camera();
    auto scene = camera ? camera->GetScene() : rendering::ScenePtr();
    if (!window || !scene || !camera)
      return;

    if (!this->settingsApplied)
    {
      // These are client-scene flags. Sensor rendering lives in gzserver and
      // is deliberately not modified.
      scene->SetShadowsEnabled(false);
      scene->SetGrid(false);
      scene->SetWireframe(false);
      scene->ShowCollisions(false);
      scene->ShowCOMs(false);
      scene->ShowInertias(false);
      scene->ShowLinkFrames(false);
      scene->ShowJoints(false);
      scene->EnableVisualizations(false);

      // GLWidget::SetRenderRate updates both the user camera and the Qt repaint
      // timer. Calling Camera::SetRenderRate separately can leave an old OGRE
      // viewport exposed after resizing on VMware's SVGA3D driver.
      gui::Events::setRenderRate(this->renderRate);

      // Give the resized GL widget a few event-loop turns to recreate its
      // render target without adding any steady-state rendering work.
      for (const int delayMs : {0, 100, 300})
      {
        QTimer::singleShot(delayMs, window, [window]()
        {
          window->update();
          for (auto *child : window->findChildren<QWidget *>())
            child->update();
        });
      }

      gzmsg << "YZBot low-load GUI active: shadows/grid/diagnostic overlays "
            << "off, " << this->renderRate << " Hz, " << this->windowWidth
            << "x" << this->windowHeight << std::endl;
      this->settingsApplied = true;
    }

    // Gazebo Classic can create a native 640x480 camera-display QWidget on top
    // of the user-camera render window even when the SDF sensor visualization
    // is disabled. On VMware SVGA3D that widget is black. Hiding only this
    // client-side overlay leaves the gzserver RGB-D sensor and ROS topics live.
    for (auto *child : window->findChildren<QWidget *>())
    {
      if (child != window && child->isVisible() && child->width() == 640 &&
          child->height() == 480)
      {
        gzmsg << "YZBot hiding fixed camera GUI overlay: class="
              << child->metaObject()->className() << " object="
              << child->objectName().toStdString() << " size=640x480"
              << std::endl;
        child->hide();
        this->cameraOverlayHidden = true;
      }
    }

    // The baseline client opened maximized at 2288x1308. Mutter/Xwayland can
    // re-maximize a freshly mapped window right after the plugin's first
    // resize, so re-assert the bounded viewport for the first five seconds.
    if (this->geometryAttempts < 20)
    {
      window->showNormal();
      window->resize(this->windowWidth, this->windowHeight);
      ++this->geometryAttempts;
    }

    // Camera widgets are created shortly after the user camera. Stop scanning
    // as soon as the overlay is found, or after 20 seconds if none is present,
    // but never before the bounded viewport has been re-asserted.
    if ((this->cameraOverlayHidden || ++this->overlayScanCount >= 80) &&
        this->geometryAttempts >= 20)
      this->timer->stop();
  }

  private: QTimer *timer{nullptr};
  private: double renderRate;
  private: int windowWidth;
  private: int windowHeight;
  private: bool settingsApplied{false};
  private: bool cameraOverlayHidden{false};
  private: int overlayScanCount{0};
  private: int geometryAttempts{0};
};

GZ_REGISTER_GUI_PLUGIN(GazeboDebugGuiPlugin)
}  // namespace gazebo
