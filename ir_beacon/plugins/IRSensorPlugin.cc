/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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
 *
 */


#include <gazebo/gazebo_config.h>
#include <gazebo/rendering/Scene.hh>
#include <gazebo/rendering/RenderTypes.hh>

#include "IRSensorPlugin.hh"

namespace gazebo
{
  /// \brief Material switcher for toggling the material of the targets and
  /// background for simulating IR camera.
  class IRMaterialHandler
      : public Ogre::RenderTargetListener, Ogre::MaterialManager::Listener
  {
    /// \brief Constructor
    /// \param[in] _camera Pointer to a camera.
    public: IRMaterialHandler(const rendering::CameraPtr &_camera);

    /// \brief Destructor
    public: ~IRMaterialHandler() = default;

    /// \brief Set the material scheme that will be applied to camera 
    /// \param[in] _scheme Name of material scheme
    public: void SetMaterialScheme(const std::string &_scheme);

    /// \brief Set the target that will be rendered non-black
    public: void SetTargets(const std::set<std::string> &_targets);

    /// \brief PreRender callback. Currently used to find visual representing
    /// the targets
    public: void PreRender();

    /// \brief Get the material scheme applied to the camera
    public: std::string MaterialScheme() const;

    /// \brief Ogre's pre render update callback
    /// \param[in] _evt Ogre render target event containing information about
    /// the source render target.
    public: virtual void preRenderTargetUpdate(
                const Ogre::RenderTargetEvent &_evt);

    /// \brief Ogre's post render update callback
    /// \param[in] _evt Ogre render target event containing information about
    /// the source render target.
    public: virtual void postRenderTargetUpdate(
                const Ogre::RenderTargetEvent &_evt);

    /// \brief Ogre callback that is used to specify the material to use when
    /// the requested scheme is not found
    /// \param[in] _schemeIndex Index of scheme requested
    /// \param[in] _schemeName Name of scheme requested
    /// \param[in] _originalMaterial Orignal material that does not contain
    /// the requested scheme
    /// \param[in] _lodIndex The material level-of-detail
    /// \param[in] _rend Pointer to the Ogre::Renderable object requesting
    /// the use of the techinique
    /// \return The Ogre material technique to use when scheme is not found.
    public: virtual Ogre::Technique *handleSchemeNotFound(
                uint16_t _schemeIndex, const Ogre::String &_schemeName,
                Ogre::Material *_originalMaterial, uint16_t _lodIndex,
                const Ogre::Renderable *_rend);

    /// \brief Tag a visual and its children as IR target
    /// \param[in] _vis Visual to be tagged
    public: void TagIRVisual(rendering::VisualPtr _vis);

    /// \brief Pointer to the camera
    private: rendering::CameraPtr camera;

    /// \brief Name of the original material scheme
    private: std::string originalMaterialScheme;

    /// \brief Name of the material scheme being used.
    private: std::string materialScheme;

    /// \brief List of target names that will be detected by the camera
    private: std::set<std::string> targets;

    /// \brief Connection pointer used to connecto pre render events.
    private: event::ConnectionPtr preRenderCon;
  };
}

using namespace gazebo;

GZ_REGISTER_SENSOR_PLUGIN(IRSensorPlugin)

/////////////////////////////////////////////////
IRMaterialHandler::IRMaterialHandler(
    const rendering::CameraPtr &_camera)
{
  this->camera = _camera;

  this->materialScheme = "";

  if (!this->camera)
  {
    gzerr << "Cannot create a material handler. "
          << "Camera is NULL" << std::endl;
    return;
  }

  // create background color material
  std::string bgMatStr = "Gazebo/IRBGBlack";
  if (!Ogre::MaterialManager::getSingleton().resourceExists(bgMatStr))
  {
    Ogre::MaterialPtr bgMat = Ogre::MaterialManager::getSingleton().create(
      bgMatStr, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    Ogre::Technique *tech = bgMat->getTechnique(0);
    Ogre::Pass *pass = tech->getPass(0);
    pass->setAmbient(0, 0, 0);
    pass->setDiffuse(0, 0, 0, 1);
    pass->setSpecular(0, 0, 0, 1);
    bgMat->load();
  }

  // create target material
  std::string targetMatStr = "Gazebo/IRTargetWhite";
  if (!Ogre::MaterialManager::getSingleton().resourceExists(targetMatStr))
  {
    Ogre::MaterialPtr targetMat = Ogre::MaterialManager::getSingleton().create(
      targetMatStr, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    Ogre::Technique *tech = targetMat->getTechnique(0);
    tech->setLightingEnabled(false);
    Ogre::Pass *pass = tech->getPass(0);
    pass->setAmbient(1, 1, 1);
    pass->setDiffuse(1, 1, 1, 1);
    pass->setSpecular(1, 1, 1, 1);
    targetMat->load();
  }
}

/////////////////////////////////////////////////
void IRMaterialHandler::SetMaterialScheme(const std::string &_scheme)
{
  if (!this->camera || !this->camera->OgreViewport())
    return;

  this->materialScheme = _scheme;

  if (_scheme.empty())
  {
    this->camera->OgreViewport()->setMaterialScheme(
        this->originalMaterialScheme);
    this->camera->OgreViewport()->getTarget()->removeListener(
        this);
  }
  else
  {
    this->originalMaterialScheme =
        this->camera->OgreViewport()->getMaterialScheme();

    this->camera->OgreViewport()->setMaterialScheme(_scheme);
    this->camera->OgreViewport()->getTarget()->addListener(
        this);
  }
}

/////////////////////////////////////////////////
void IRMaterialHandler::SetTargets(const std::set<std::string> &_targets)
{
  this->targets = _targets;

  this->preRenderCon = event::Events::ConnectPreRender(
      std::bind(&IRMaterialHandler::PreRender, this));
}

/////////////////////////////////////////////////
std::string IRMaterialHandler::MaterialScheme() const
{
  return this->materialScheme;
}

/////////////////////////////////////////////////
void IRMaterialHandler::TagIRVisual(rendering::VisualPtr _vis)
{
  if (!_vis)
    return;

  // tag child visuals too
  for (unsigned int i = 0; i <_vis->GetChildCount(); ++i)
  {
    rendering::VisualPtr child = _vis->GetChild(i);
    this->TagIRVisual(child);
  }

  if (_vis->GetType() != rendering::Visual::VT_VISUAL)
    return;


  // create a new visual representing the glow of the IR LED light source
  // so that the target appears larger in the image
#if GAZEBO_MAJOR_VERSION >= 8
  rendering::VisualPtr visGlow(
      new rendering::Visual(_vis->Name()+"_glow", _vis, false));
#else
  rendering::VisualPtr visGlow(
      new rendering::Visual(_vis->GetName()+"_glow", _vis, false));
#endif
  visGlow->Load();
  visGlow->AttachMesh("unit_sphere");
  visGlow->SetScale(2 * ignition::math::Vector3d::One);

  Ogre::SceneNode *node = visGlow->GetSceneNode();
  Ogre::MovableObject *obj = node->getAttachedObject(0);
  obj->getUserObjectBindings().setUserAny(Ogre::Any(std::string("ir_glow")));
}


/////////////////////////////////////////////////
void IRMaterialHandler::PreRender()
{
  rendering::ScenePtr scene = this->camera->GetScene();
  for (auto it = this->targets.begin(); it != this->targets.end();)
  {
    rendering::VisualPtr vis = scene->GetVisual(*it);
    if (!vis)
    {
      ++it;
      continue;
    }
    // tag it as an IR target
    this->TagIRVisual(vis);

    this->targets.erase(it++);
  }

  if (this->targets.empty())
    this->preRenderCon.reset();
}

/////////////////////////////////////////////////
void IRMaterialHandler::preRenderTargetUpdate(
    const Ogre::RenderTargetEvent &/*_evt*/)
{
  this->camera->OgreViewport()->setBackgroundColour(
      Ogre::ColourValue(0, 0, 0, 1));

  this->camera->OgreViewport()->setShadowsEnabled(false);

  Ogre::MaterialManager::getSingleton().addListener(this);
}

/////////////////////////////////////////////////
void IRMaterialHandler::postRenderTargetUpdate(
    const Ogre::RenderTargetEvent &/*_evt*/)
{
  Ogre::MaterialManager::getSingleton().removeListener(this);
}

/////////////////////////////////////////////////
Ogre::Technique *IRMaterialHandler::handleSchemeNotFound(
    uint16_t /*_schemeIndex*/, const Ogre::String & /*_schemeName*/,
    Ogre::Material * /*_originalMaterial*/, uint16_t /*_lodIndex*/,
    const Ogre::Renderable *_rend)
{
  if (_rend && typeid(*_rend) == typeid(Ogre::SubEntity))
  {
    std::string material = "";

    const Ogre::SubEntity *subEntity =
      static_cast<const Ogre::SubEntity *>(_rend);

    if (!subEntity)
    {
      gzerr << "Unable to get an Ogre sub-entity" << std::endl;
      return nullptr;
    }

    Ogre::Entity *entity = subEntity->getParent();
    if (!entity)
    {
      gzerr << "Unable to get an Ogre entity" << std::endl;
      return nullptr;
    }

    if (entity->getUserObjectBindings().getUserAny().isEmpty())
      return nullptr;

    std::string userAny = "";
    try
    {
      userAny = Ogre::any_cast<std::string>(
          entity->getUserObjectBindings().getUserAny());
    }
    catch(Ogre::Exception &e)
    {
      gzerr << "Unable to cast Ogre user data" << std::endl;
      return nullptr;
    }

    // targets will have the user data "ir_glow"
    // everything else - make it black.
    if (userAny == "ir_glow")
    {
      material = "Gazebo/IRTargetWhite";
    }
    else
    {
      material = "Gazebo/IRBGBlack";
    }

    // set the material for the models
    Ogre::ResourcePtr res =
        Ogre::MaterialManager::getSingleton().getByName(material);
    if (res.isNull())
    {
      Ogre::MaterialManager::getSingleton().load(material,
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    }

    // OGRE 1.9 changes the shared pointer definition
    // But the 1.9 RC, which we're using on Windows, doesn't have the
    // staticCast change.  It will be in the final release.
    #if (OGRE_VERSION < ((1 << 16) | (9 << 8) | 0)) || defined(_WIN32)
    // Make sure we keep the same depth properties so that
    // certain overlay objects can be picked by the mouse.
    Ogre::Technique *newTechnique =
        static_cast<Ogre::MaterialPtr>(res)->getTechnique(0);
    #else
    Ogre::Technique *newTechnique =
        res.staticCast<Ogre::Material>()->getTechnique(0);
    #endif

    return newTechnique;
  }
  return nullptr;
}


/////////////////////////////////////////////////
IRSensorPlugin::IRSensorPlugin()
: SensorPlugin()
{
}

/////////////////////////////////////////////////
IRSensorPlugin::~IRSensorPlugin()
{
  this->parentSensor.reset();
  this->camera.reset();
}

/////////////////////////////////////////////////
void IRSensorPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
  if (!_sensor)
    gzerr << "Invalid sensor pointer." << std::endl;

  this->parentSensor =
    std::dynamic_pointer_cast<sensors::CameraSensor>(_sensor);

  if (!this->parentSensor)
  {
    gzerr << "IRSensorPlugin requires a CameraSensor." << std::endl;
    return;
  }

  this->camera = this->parentSensor->Camera();

  if (!this->parentSensor)
  {
    gzerr << "IRSensorPlugin not attached to a camera sensor" << std::endl;
    return;
  }

  // load the fiducial targets
  std::set<std::string> targets;
  if (_sdf->HasElement("target"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("target");
    while (elem)
    {
      targets.insert(elem->Get<std::string>());
      elem = elem->GetNextElement("target");
    }
  }
  else
  {
    gzwarn << "No IR targets specified!" << std::endl;
  }

  this->materialHandler.reset(new IRMaterialHandler(camera));
  this->materialHandler->SetMaterialScheme("IR");
  this->materialHandler->SetTargets(targets);
}
