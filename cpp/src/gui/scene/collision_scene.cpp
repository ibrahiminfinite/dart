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

#include "dart/gui/scene/collision_scene.hpp"

#include <unordered_map>
#include <unordered_set>

#include "dart/common/logging.hpp"
#include "dart/common/macro.hpp"
#include "dart/gui/camera.hpp"
#include "dart/gui/raylib_include.hpp"

namespace raylib {
#define RLIGHTS_IMPLEMENTATION
#include "rlights.h"
} // namespace raylib

namespace dart::gui {

const std::string base_lighting_vs = R"(#version 330

    // Input vertex attributes
    in vec3 vertexPosition;
    in vec2 vertexTexCoord;
    in vec3 vertexNormal;
    in vec4 vertexColor;

    // Input uniform values
    uniform mat4 mvp;
    uniform mat4 matModel;
    uniform mat4 matNormal;

    // Output vertex attributes (to fragment shader)
    out vec3 fragPosition;
    out vec2 fragTexCoord;
    out vec4 fragColor;
    out vec3 fragNormal;

    // NOTE: Add here your custom variables

    void main()
    {
        // Send vertex attributes to fragment shader
        fragPosition = vec3(matModel*vec4(vertexPosition, 1.0));
        fragTexCoord = vertexTexCoord;
        fragColor = vertexColor;
        fragNormal = normalize(vec3(matNormal*vec4(vertexNormal, 1.0)));

        // Calculate final vertex position
        gl_Position = mvp*vec4(vertexPosition, 1.0);
    })";

const std::string lighting_fs = R"(#version 330

// Input vertex attributes (from vertex shader)
in vec3 fragPosition;
in vec2 fragTexCoord;
in vec4 fragColor;
in vec3 fragNormal;

// Input uniform values
uniform sampler2D texture0;
uniform vec4 colDiffuse;

// Output fragment color
out vec4 finalColor;

// NOTE: Add here your custom variables

#define     MAX_LIGHTS              4
#define     LIGHT_DIRECTIONAL       0
#define     LIGHT_POINT             1

struct MaterialProperty {
    vec3 color;
    int useSampler;
    sampler2D sampler;
};

struct Light {
    int enabled;
    int type;
    vec3 position;
    vec3 target;
    vec4 color;
};

// Input lighting values
uniform Light lights[MAX_LIGHTS];
uniform vec4 ambient;
uniform vec3 viewPos;

void main()
{
    // Texel color fetching from texture sampler
    vec4 texelColor = texture(texture0, fragTexCoord);
    vec3 lightDot = vec3(0.0);
    vec3 normal = normalize(fragNormal);
    vec3 viewD = normalize(viewPos - fragPosition);
    vec3 specular = vec3(0.0);

    // NOTE: Implement here your fragment shader code

    for (int i = 0; i < MAX_LIGHTS; i++)
    {
        if (lights[i].enabled == 1)
        {
            vec3 light = vec3(0.0);

            if (lights[i].type == LIGHT_DIRECTIONAL)
            {
                light = -normalize(lights[i].target - lights[i].position);
            }

            if (lights[i].type == LIGHT_POINT)
            {
                light = normalize(lights[i].position - fragPosition);
            }

            float NdotL = max(dot(normal, light), 0.0);
            lightDot += lights[i].color.rgb*NdotL;

            float specCo = 0.0;
            if (NdotL > 0.0) specCo = pow(max(0.0, dot(viewD, reflect(-(light), normal))), 16.0); // 16 refers to shine
            specular += specCo;
        }
    }

    finalColor = (texelColor*((colDiffuse + vec4(specular, 1.0))*vec4(lightDot, 1.0)));
    finalColor += texelColor*(ambient/10.0)*colDiffuse;

    // Gamma correction
    finalColor = pow(finalColor, vec4(1.0/2.2));
})";

//==============================================================================
struct CollisionScene::Implementation
{
  std::shared_ptr<collision::Scene<double>> collision_scene;

  raylib::Shader shader;
  raylib::Light lights[MAX_LIGHTS];

  std::unordered_map<collision::ConstObjectPtr<double>, raylib::Model>
      m_map_object_to_model;

  raylib::Model sphere_model;

  Implementation()
  {
    // Do nothing
  }
};

//==============================================================================
CollisionScene::CollisionScene(std::shared_ptr<collision::Scene<double>> scene)
  : m_impl(std::make_unique<Implementation>())
{
  // Set collision scene
  m_impl->collision_scene = std::move(scene);

  // Initialize shaders
  m_impl->shader = raylib::LoadShaderFromMemory(
      base_lighting_vs.c_str(), lighting_fs.c_str());
  m_impl->shader.locs[raylib::SHADER_LOC_VECTOR_VIEW]
      = raylib::GetShaderLocation(m_impl->shader, "viewPos");

  // Ambient light level (some basic lighting)
  int ambientLoc = raylib::GetShaderLocation(m_impl->shader, "ambient");
  float pos[4] = {0.1, 0.1, 0.1, 0.1};
  raylib::SetShaderValue(
      m_impl->shader, ambientLoc, pos, raylib::SHADER_UNIFORM_VEC4);

  // Using 4 point lights: gold, red, green and blue

  m_impl->lights[0] = raylib::CreateLight(
      raylib::LIGHT_POINT,
      (raylib::Vector3){-2, 1, -2},
      {0, 0, 0},
      raylib::YELLOW,
      m_impl->shader);
  m_impl->lights[1] = raylib::CreateLight(
      raylib::LIGHT_POINT,
      (raylib::Vector3){2, 1, 2},
      {0, 0, 0},
      raylib::RED,
      m_impl->shader);
  m_impl->lights[2] = raylib::CreateLight(
      raylib::LIGHT_POINT,
      (raylib::Vector3){-2, 1, 2},
      {0, 0, 0},
      raylib::GREEN,
      m_impl->shader);
  m_impl->lights[3] = raylib::CreateLight(
      raylib::LIGHT_POINT,
      (raylib::Vector3){2, 1, -2},
      {0, 0, 0},
      raylib::BLUE,
      m_impl->shader);
  m_impl->lights[0].enabled = true;
  m_impl->lights[1].enabled = true;
  m_impl->lights[2].enabled = true;
  m_impl->lights[3].enabled = true;

  // Initialize sphere models
  m_impl->sphere_model
      = raylib::LoadModelFromMesh(raylib::GenMeshSphere(1, 32, 32));
  m_impl->sphere_model.materials[0].shader = m_impl->shader;

  std::cout << "[DEBUG] m_impl->sphere_model.materials: "
            << m_impl->sphere_model.materialCount << std::endl;
}

//==============================================================================
CollisionScene::~CollisionScene()
{
  raylib::UnloadModel(m_impl->sphere_model);

  raylib::UnloadShader(m_impl->shader);
}

//==============================================================================
void CollisionScene::update()
{
  // Do nothing
}

//==============================================================================
void CollisionScene::render()
{
  auto& camera = get_mutable_camera()->get_mutable_raylib_camera();

  raylib::UpdateCamera(&camera);

  // Update light values (actually, only enable/disable them)
  raylib::UpdateLightValues(m_impl->shader, m_impl->lights[0]);
  raylib::UpdateLightValues(m_impl->shader, m_impl->lights[1]);
  raylib::UpdateLightValues(m_impl->shader, m_impl->lights[2]);
  raylib::UpdateLightValues(m_impl->shader, m_impl->lights[3]);

  // Update the shader with the camera view vector (points towards { 0.0f, 0.0f,
  // 0.0f })
  float cameraPos[3]
      = {camera.position.x, camera.position.y, camera.position.z};
  SetShaderValue(
      m_impl->shader,
      m_impl->shader.locs[raylib::SHADER_LOC_VECTOR_VIEW],
      cameraPos,
      raylib::SHADER_UNIFORM_VEC3);

  raylib::BeginDrawing();
  {
    raylib::ClearBackground(raylib::RAYWHITE);

    raylib::BeginMode3D(get_mutable_camera()->get_mutable_raylib_camera());
    {
      for (auto i = 0; i < m_impl->collision_scene->get_object_count(); ++i) {
        auto collision_object = m_impl->collision_scene->get_object_by_index(i);
        auto position = collision_object->get_position();
        auto geometry = collision_object->get_geometry();
        if (auto sphere = geometry->as<math::Sphered>()) {
          const float raidus = sphere->get_radius();
          raylib::DrawModelEx(
              m_impl->sphere_model,
              {(float)position.x(), (float)position.y(), (float)position.z()},
              {1, 0, 0},
              0,
              {raidus, raidus, raidus},
              raylib::WHITE);
        }
      }

      raylib::DrawGrid(10, 1.0f);
    }
    raylib::EndMode3D();

    raylib::DrawFPS(10, 10);
  }
  raylib::EndDrawing();
}

} // namespace dart::gui
