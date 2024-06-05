#include "Engine/app.h"
#include "Labs/4-FSM/CaseDrop.h"
#include "Labs/Common/ImGuiHelper.h"

#include <iostream>

namespace VCX::Labs::FSM {
    CaseDrop::CaseDrop() :
        _program(
            Engine::GL::UniqueProgram({
                Engine::GL::SharedShader("assets/shaders/flat.vert"),
                Engine::GL::SharedShader("assets/shaders/flat.frag") })),
        _particlesItem(Engine::GL::VertexLayout()
            .Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream , 0), Engine::GL::PrimitiveType::Points),
        _springsItem(Engine::GL::VertexLayout()
            .Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream , 0), Engine::GL::PrimitiveType::Lines) {
        _cameraManager.AutoRotate = false;
        _cameraManager.Save(_camera);
        ResetSystem();
    }

    void CaseDrop::OnSetupPropsUI() {
        if (ImGui::CollapsingHeader("Algorithm", ImGuiTreeNodeFlags_DefaultOpen)) {
            if (ImGui::Button("Reset System")) ResetSystem();
            ImGui::SameLine();
            if (ImGui::Button(_stopped ? "Start Simulation" : "Stop Simulation")) _stopped = ! _stopped;
            ImGui::SliderFloat("Part. Mass", &_massSpringSystem.Mass, .5f, 10.f);
            ImGui::SliderFloat("Spr. Stiff.", &_massSpringSystem.Stiffness, 100.f, 3000.f);
            ImGui::SliderFloat("Spr. Damp.", &_massSpringSystem.Damping, 0.9f, 1.f);
            ImGui::SliderFloat("Gravity", &_massSpringSystem.Gravity, 1.f, 10.f);
        }
        ImGui::Spacing();

        if (ImGui::CollapsingHeader("Appearance")) {
            ImGui::SliderFloat("Part. Size", &_particleSize, 1, 6);
            ImGui::ColorEdit3("Part. Color", glm::value_ptr(_particleColor));
            ImGui::SliderFloat("Spr. Width", &_springWidth, .001f, 1.f);
            ImGui::ColorEdit3("Spr. Color", glm::value_ptr(_springColor));
        }
        ImGui::Spacing();
    }

    Common::CaseRenderResult CaseDrop::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        if (! _stopped){
            _FSMSolver.Solve(_massSpringSystem);
            _FSMSolver.Step(_massSpringSystem);

            for (std::size_t idx = 0; idx < _massSpringSystem.Positions.size(); ++idx)
            {
                glm::vec3 particle_pos = _massSpringSystem.Positions[idx];
                glm::vec3 particle_vel = _massSpringSystem.Velocities[idx];
                glm::vec3 dis          = particle_pos - center;

                float length = glm::length( dis );
                if (length < radius) // Collision happens
                {
                    glm::vec3 new_pos = center + radius * dis / length; 

                    _massSpringSystem.Positions[idx] = new_pos;
                }
            }
        }
        
        _particlesItem.UpdateVertexBuffer("position", Engine::make_span_bytes<glm::vec3>(_massSpringSystem.Positions));
        _springsItem.UpdateVertexBuffer("position", Engine::make_span_bytes<glm::vec3>(_massSpringSystem.Positions));

        _frame.Resize(desiredSize);

        _cameraManager.Update(_camera);

        _program.GetUniforms().SetByName("u_Projection", _camera.GetProjectionMatrix((float(desiredSize.first) / desiredSize.second)));
        _program.GetUniforms().SetByName("u_View"      , _camera.GetViewMatrix());

        gl_using(_frame);
        glEnable(GL_LINE_SMOOTH);
        glPointSize(_particleSize);
        glLineWidth(_springWidth);

        _program.GetUniforms().SetByName("u_Color", _springColor);
        _springsItem.Draw({ _program.Use() });
        _program.GetUniforms().SetByName("u_Color", _particleColor);
        _particlesItem.Draw({ _program.Use() });

        glLineWidth(1.f);
        glPointSize(1.f);
        glDisable(GL_LINE_SMOOTH);

        return Common::CaseRenderResult {
            .Fixed     = false,
            .Flipped   = true,
            .Image     = _frame.GetColorAttachment(),
            .ImageSize = desiredSize,
        };
    }

    void CaseDrop::OnProcessInput(ImVec2 const & pos) {
        _cameraManager.ProcessInput(_camera, pos);
    }

    void CaseDrop::ResetSystem() {
        // _massSpringSystem = { };
        _massSpringSystem.Springs.clear();
        _massSpringSystem.Positions.clear();
        _massSpringSystem.Velocities.clear();
        std::size_t const n = 33;
        float const delta = 2.f / n;
        auto constexpr GetID = [](std::size_t const i, std::size_t const j) { return i * (n + 1) + j; };
        for (std::size_t i = 0; i <= n; i++) {
            for (std::size_t j = 0; j <= n; j++) {
                _massSpringSystem.AddParticle(glm::vec3(i * delta , 1.5f, j * delta - 1.f));
                if (i > 0) _massSpringSystem.AddSpring(GetID(i, j), GetID(i - 1, j));
                // if (i > 1) _massSpringSystem.AddSpring(GetID(i, j), GetID(i - 2, j));
                if (j > 0) _massSpringSystem.AddSpring(GetID(i, j), GetID(i, j - 1));
                // if (j > 1) _massSpringSystem.AddSpring(GetID(i, j), GetID(i, j - 2));
                if (i > 0 && j > 0) _massSpringSystem.AddSpring(GetID(i, j), GetID(i - 1, j - 1));
                if (i > 0 && j < n) _massSpringSystem.AddSpring(GetID(i, j), GetID(i - 1, j + 1));
            }
        }

        std::vector<std::uint32_t> indices;
        for (auto const & spring : _massSpringSystem.Springs) {
            indices.push_back(std::uint32_t(spring.AdjIdx.first));
            indices.push_back(std::uint32_t(spring.AdjIdx.second));
        }
        _springsItem.UpdateElementBuffer(indices);

        _FSMSolver.Reset(_massSpringSystem, 0.008, 10);
    }
}
