#include "Engine/app.h"
#include "Labs/4-FSM/CaseHang.h"
#include "Labs/Common/ImGuiHelper.h"
#include "Labs/4-FSM/utils.hpp"

#include <iostream>

namespace VCX::Labs::FSM {
    CaseHang::CaseHang() :
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

    void CaseHang::OnSetupPropsUI() {
        ImGui::Text("Selected Point %u", _sel_id);
        ImGui::Spacing();
        
        if (ImGui::CollapsingHeader("Algorithm", ImGuiTreeNodeFlags_DefaultOpen)) {
            if (ImGui::Button("Reset System")) ResetSystem();
            ImGui::SameLine();
            if (ImGui::Button(_stopped ? "Start Simulation" : "Stop Simulation")) _stopped = ! _stopped;
            ImGui::SliderFloat("Total. Mass", &_massSpringSystem.TotalMass, .5f, 10.f);
            ImGui::SliderFloat("Spr. Stiff.", &_massSpringSystem.Stiffness, 1.f, 100.f);
            ImGui::SliderFloat("Spr. Damp.", &_massSpringSystem.Damping, 0.8f, 1.f);
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

    Common::CaseRenderResult CaseHang::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        if (! _stopped){
            OnProcessMouseControl(_cameraManager.getMouseMove());
            _FSMSolver.Solve(_massSpringSystem);
            SpringConstraint(_massSpringSystem, 0.4f, 15);
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

    void CaseHang::OnProcessInput(ImVec2 const & pos) {
        _cameraManager.ProcessInput(_camera, pos);
        if (ImGui::IsMouseDown(ImGuiMouseButton_Left) && ImGui::IsKeyDown(ImGuiKey_LeftCtrl)){
            ImVec2 win_size = ImGui::GetWindowSize();
            float w = win_size.x, h = win_size.y;

            glm::mat4 transformation = _camera.GetTransformationMatrix(w / h);

            float mouse_w = pos.x, mouse_h = pos.y;
            for (std::uint32_t idx = 0; idx < _massSpringSystem.Positions.size(); ++idx){
                glm::vec3 p   = _massSpringSystem.Positions[idx];
                glm::vec4 res = transformation * glm::vec4(p, 1.f);

                float pixel_w = res[0] / res[3],
                      pixel_h = res[1] / res[3];

                float img_w = (pixel_w + 1.f) * 0.5f * w;
                float img_h = (1.f - pixel_h) * 0.5f * h;

                float dw = img_w - mouse_w, dh = img_h - mouse_h;
                if ((dw * dw + dh * dh) <= 100){
                    _massSpringSystem.Fixed[_sel_id] = false;
                    _sel_id = idx;
                    break;
                }
            }
        }
    }

    void CaseHang::OnProcessMouseControl(glm::vec3 mouseDelta) {   
        if (ImGui::IsMouseDown(ImGuiMouseButton_Left)){
            _massSpringSystem.Fixed[_sel_id] = true;
            _massSpringSystem.Positions[_sel_id] += .1f * mouseDelta;
        }
        else{
            _massSpringSystem.Fixed[_sel_id] = false;
        }
    }

    void CaseHang::ResetSystem() {
        // _massSpringSystem = { };
        _massSpringSystem.Springs.clear();
        _massSpringSystem.Positions.clear();
        _massSpringSystem.Velocities.clear();
        _massSpringSystem.Fixed.clear();
        _massSpringSystem.Forces.clear();
        _massSpringSystem.Mass.clear();
        std::size_t const n = 33;
        float const delta = 2.f / n;
        auto constexpr GetID = [](std::size_t const i, std::size_t const j) { return i * (n + 1) + j; };
        for (std::size_t i = 0; i <= n; i++) {
            for (std::size_t j = 0; j <= n; j++) {
                _massSpringSystem.AddParticle(glm::vec3(i * delta , 1.5f, j * delta - 1.f), _massSpringSystem.TotalMass/(int(n)*int(n)));
                if (i > 0) _massSpringSystem.AddSpring(GetID(i, j), GetID(i - 1, j));
                if (i > 1) _massSpringSystem.AddSpring(GetID(i, j), GetID(i - 2, j));
                if (j > 0) _massSpringSystem.AddSpring(GetID(i, j), GetID(i, j - 1));
                if (j > 1) _massSpringSystem.AddSpring(GetID(i, j), GetID(i, j - 2));
                if (i > 0 && j > 0) _massSpringSystem.AddSpring(GetID(i, j), GetID(i - 1, j - 1));
                if (i > 0 && j < n) _massSpringSystem.AddSpring(GetID(i, j), GetID(i - 1, j + 1));
            }
        }
        // _massSpringSystem.Fixed[GetID(0, 0)] = true;
        // _massSpringSystem.Fixed[GetID(0, n)] = true;

        _massSpringSystem.Fixed[GetID(n, 0)] = true;
        _massSpringSystem.Fixed[GetID(n, n)] = true;

        // _massSpringSystem.Fixed[GetID(int(n/2), int(n/2))] = true;

        std::vector<std::uint32_t> indices;
        for (auto const & spring : _massSpringSystem.Springs) {
            indices.push_back(std::uint32_t(spring.AdjIdx.first));
            indices.push_back(std::uint32_t(spring.AdjIdx.second));
        }
        _springsItem.UpdateElementBuffer(indices);

        _FSMSolver.Reset(_massSpringSystem, 0.008, 1);
    }
}
