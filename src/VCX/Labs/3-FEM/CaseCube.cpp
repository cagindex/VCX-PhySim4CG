#include "Labs/3-FEM/CaseCube.h"

namespace VCX::Labs::FEM {
    CaseCube::CaseCube():
        _program(
            Engine::GL::UniqueProgram({ Engine::GL::SharedShader("assets/shaders/flat.vert"),
                                        Engine::GL::SharedShader("assets/shaders/flat.frag") })),
        _boxItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Triangles),
        _lineItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Lines),
        _particlesItem(Engine::GL::VertexLayout()
            .Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream , 0), Engine::GL::PrimitiveType::Points),
        _selectedItem(Engine::GL::VertexLayout()
            .Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream , 0), Engine::GL::PrimitiveType::Points) {

        Reset();

        _cameraManager.AutoRotate = false;
        _cameraManager.Save(_camera);
    }

    void CaseCube::OnSetupPropsUI() {
        if (ImGui::CollapsingHeader("Algorithm", ImGuiTreeNodeFlags_DefaultOpen)){
            if (ImGui::Button("Reset System")) Reset();
            ImGui::SameLine();
            if (ImGui::Button(_stopped ? "Start Simulation" : "Stop Simulation")) _stopped = ! _stopped;
            ImGui::RadioButton("StVk", &Neohookean, 0);
            ImGui::SameLine();
            ImGui::RadioButton("Neohookean", &Neohookean, 1);
            ImGui::SliderFloat("Mass", &_tetSystem.mass, 1, 400);
            ImGui::SliderFloat("mu", &_tetSystem.stiffness_1, 1000, 10000);
            ImGui::SliderFloat("lambda", &_tetSystem.stiffness_0, 5000, 30000);
            ImGui::SliderFloat("Gravity", &_tetSystem.gravity_k, .1f, 1.f);
        }
        ImGui::Spacing();
        
        if (ImGui::CollapsingHeader("Appearance")) {
            ImGui::SliderFloat("Point Size", &PointSize, 3.f, 10.f);
            ImGui::SliderInt("Width X", &wx, 1, 8);
            ImGui::SliderInt("Width Y", &wy, 1, 8);
            ImGui::SliderInt("Width Z", &wz, 1, 8);
            ImGui::ColorEdit3("Box Color", glm::value_ptr(_boxColor));
            ImGui::ColorEdit3("Line Color", glm::value_ptr(_lineColor));
            ImGui::ColorEdit3("Point Color", glm::value_ptr(_pointColor));
            ImGui::ColorEdit3("Selected Color", glm::value_ptr(_selectedColor));
        }
    }

    Common::CaseRenderResult CaseCube::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        // apply mouse control first
        if (!_stopped){
            OnProcessMouseControl(_cameraManager.getMouseMove());
            /* Simulate */
            _tetSystem.Update(0.003f, Neohookean);
        }
        // rendering
        _frame.Resize(desiredSize);

        _cameraManager.Update(_camera);
        _program.GetUniforms().SetByName("u_Projection", _camera.GetProjectionMatrix((float(desiredSize.first) / desiredSize.second)));
        _program.GetUniforms().SetByName("u_View", _camera.GetViewMatrix());

        gl_using(_frame);
        glEnable(GL_LINE_SMOOTH);
        glLineWidth(.5f);

        /* Draw */
        const std::vector<glm::vec3> obj_x = _tetSystem.GetX();
        std::vector<glm::vec3>       X     = obj_x;
        for (auto pos : floor) X.emplace_back(pos);

        _boxItem.UpdateVertexBuffer("position", Engine::make_span_bytes<glm::vec3>(obj_x));
        _program.GetUniforms().SetByName("u_Color", _boxColor);
        _boxItem.Draw({ _program.Use() });

        _lineItem.UpdateVertexBuffer("position", Engine::make_span_bytes<glm::vec3>(X));
        _program.GetUniforms().SetByName("u_Color", _lineColor);
        _lineItem.Draw({ _program.Use() });

        _particlesItem.UpdateVertexBuffer("position", Engine::make_span_bytes<glm::vec3>(obj_x));
        _program.GetUniforms().SetByName("u_Color", _pointColor);
        _particlesItem.Draw({ _program.Use() });

        glm::vec3 sp = obj_x[selected_point_idx];
        _selectedItem.UpdateVertexBuffer("position", Engine::make_span_bytes<glm::vec3>(std::vector<glm::vec3>{sp}));
        _program.GetUniforms().SetByName("u_Color", _selectedColor);
        _selectedItem.Draw({ _program.Use() });

        glLineWidth(1.f);
        glPointSize(PointSize);
        glDisable(GL_LINE_SMOOTH);

        return Common::CaseRenderResult {
            .Fixed     = false,
            .Flipped   = true,
            .Image     = _frame.GetColorAttachment(),
            .ImageSize = desiredSize,
        };
    }

    void CaseCube::OnProcessInput(ImVec2 const & pos) {
        _cameraManager.ProcessInput(_camera, pos);
        if (ImGui::IsMouseDown(ImGuiMouseButton_Left) && ImGui::IsKeyDown(ImGuiKey_LeftCtrl)){
            ImVec2 win_size = ImGui::GetWindowSize();
            float w = win_size.x, h = win_size.y;

            const std::vector<glm::vec3> X = _tetSystem.GetX();
            glm::mat4 transformation = _camera.GetTransformationMatrix(w / h);

            float mouse_w = pos.x, mouse_h = pos.y;
            for (std::uint32_t idx = 0; idx < X.size(); ++idx){
                glm::vec3 p   = X[idx];
                glm::vec4 res = transformation * glm::vec4(p, 1.f);

                float pixel_w = res[0] / res[3],
                      pixel_h = res[1] / res[3];

                float img_w = (pixel_w + 1.f) * 0.5f * w;
                float img_h = (1.f - pixel_h) * 0.5f * h;

                float dw = img_w - mouse_w, dh = img_h - mouse_h;
                if ((dw * dw + dh * dh) <= 100){
                    selected_point_idx = idx;
                    break;
                }
            }
        }
    }

    void CaseCube::OnProcessMouseControl(glm::vec3 mouseDelta) {    
        _tetSystem.Force[selected_point_idx] += 5000.f * mouseDelta;
        if (ImGui::IsKeyPressed(ImGuiKey_Space)){
            for (int i = 0; i < _tetSystem.GetX().size(); ++i)
                _tetSystem.X[i].y += 0.2f;
        }
    }

    void CaseCube::Reset() {
        _tetSystem.Clear();
        _tetSystem.InitCube(wx, wy, wz, delta);

        const std::vector<std::uint32_t> tri_index = GetTriIndex(wx, wy, wz);
        const std::vector<std::uint32_t> obj_line_index = GetLineIndex(wx, wy, wz);

        std::uint32_t              offset     = (wx + 1) * (wy + 1) * (wz + 1);
        std::vector<std::uint32_t> line_index = obj_line_index;
        for (auto idx : floor_idx) line_index.emplace_back(idx + offset);

        _lineItem.UpdateElementBuffer(line_index);
        _boxItem.UpdateElementBuffer(tri_index);
    }

    const std::vector<std::uint32_t> CaseCube::GetTriIndex(std::size_t const wx, std::size_t const wy, std::size_t const wz) const {
        std::vector<std::uint32_t> ret;
        ret.reserve(12 * (wy * wz + wy * wx + wx * wz));

        for (int i = 0; i < 12 * (wy * wz + wy * wx + wx * wz); ++i)
            ret.emplace_back(0);

        std::size_t idx = 0;
        for (std::size_t j = 0; j < wy; ++j) {
            for (std::size_t k = 0; k < wz; ++k) {
                ret[idx++] = _tetSystem.GetID(0, j, k);
                ret[idx++] = _tetSystem.GetID(0, j + 1, k + 1);
                ret[idx++] = _tetSystem.GetID(0, j + 1, k);

                ret[idx++] = _tetSystem.GetID(0, j, k);
                ret[idx++] = _tetSystem.GetID(0, j, k + 1);
                ret[idx++] = _tetSystem.GetID(0, j + 1, k + 1);

                ret[idx++] = _tetSystem.GetID(wx, j, k);
                ret[idx++] = _tetSystem.GetID(wx, j + 1, k);
                ret[idx++] = _tetSystem.GetID(wx, j + 1, k + 1);

                ret[idx++] = _tetSystem.GetID(wx, j, k);
                ret[idx++] = _tetSystem.GetID(wx, j + 1, k + 1);
                ret[idx++] = _tetSystem.GetID(wx, j, k + 1);
            }
        }
        for (std::size_t i = 0; i < wx; ++i) {
            for (std::size_t k = 0; k < wz; ++k) {
                ret[idx++] = _tetSystem.GetID(i, 0, k);
                ret[idx++] = _tetSystem.GetID(i + 1, 0, k);
                ret[idx++] = _tetSystem.GetID(i + 1, 0, k + 1);

                ret[idx++] = _tetSystem.GetID(i, 0, k);
                ret[idx++] = _tetSystem.GetID(i + 1, 0, k + 1);
                ret[idx++] = _tetSystem.GetID(i, 0, k + 1);

                ret[idx++] = _tetSystem.GetID(i, wy, k);
                ret[idx++] = _tetSystem.GetID(i + 1, wy, k + 1);
                ret[idx++] = _tetSystem.GetID(i + 1, wy, k);

                ret[idx++] = _tetSystem.GetID(i, wy, k);
                ret[idx++] = _tetSystem.GetID(i, wy, k + 1);
                ret[idx++] = _tetSystem.GetID(i + 1, wy, k + 1);
            }
        }
        for (std::size_t i = 0; i < wx; ++i) {
            for (std::size_t j = 0; j < wy; ++j) {
                ret[idx++] = _tetSystem.GetID(i, j, 0);
                ret[idx++] = _tetSystem.GetID(i + 1, j + 1, 0);
                ret[idx++] = _tetSystem.GetID(i + 1, j, 0);

                ret[idx++] = _tetSystem.GetID(i, j, 0);
                ret[idx++] = _tetSystem.GetID(i, j + 1, 0);
                ret[idx++] = _tetSystem.GetID(i + 1, j + 1, 0);

                ret[idx++] = _tetSystem.GetID(i, j, wz);
                ret[idx++] = _tetSystem.GetID(i + 1, j, wz);
                ret[idx++] = _tetSystem.GetID(i + 1, j + 1, wz);

                ret[idx++] = _tetSystem.GetID(i, j, wz);
                ret[idx++] = _tetSystem.GetID(i + 1, j + 1, wz);
                ret[idx++] = _tetSystem.GetID(i, j + 1, wz);
            }
        }

        return ret;
    }

    const std::vector<std::uint32_t> CaseCube::GetLineIndex(std::size_t const wx, std::size_t const wy, std::size_t const wz) const {
        std::vector<std::uint32_t> ret;
        ret.reserve(8 * (wx + wy + wz));

        for (int i = 0; i < wx; ++i) {
            ret.emplace_back(_tetSystem.GetID(i, 0, 0));
            ret.emplace_back(_tetSystem.GetID(i + 1, 0, 0));

            ret.emplace_back(_tetSystem.GetID(i, 0, wz));
            ret.emplace_back(_tetSystem.GetID(i + 1, 0, wz));

            ret.emplace_back(_tetSystem.GetID(i, wy, 0));
            ret.emplace_back(_tetSystem.GetID(i + 1, wy, 0));

            ret.emplace_back(_tetSystem.GetID(i, wy, wz));
            ret.emplace_back(_tetSystem.GetID(i + 1, wy, wz));
        }
        for (int j = 0; j < wy; ++j) {
            ret.emplace_back(_tetSystem.GetID(0, j, 0));
            ret.emplace_back(_tetSystem.GetID(0, j + 1, 0));

            ret.emplace_back(_tetSystem.GetID(wx, j, 0));
            ret.emplace_back(_tetSystem.GetID(wx, j + 1, 0));

            ret.emplace_back(_tetSystem.GetID(0, j, wz));
            ret.emplace_back(_tetSystem.GetID(0, j + 1, wz));

            ret.emplace_back(_tetSystem.GetID(wx, j, wz));
            ret.emplace_back(_tetSystem.GetID(wx, j + 1, wz));
        }
        for (int k = 0; k < wz; ++k) {
            ret.emplace_back(_tetSystem.GetID(0, 0, k));
            ret.emplace_back(_tetSystem.GetID(0, 0, k + 1));

            ret.emplace_back(_tetSystem.GetID(wx, 0, k));
            ret.emplace_back(_tetSystem.GetID(wx, 0, k + 1));

            ret.emplace_back(_tetSystem.GetID(0, wy, k));
            ret.emplace_back(_tetSystem.GetID(0, wy, k + 1));

            ret.emplace_back(_tetSystem.GetID(wx, wy, k));
            ret.emplace_back(_tetSystem.GetID(wx, wy, k + 1));
        }

        return ret;
    }
} // namespace VCX::Labs::FEM