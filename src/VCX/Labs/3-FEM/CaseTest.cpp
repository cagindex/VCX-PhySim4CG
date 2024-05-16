#include "Labs/3-FEM/CaseTest.h"
#include <span>

namespace VCX::Labs::FEM {
    CaseTest::CaseTest():
        _program(
            Engine::GL::UniqueProgram({ Engine::GL::SharedShader("assets/shaders/flat.vert"),
                                        Engine::GL::SharedShader("assets/shaders/flat.frag") })),
        _boxItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Triangles),
        _lineItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Lines)
        {
        _lineItem.UpdateElementBuffer(line_index);
        _boxItem.UpdateElementBuffer(tri_index);

        _cameraManager.AutoRotate = false;
        _cameraManager.Save(_camera);

        vec.reserve(4);
        f.reserve(4);
        pos = ref_pos;
        for (int i = 0; i < 4; ++i){ vec[i] = glm::vec3(0.f); f[i] = glm::vec3(0.f); }

        Dm     = BuildEdgeMatrix();
        Dm_inv = glm::inverse(Dm);
    }

    void CaseTest::OnSetupPropsUI(){
        ImGui::Text("Halo Halo");

        bool ret = ImGui::Button("Reset");
        if (ret) Reset();
    }

    Common::CaseRenderResult CaseTest::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        // apply mouse control first
        OnProcessMouseControl(_cameraManager.getMouseMove());

        /* Simulate */
        Simulate(0.003f);
        /*Simulate(Engine::GetDeltaTime());*/
        // rendering
        _frame.Resize(desiredSize);

        _cameraManager.Update(_camera);
        _program.GetUniforms().SetByName("u_Projection", _camera.GetProjectionMatrix((float(desiredSize.first) / desiredSize.second)));
        _program.GetUniforms().SetByName("u_View", _camera.GetViewMatrix());

        gl_using(_frame);
        glEnable(GL_LINE_SMOOTH);
        glLineWidth(.5f);

        /* Draw */
        _program.GetUniforms().SetByName("u_Color", _boxColor);
        _boxItem.UpdateVertexBuffer("position", Engine::make_span_bytes<glm::vec3>(pos));
        _boxItem.Draw({ _program.Use() });

        _program.GetUniforms().SetByName("u_Color", glm::vec3(1.f, 1.f, 1.f));
        _lineItem.UpdateVertexBuffer("position", Engine::make_span_bytes<glm::vec3>(pos));
        _lineItem.Draw({ _program.Use() });

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

    void CaseTest::OnProcessInput(ImVec2 const& pos){
        _cameraManager.ProcessInput(_camera, pos);
    }
    
    void CaseTest::OnProcessMouseControl(glm::vec3 mouseDelta) {
    }

    void CaseTest::Reset(){
        pos = ref_pos;
        for (int i = 0; i < 4; ++i){ vec[i] = glm::vec3(0.f); f[i] = glm::vec3(0.f); }
    }

    glm::mat3 CaseTest::BuildEdgeMatrix(){
        glm::mat3 res { 0.f };
        for (int i = 1; i < 4; ++i)
            res[i-1] = pos[i] - pos[0];
        return res;
    }

    void CaseTest::Smooth_vec() {
        glm::vec3 sum = glm::vec3(0.f);
        for (int i = 0; i < 4; ++i)
            sum += vec[i];
        for (int i = 0; i < 4; ++i)
            vec[i] = 0.9f * vec[i] + 0.1f * sum / 4.f;
    }

    void CaseTest::Simulate(float dt){
        for (int i = 0; i < 4; ++i)
            f[i] = ref_mass[i] * gravity;

        /*float stiffness_0 = Y / (2 * (1 + P));
        float stiffness_1 = Y * P / ((1 + P) * (1 - 2 * P));*/

        float stiffness_0 = 20000;
        float stiffness_1 = 5000;

        glm::mat3 F = BuildEdgeMatrix() * Dm_inv;

        glm::mat3 G = 0.5f * (glm::transpose(F) * F - glm::mat3(1.f));

        glm::mat3 S = 2.f * stiffness_1 * G + stiffness_0 * (G[0][0] + G[1][1] + G[2][2]) * glm::mat3(1.f);

        float     scale  = -1.f / (6.f * glm::determinant(Dm_inv));
        glm::mat3 result = scale * F * S * glm::transpose(Dm_inv);

        f[0] -= (result[0] + result[1] + result[2]);
        f[1] += result[0];
        f[2] += result[1];
        f[3] += result[2];

        /*Smooth_vec();*/
        for (int i = 0; i < 4; ++i){
            vec[i]  = (vec[i] + dt * f[i] / ref_mass[i]);
            pos[i] += dt * vec[i];

            if (pos[i].y < -1.0f) {
                vec[i].x = 0.f;
                vec[i].z = 0.f;
                vec[i].y += (-1.f - pos[i].y) / dt;
                pos[i].y = -1.0f;
            }
        }
    }
}