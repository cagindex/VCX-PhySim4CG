#include <cmath>

#include "Engine/app.h"
#include "Labs/1-RigidBody/CaseSpinningCube.h"

namespace VCX::Labs::RigidBody {
    CaseSpinningCube::CaseSpinningCube():
        _program(
            Engine::GL::UniqueProgram({ Engine::GL::SharedShader("assets/shaders/flat.vert"),
                                        Engine::GL::SharedShader("assets/shaders/flat.frag") })),
        _boxItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Triangles),
        _lineItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Lines),
        
        _mesh(positions, tri_index, line_index),
        _rigidbody(&_mesh, mass, Ibody) {
        
        _lineItem.UpdateElementBuffer(line_index);
        _boxItem.UpdateElementBuffer(tri_index);

        _cameraManager.AutoRotate = false;
        _cameraManager.Save(_camera);

        _rigidbody.SetW(glm::vec3(0.f, omega, 0.f));
    }

    void CaseSpinningCube::OnSetupPropsUI(){
        ImGui::Text("Halo Halo");
        ImGui::SliderFloat("omega", &omega, 0.0f, 7.0f);
    }

    Common::CaseRenderResult CaseSpinningCube::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        // apply mouse control first
        OnProcessMouseControl(_cameraManager.getMouseMove());

        /* Simulate */
        _rigidbody.SetW(glm::vec3(0.f, omega, 0.f));
        _rigidbody.Step(Engine::GetDeltaTime());
        time += Engine::GetDeltaTime();
        // rendering
        _frame.Resize(desiredSize);

        _cameraManager.Update(_camera);
        _program.GetUniforms().SetByName("u_Projection", _camera.GetProjectionMatrix((float(desiredSize.first) / desiredSize.second)));
        _program.GetUniforms().SetByName("u_View", _camera.GetViewMatrix());

        gl_using(_frame);
        glEnable(GL_LINE_SMOOTH);
        glLineWidth(.5f);

        /* Draw */
        auto span_bytes = _rigidbody.Mesh_Span();
        _program.GetUniforms().SetByName("u_Color", _boxColor);
        _boxItem.UpdateVertexBuffer("position", span_bytes);
        _boxItem.Draw({ _program.Use() });

        _program.GetUniforms().SetByName("u_Color", glm::vec3(1.f, 1.f, 1.f));
        _lineItem.UpdateVertexBuffer("position", span_bytes);
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

    void CaseSpinningCube::OnProcessInput(ImVec2 const& pos){
        _cameraManager.ProcessInput(_camera, pos);
    }
    
    void CaseSpinningCube::OnProcessMouseControl(glm::vec3 mouseDelta) {
        float movingScale = 0.01f;
        _rigidbody.AddForce( mouseDelta * movingScale );
    }
}