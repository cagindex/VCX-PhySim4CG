#include "Labs/1-RigidBody/CaseMultiCollision.h"

namespace VCX::Labs::RigidBody {
    RigidBody AddCube(int& idx, float mass, float x, float y, float z, 
            std::vector<Engine::GL::UniqueIndexedRenderItem>& _boxItems, 
            std::vector<Engine::GL::UniqueIndexedRenderItem>& _lineItems){
        x /= 2; y /= 2; z /= 2;
        const std::vector<glm::vec3>    positions = {
            {-x,  y,  z},
            { x,  y,  z},
            { x, -y,  z},
            {-x, -y,  z},
            {-x,  y, -z},
            { x,  y, -z},
            { x, -y, -z},
            {-x, -y, -z},   
        };
        //     3-----2
        //    /|    /|
        //   0 --- 1 |
        //   | 7 - | 6
        //   |/    |/
        //   4 --- 5
        const std::vector<std::uint32_t> line_index = { 0, 1, 1, 2, 2, 3, 3, 0, 4, 5, 5, 6, 6, 7, 7, 4, 0, 4, 1, 5, 2, 6, 3, 7 }; // line index
        const std::vector<std::uint32_t> tri_index  = { 0, 1, 2, 0, 2, 3, 1, 4, 0, 1, 4, 5, 1, 6, 5, 1, 2, 6, 2, 3, 7, 2, 6, 7, 0, 3, 7, 0, 4, 7, 4, 5, 6, 4, 6, 7 };

        /* Update render part */
        idx = _boxItems.size();
        _boxItems.push_back({ Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Triangles });
        _lineItems.push_back({ Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Lines });

        _boxItems[idx].UpdateElementBuffer(tri_index);
        _lineItems[idx].UpdateElementBuffer(line_index);

        /* Get I */
        glm::mat3 Ibody = {
            { mass * (y*y + z*z) / 3.0, 0.f, 0.f },
            { 0.f, mass * (x*x + z*z) / 3.0, 0.f },
            { 0.f, 0.f, mass * (x*x + y*y) / 3.0 }
        };

        /* Construct the RigidBody */
        return RigidBody(
            new mesh(positions, tri_index, line_index),
            mass,
            Ibody
        );
    }

    CaseMultiCollision::CaseMultiCollision():
        _program(
            Engine::GL::UniqueProgram({ Engine::GL::SharedShader("assets/shaders/flat.vert"),
                                        Engine::GL::SharedShader("assets/shaders/flat.frag") }))
        {
        _cameraManager.AutoRotate = false;
        _cameraManager.Save(_camera);

        /* Construct RigidBody sys*/
        int idx = 0;
        std::vector<RigidBody> bodies;
        std::vector<CollisionGeometryPtr_t> _geoptr;
        for (int i = 0; i < _masses.size(); ++i){
            glm::vec3 cube = _cubes[i];
            auto item = AddCube(idx, _masses[i], cube.x, cube.y, cube.z, _boxItems, _lineItems);
            bodies.push_back( item );
            _geoptr.push_back( std::make_shared<fcl::Box<float>>(cube.x, cube.y, cube.z) );
        }

        _rigidBodySys = RigidBodySys(bodies, _geoptr, _X, _Q);
        Reset();
    }

    void CaseMultiCollision::Render(   
            Engine::GL::UniqueIndexedRenderItem& item,
            glm::vec3 const& color,
            std::span<std::byte const> const& span_bytes){
        _program.GetUniforms().SetByName("u_Color", color);
        item.UpdateVertexBuffer("position", span_bytes);
        item.Draw({ _program.Use() });
    }

    void CaseMultiCollision::Render(
            std::uint32_t idx, 
            glm::vec3 const& boxColor, 
            glm::vec3 const& lineColor){
        auto span_bytes = _rigidBodySys[idx].Mesh_Span();
        Render(_boxItems[idx], boxColor, span_bytes);
        Render(_lineItems[idx], lineColor, span_bytes);
    }

    void CaseMultiCollision::RenderAll(){
        for(std::uint32_t idx = 0; idx < _boxItems.size(); ++idx)
            Render(idx, _boxColor, _lineColor);
    }
    
    void CaseMultiCollision::Reset(){
        _rigidBodySys.Reset();
        _rigidBodySys.AddOverallForce(gravity);
    }

    void CaseMultiCollision::OnSetupPropsUI(){
        ImGui::Text("Halo Halo");
        if (ImGui::Button("Reset")) Reset();
    }

    Common::CaseRenderResult CaseMultiCollision::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        /* Simulate */
        _rigidBodySys.Collision();
        _rigidBodySys.Step(Engine::GetDeltaTime());
        // rendering
        _frame.Resize(desiredSize);

        _cameraManager.Update(_camera);
        _program.GetUniforms().SetByName("u_Projection", _camera.GetProjectionMatrix((float(desiredSize.first) / desiredSize.second)));
        _program.GetUniforms().SetByName("u_View", _camera.GetViewMatrix());

        gl_using(_frame);
        glEnable(GL_LINE_SMOOTH);
        glLineWidth(.5f);

        /* Draw */
        RenderAll();

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

    void CaseMultiCollision::HandleKey(ImGuiKey key, bool& t, glm::vec3 const& f){
        if (ImGui::IsKeyDown(key) && (!t)){
            _rigidBodySys[1].AddForce(f);
            t = true;
        }
        if (!ImGui::IsKeyDown(key) && (t)){ 
            _rigidBodySys[1].AddForce(-f);
            t = false; 
        }
    }

    void CaseMultiCollision::OnProcessInput(ImVec2 const& pos){
        static bool w = false, s = false, a = false, d = false;
        _cameraManager.ProcessInput(_camera, pos);

        HandleKey(ImGuiKey_W, w, drag_force_x);
        HandleKey(ImGuiKey_S, s, -drag_force_x);
        HandleKey(ImGuiKey_A, a, -drag_force_z);
        HandleKey(ImGuiKey_D, d, drag_force_z);
    }
}