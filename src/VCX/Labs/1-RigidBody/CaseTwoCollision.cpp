#include "Engine/app.h"
#include "Labs/1-RigidBody/CaseTwoCollision.h"

namespace VCX::Labs::RigidBody {
    CaseTwoCollision::CaseTwoCollision():
        _program(
            Engine::GL::UniqueProgram({ Engine::GL::SharedShader("assets/shaders/flat.vert"),
                                        Engine::GL::SharedShader("assets/shaders/flat.frag") })),
        _boxItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Triangles),
        _lineItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Lines)
        {
            std::vector<std::uint32_t> line_index;
            std::vector<std::uint32_t> tri_index;
            
            for (auto index : line_index1) line_index.push_back(index);
            for (auto index : tri_index1)  tri_index.push_back(index);
            for (auto index : line_index2) line_index.push_back( index+position1.size() ); 
            for (auto index : tri_index2)  tri_index.push_back( index+position1.size() ); 

            _lineItem.UpdateElementBuffer(line_index);
            _boxItem.UpdateElementBuffer(tri_index);


            /* Construct RigidBody */
            _mesh[0] = mesh(position1, tri_index1, line_index1);
            _rigidbody[0] = RigidBody(_mesh+0, mass[0], Ibody1);
            _mesh[1] = mesh(position2, tri_index2, line_index2);
            _rigidbody[1] = RigidBody(_mesh+1, mass[1], Ibody2);

            /* Init Status */
            for (int i = 0; i < 2; ++i){
                w[0][i] = glm::vec3(0.f, 0.f, 0.f);
                w[1][i] = glm::vec3(0.f, 0.f, 0.f);
                w[2][i] = glm::vec3(0.f, 0.f, 0.f);}
            
            x[0][0] = {2.f, 0.f, 2.f};
            x[0][1] = {-2.f, 0.f, -2.f};
            v[0][0] = {-0.5f, 0.f, -0.5f};
            v[0][1] = {0.5f, 0.f, 0.5f};
            q[0][0] = {0.866f, 0.354f, 0.f, 0.354f};
            q[0][1] = {0.866f, -0.354f, 0.f, -0.354f};

            x[1][0] = {2.f, 0.f, 0.f};
            x[1][1] = {-2.f, 0.f, 0.f};
            v[1][0] = {-0.5f, 0.f, 0.f};
            v[1][1] = {0.5f, 0.f, 0.f};
            q[1][0] = glm::normalize(glm::quat(0.2f, 0.1f, 0.3f, 0.3f));
            q[1][1] = glm::quat(0.966f, 0.f, 0.259f, 0.f);
            
            x[2][0] = {0.f, 0.f, 2.f};
            x[2][1] = {0.f, 0.f, -2.f};
            v[2][0] = {0.f, 0.f, -0.5f};
            v[2][1] = {0.f, 0.f, 0.5f};
            q[2][0] = {1.f, 0.f, 0.f, 0.f};
            q[2][1] = {1.f, 0.f, 0.f, 0.f};

            Reset();
            

        _cameraManager.AutoRotate = false;
        _cameraManager.Save(_camera);
    }

    void CaseTwoCollision::Render(
                Engine::GL::UniqueIndexedRenderItem& item, 
                glm::vec3 const& color, 
                std::vector<glm::vec3> const& poses){
        auto span_bytes = Engine::make_span_bytes<glm::vec3>(poses);

        _program.GetUniforms().SetByName("u_Color", color);
        item.UpdateVertexBuffer("position", span_bytes);
        item.Draw({ _program.Use() });
    }

    void CaseTwoCollision::Reset(){
        for( int i = 0; i < 2; ++i ){
            _rigidbody[i].SetX(x[combo_index][i]);
            _rigidbody[i].SetQ(q[combo_index][i]);
            _rigidbody[i].SetV(v[combo_index][i]);
            _rigidbody[i].SetW(w[combo_index][i]);
            _rigidbody[i].SetForce({0.f, 0.f, 0.f});
            _rigidbody[i].SetTorque({0.f, 0.f, 0.f});
        }
    }

    void CaseTwoCollision::Collision(){
        auto &item_a = _rigidbody[0], &item_b = _rigidbody[1];
        /* Construct Collision Geometry box */
        using CollisionGeometryPtr_t = std::shared_ptr<fcl::CollisionGeometry<float>>;
        CollisionGeometryPtr_t box_geometry_A(new fcl::Box<float>(dim1[0], dim1[1], dim1[2]));
        CollisionGeometryPtr_t box_geometry_B(new fcl::Box<float>(dim2[0], dim2[1], dim2[2]));

        /* Construct Transform3f */
        glm::vec3 xa = item_a.GetX(),
                  xb = item_b.GetX();
        glm::quat qa = item_a.GetQ(),
                  qb = item_b.GetQ();
        fcl::Transform3f Transform_A = fcl::Transform3f(
            Eigen::Translation3f(xa.x, xa.y, xa.z) * 
            Eigen::Quaternion(qa.w, qa.x, qa.y, qa.z)
        );
        fcl::Transform3f Transform_B = fcl::Transform3f(
            Eigen::Translation3f(xb.x, xb.y, xb.z) * 
            Eigen::Quaternion(qb.w, qb.x, qb.y, qb.z)
        );

        /* Construct Collision Object */
        fcl::CollisionObject<float> box_A(box_geometry_A, Transform_A);
        fcl::CollisionObject<float> box_B(box_geometry_B, Transform_B);

        /* Define Collision request and result */
        fcl::CollisionRequest<float> collisionRequest(8, true);
        fcl::CollisionResult<float>  collisionResult;

        /* Collide */
        fcl::collide(&box_B, &box_A, collisionRequest, collisionResult);

        if (!collisionResult.isCollision()) return;
        std::vector<fcl::Contact<float>> contacts;
        collisionResult.getContacts(contacts);
        /* Collision handle */ 
        glm::vec3 n   = {0.f, 0.f, 0.f},
                  x_a = {0.f, 0.f, 0.f},
                  x_b = {0.f, 0.f, 0.f};
        for (auto _contact : contacts){
            // Get N
            n   += flc2glm(_contact.normal);
            // Get X
            x_a += flc2glm(_contact.pos);
            x_b += flc2glm(_contact.pos);
        }
        n   /= contacts.size();
        x_a /= contacts.size();
        x_b /= contacts.size();

        // Get Mass
        float m_a = item_a.GetM(),
              m_b = item_b.GetM();
        // Get Iinv
        glm::mat3 Iinv_a = item_a.GetIinv(),
                  Iinv_b = item_b.GetIinv();
        // Update x_a
        x_a = x_a - item_a.GetX();
        x_b = x_b - item_b.GetX();
        // calculate v
        glm::vec3 v_a   = item_a.GetV() + glm::cross(item_a.GetW(), x_a),
                  v_b   = item_b.GetV() + glm::cross(item_b.GetW(), x_b);
        float     v_rel = glm::dot(n, v_a - v_b);
        if (v_rel >= 0) return; // if they seperate return;
        
        glm::vec3 OP1     = Iinv_a * (glm::cross(glm::cross(x_a, n), x_a));
        glm::vec3 OP2     = Iinv_b * (glm::cross(glm::cross(x_b, n), x_b));
        float     OP      = glm::dot((OP1 + OP2), n);
        float     Genshin = -(1 + c)*v_rel;
        float     Impact  = (1/m_a + 1/m_b + OP);

        float     J       = Genshin/Impact;

        /* Update v and w */
        glm::vec3 vnew_a = item_a.GetV() + J * n / m_a;
        glm::vec3 vnew_b = item_b.GetV() - J * n / m_b;
        glm::vec3 wnew_a = item_a.GetW() + Iinv_a * glm::cross(x_a, J*n);
        glm::vec3 wnew_b = item_b.GetW() - Iinv_b * glm::cross(x_b, J*n);

        item_a.SetV(vnew_a); item_b.SetV(vnew_b);
        item_a.SetW(wnew_a); item_b.SetW(wnew_b);
    }

    void CaseTwoCollision::OnSetupPropsUI(){
        ImGui::Text("Halo Halo");

        bool ret = ImGui::Button("Reset");
        if (ret) Reset();

        if (pause){
            if (ImGui::Button("Start")) pause = false;
        }
        else{
            if (ImGui::Button("Pause")) pause = true;
        }

        int tmp = combo_index;
        ImGui::Combo("Combo", &combo_index, items, 3);
        if (tmp != combo_index)  Reset();

        ImGui::SliderFloat("Coefficient c", &c, 0.0f, 1.0f);
    }

    Common::CaseRenderResult CaseTwoCollision::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        /* Simulate */
        if (!pause){
            Collision();
            for (auto& rigidbody : _rigidbody){
                rigidbody.Step(Engine::GetDeltaTime());
            }
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
        auto poses1 = _rigidbody[0].Get_Poses();
        auto poses2 = _rigidbody[1].Get_Poses();
        std::vector<glm::vec3> poses;
        for (auto item : poses1) poses.push_back(item);
        for (auto item : poses2) poses.push_back(item);
        Render(_boxItem, _boxColor, poses);
        Render(_lineItem, _lineColor, poses);


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

    void CaseTwoCollision::OnProcessInput(ImVec2 const& pos){
        _cameraManager.ProcessInput(_camera, pos);
    }
}