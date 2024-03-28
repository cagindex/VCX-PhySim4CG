#include "Labs/1-RigidBody/SpecialRigidBodySystem.h"

namespace VCX::Labs::RigidBody{
    SpecialRigidBodySystem::SpecialRigidBodySystem(
        std::vector<RigidBody> const& items,
        std::vector<CollisionGeometryPtr_t> const& _geoptr,
        std::vector<glm::vec3> const& Init_X,
        std::vector<glm::quat> const& Init_Q):
        
        _bodies(items),
        _collisionGeometryPtr(_geoptr),
        init_x(Init_X),
        init_q(Init_Q){
            std::uint32_t size = _bodies.size();
            _collisionTransformes.assign(size, fcl::Transform3f());
            
            Reset();
    }

    void SpecialRigidBodySystem::InitCollisionTransformes(){
        for (std::uint32_t idx = 0; idx < _bodies.size(); ++idx){
            auto item = _bodies[idx];

            glm::vec3 x = item.GetX();
            glm::quat q = item.GetQ();

            _collisionTransformes[idx] = fcl::Transform3f(
                Eigen::Translation3f(x.x, x.y, x.z) * 
                Eigen::Quaternion(q.w, q.x, q.y, q.z)
            );
        }
    }

    void SpecialRigidBodySystem::Reset(){
        for(std::uint32_t idx = 0; idx < _bodies.size(); ++idx){
            _bodies[idx].SetX(init_x[idx]);
            _bodies[idx].SetQ(init_q[idx]);
            _bodies[idx].SetW({0.f, 0.f, 0.f});
            _bodies[idx].SetV({0.f, 0.f, 0.f});
            _bodies[idx].SetForce({0.f, 0.f, 0.f});
            _bodies[idx].SetTorque({0.f, 0.f, 0.f});
        }
    }

    void SpecialRigidBodySystem::TwoCollision(std::uint32_t i, std::uint32_t j){
        auto& item_a         = _bodies[i];
        auto& item_b         = _bodies[j];

        auto& box_geometry_A = _collisionGeometryPtr[i];
        auto& box_geometry_B = _collisionGeometryPtr[j];

        auto& Transform_A    = _collisionTransformes[i];
        auto& Transform_B    = _collisionTransformes[j];

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
        // Update x
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

    void SpecialRigidBodySystem::SingleCollision(std::uint32_t j){
        auto& item_a         = _bodies[0];
        auto& item_b         = _bodies[j];

        auto& box_geometry_A = _collisionGeometryPtr[0];
        auto& box_geometry_B = _collisionGeometryPtr[j];

        auto& Transform_A    = _collisionTransformes[0];
        auto& Transform_B    = _collisionTransformes[j];

        /* Construct Collision Object */
        fcl::CollisionObject<float> box_A(box_geometry_A, Transform_A);
        fcl::CollisionObject<float> box_B(box_geometry_B, Transform_B);

        /* Define Collision request and result */
        fcl::CollisionRequest<float> collisionRequest(8, true);
        fcl::CollisionResult<float>  collisionResult;

        /* Collide */
        fcl::collide(&box_A, &box_B, collisionRequest, collisionResult);

        if (!collisionResult.isCollision()) return;
        std::vector<fcl::Contact<float>> contacts;
        collisionResult.getContacts(contacts);

        /* Get X and W and V and Q */
        glm::quat q = item_b.GetQ();
        glm::vec3 x = item_b.GetX(),
                  w = item_b.GetW(),
                  v = item_b.GetV();

        int       t = 0;
        glm::vec3 n = {0.f, 0.f, 0.f},
                  r_i = {0.f, 0.f, 0.f};
        for (auto _contact : contacts){
            glm::vec3 normal = flc2glm(_contact.normal);
            glm::vec3 pos = flc2glm(_contact.pos) - x;
            if (glm::dot(normal, glm::cross(w, pos)+v) < 0.f){
                r_i += pos;
                n   += normal;
                t   += 1;
            }
        }

        if (t == 0) return;
        r_i /= t;
        n   /= t;

        /* Update v */
        glm::vec3 v_i = glm::cross(w, r_i) + v;
        glm::vec3 v_n = glm::dot(n, v_i) * n;
        glm::vec3 v_t = v_i - v_n;

        float a = glm::max(0.f, 1 - friction * (1 + restitution) * glm::length(v_n) / glm::length(v_t));
        v_n = - restitution * v_n;
        v_t = a * v_t;

        glm::vec3 v_new = v_t + v_n;

        /* Update K */
        glm::mat3 Iinv   = item_b.GetIinv();
        glm::mat3 r_star = Get_Cross_Matrix(r_i); 
        glm::mat3 I      = r_star * Iinv * r_star;

        float mass = item_b.GetM();

        glm::mat3 K      = glm::mat3({1.f/mass}) - I;

        /* Get J */
        glm::vec3 J      = glm::inverse(K) * (v_new - v_i);

        glm::vec3 vnew = v + J/mass;
        glm::vec3 wnew = w + Iinv * glm::cross(r_i, J);

        item_b.SetV(vnew); item_b.SetW(wnew);
    }

    void SpecialRigidBodySystem::Collision(){
        InitCollisionTransformes();

        std::uint32_t i, j;
        std::uint32_t size = _bodies.size();
        for (j = 1; j < size; ++j){
            SingleCollision(j); // Collision with ground i.e idx with 0
        }
        for (i = 1; i < size; ++i){
            for (j = i+1; j < size; ++j){
                TwoCollision(i, j); // Collision with two object idx with i and j.
            }
        }

        glm::vec3 n = {0.f, 0.f, 0.f},
                  x = {0.f, 0.f, 0.f};
    }

    void SpecialRigidBodySystem::AddOverallForce(glm::vec3 const& f){
        for (int idx = 1; idx < _bodies.size(); ++idx)
            _bodies[idx].AddForce(f);
    }

    void SpecialRigidBodySystem::Step(const float dt){
        for (auto& item : _bodies) item.Step(dt);
    }
    void SpecialRigidBodySystem::StepWV(const float dt){
        for (auto& item : _bodies) item.StepWV(dt);
    }
    void SpecialRigidBodySystem::StepQX(const float dt){
        for (auto& item : _bodies) item.StepQX(dt);
    }
}