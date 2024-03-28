#include "Labs/1-RigidBody/RigidBody.h"

namespace VCX::Labs::RigidBody
{
    mesh::mesh(std::vector<glm::vec3> const& pos, 
             std::vector<std::uint32_t> const& tri_idx, 
             std::vector<std::uint32_t> const& line_idx):
        positions(pos), 
        tri_index(tri_idx),
        line_index(line_idx){}

    std::vector<glm::vec3> mesh::GetMesh(glm::vec3 x, glm::quat q){
        std::vector<glm::vec3> ret_value = positions;
        for (std::uint32_t idx = 0; idx < positions.size(); ++idx){
            ret_value[idx] = q * positions[idx] + x;
        }
        return ret_value;
    }

    std::span<std::byte const> mesh::Span_Bytes(glm::vec3 x, glm::quat q){
        auto poses = GetMesh(x, q);
        auto span_bytes = Engine::make_span_bytes<glm::vec3>(poses);
        return span_bytes;
    }

    RigidBody::RigidBody(mesh* m_ptr, double m, glm::mat3 I_ref):
        mesh_ptr(m_ptr),
        Mass(m), Ibody(I_ref){
            IbodyInv = glm::inverse(Ibody);
            x = glm::vec3(0.f);
            q = glm::quat(1.f, 0.f, 0.f, 0.f);
            v = glm::vec3(0.f);
            w = glm::vec3(0.f);

            force = glm::vec3(0.f);
            torque = glm::vec3(0.f);
        }

    std::span<std::byte const> RigidBody::Mesh_Span() const { return mesh_ptr->Span_Bytes(x, q); }

    void RigidBody::Step(float dt){
        /* Update w and v */
        v += force * dt;
        w += torque * dt;

        /* Update quaternion and x */
        x += v * dt;
        q = glm::normalize( q + glm::quat(0, w * dt / 2.f) * q );

        /* Update Iinv */
        glm::mat3 R = glm::mat3_cast( q );
        Iinv = R * IbodyInv * glm::transpose(R);
    }

    void RigidBody::StepWV(float dt){
        /* Update w and v */
        v += force * dt;
        w += torque * dt;
    }
    
    void RigidBody::StepQX(float dt){
        /* Update quaternion and x */
        x += v * dt;
        q = glm::normalize( q + glm::quat(0, w * dt / 2.f) * q );

        /* Update Iinv */
        glm::mat3 R = glm::mat3_cast( q );
        Iinv = R * IbodyInv * glm::transpose(R);
    }


} // namespace VCX::Labs::RigidBody
