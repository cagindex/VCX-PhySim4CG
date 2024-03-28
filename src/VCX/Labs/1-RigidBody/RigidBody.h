#pragma once

#include <fcl/narrowphase/collision.h>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "Engine/GL/Frame.hpp"
#include "Engine/GL/Program.h"
#include "Engine/GL/RenderItem.h"
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <vector>

namespace VCX::Labs::RigidBody {
    class mesh {
    public:
        mesh(){}
        mesh(std::vector<glm::vec3> const& pos, 
             std::vector<std::uint32_t> const& tri_idx, 
             std::vector<std::uint32_t> const& line_idx);

        /* Get positions */
        std::vector<glm::vec3> GetMesh(glm::vec3 x, glm::quat q);
        /* To Bytes */
        std::span<std::byte const> Span_Bytes(glm::vec3 x, glm::quat q);


        std::vector<glm::vec3> positions;
        std::vector<std::uint32_t> tri_index;
        std::vector<std::uint32_t> line_index;
    };

	class RigidBody {
    public:
        RigidBody(){}
        RigidBody(mesh* m_ptr, double m, glm::mat3 I_ref);

        void SetX(glm::vec3 new_x){x = new_x;};
        void SetQ(glm::quat new_q){q = new_q; glm::mat3 R = glm::mat3_cast( q ); Iinv = R * IbodyInv * glm::transpose(R);};
        void SetV(glm::vec3 new_v){v = new_v;};
        void SetW(glm::vec3 new_w){w = new_w;};

        void SetForce(glm::vec3 f){force = f;};
        void AddForce(glm::vec3 f){force += f;};
        void SetTorque(glm::vec3 tor){torque = tor;};
        void AddTorque(glm::vec3 tor){torque += tor;};

        float     GetM(){ return Mass; }
        glm::vec3 GetX(){ return x; }
        glm::quat GetQ(){ return q; }
        glm::vec3 GetV(){ return v; }
        glm::vec3 GetW(){ return w; }
        glm::mat3 GetI(){ return glm::inverse(Iinv); }
        glm::mat3 GetIinv(){ return Iinv; }

        void Step(float dt);
        void StepWV(float dt);
        void StepQX(float dt);

        std::span<std::byte const> Mesh_Span() const;

    // private:
        /* Constant quantities */
        float     Mass; 
        glm::mat3 Ibody, 
                  IbodyInv; 

        /* State quantities */
        glm::vec3 x = {0.f, 0.f, 0.f}; 
        glm::quat q = {1.f, 0.f, 0.f, 0.f}; 

        /* Derived quantities */
        glm::mat3 Iinv; /* Inv of Interia */
        glm::vec3 v = {0.f, 0.f, 0.f}, /* velocity */
                  w = {0.f, 0.f, 0.f}; /* Angular velocity */

        /* Computed quatities */
        glm::vec3 force  = {0.f, 0.f, 0.f},
                  torque = {0.f, 0.f, 0.f};

        mesh* mesh_ptr = nullptr;
	};
}