#pragma once
#include "AidFunc.hpp"
#include "Labs/1-RigidBody/RigidBody.h"

namespace VCX::Labs::RigidBody {
    using CollisionGeometryPtr_t = std::shared_ptr<fcl::CollisionGeometry<float>>;

    // The first RigidBody is the ground with huge mass and no gravity
    // The second RigidBody is the player control 
    class SpecialRigidBodySystem{
    public:
        SpecialRigidBodySystem(){}
        SpecialRigidBodySystem(std::vector<RigidBody> const& _bodies,
                               std::vector<CollisionGeometryPtr_t> const& _geoptr,
                               std::vector<glm::vec3> const& Init_X,
                               std::vector<glm::quat> const& Init_Q);

        void Collision(); // Operate Collision
        void AddOverallForce(glm::vec3 const& f);
        void Step(const float dt);
        void StepWV(const float dt);
        void StepQX(const float dt);

        void InitCollisionTransformes();
        void Reset();

        void SingleCollision(std::uint32_t);
        void TwoCollision(std::uint32_t, std::uint32_t);

        RigidBody& operator [] (int idx) { return _bodies[idx]; }

        std::span<const std::byte> Mesh_Span() const;


        float c = 0.5; // will be defined later
        float restitution 	= 0.5f;                 // for collision
	    float friction = 0.2f;

    private:
        std::vector<RigidBody> _bodies;
        std::vector<CollisionGeometryPtr_t>       _collisionGeometryPtr;
        std::vector<fcl::Transform3f>             _collisionTransformes;

        std::vector<glm::vec3> init_x;
        std::vector<glm::quat> init_q;
    };
}