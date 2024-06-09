#pragma once

#include <vector>
#include <string>
#include <memory>
#include <glm/glm.hpp>
#include <glm/ext/quaternion_float.hpp>
#include <glm/ext.hpp>
#include <glm/gtx/quaternion.hpp>

namespace VCX::Labs::FSM
{
    struct Joint
    {
        Joint();

        Joint *BroPtr = nullptr, *ChiPtr = nullptr;

        int             PositionIdx[3] = { 0, 1, 2 };
        int             RotationIdx[3] = { 2, 0, 1 };
        std::string     Name;
        glm::vec3       LocalOffset    = { 0.f, 0.f, 0.f };
        glm::vec3       GlobalPosition = { 0.f, 0.f, 0.f };
        glm::quat       LocalRotation  = glm::quat_cast(glm::mat4{ 1.f });
        glm::quat       GlobalRotation = glm::quat_cast(glm::mat4{ 1.f });
    };

    struct Skeleton
    {
        Skeleton();

        std::pair<std::vector<glm::vec3>, std::vector<std::uint32_t>> Convert() const;
        void                                                          ForwardKinematics();


        Joint *Root;
    private:
        void Construct(const Joint *, std::vector<glm::vec3> &, std::vector<std::uint32_t> &) const;
        void ItsMyGo(Joint* ptr); // Inner Forward Kinematics
        void Adjust(Joint* ptr);

        float                                                         Scale = 0.1f;
        glm::vec3                                                     Offset = { 0.f, 0.15f, 0.f };
    };
}