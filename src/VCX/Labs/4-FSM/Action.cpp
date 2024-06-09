#define GLM_ENABLE_EXPERIMENTAL
#include "Labs/4-FSM/Action.h"
namespace VCX::Labs::FSM
{
    Action::Action(){}

    void Action::Load(Skeleton & skeleton, const float dt)
    {
        // TimeIndex += 1;
        // if (TimeIndex == Frames) Reset();
        // std::uint32_t idx = 0;
        // Yoimiya(skeleton.Root, FrameParams.at(TimeIndex), idx);
        // skeleton.ForwardKinematics();

        TotalTime += dt;
        std::uint32_t frame = TotalTime/FrameTime;
        if (frame >= Frames)
        {
            Reset();
            TotalTime += dt;
            frame = TotalTime/FrameTime;
        }
        for (; TimeIndex < frame; ++TimeIndex)
        {
            std::uint32_t idx = 0;
            Yoimiya(skeleton.Root, FrameParams.at(TimeIndex), idx);
            skeleton.ForwardKinematics();
        } 
    }

    void Action::Reset()
    {
        TimeIndex = 0;
        TotalTime = 0.f;
    }

    void Action::Yoimiya(Joint *ptr, const std::vector<float>& params, std::uint32_t & idx)
    {
        if (idx == 0)
        {
            // Position
            for (std::uint32_t i = 0; i < 3; ++i)
                ptr->LocalOffset[ptr->PositionIdx[i]] = params.at(idx + i);
            idx += 3;
            // Rotation
            glm::mat4 res { 1.0f };
            for (std::uint32_t i = 0; i < 3; ++i)
            {
                glm::vec3 axis = { 0.f, 0.f, 0.f };
                axis[ptr->RotationIdx[i]] = 1.f;
                res *= glm::rotate(glm::radians(params.at(idx+i)), axis);
            }
            ptr->LocalRotation = glm::quat_cast(res);
            idx += 3;
        }
        else if (ptr->Name == EndSiteName)
        {
            return;
        }
        else
        {
            // Rotation
            glm::mat4 res { 1.0f };
            for (std::uint32_t i = 0; i < 3; ++i)
            {
                glm::vec3 axis = { 0.f, 0.f, 0.f };
                axis[ptr->RotationIdx[i]] = 1.f;
                res *= glm::rotate(glm::radians(params.at(idx+i)), axis);
            }
            ptr->LocalRotation = glm::quat_cast(res);
            idx += 3;
        }


        Joint *NextPtr = ptr->ChiPtr;
        while(NextPtr != nullptr)
        {
            Yoimiya(NextPtr, params, idx);
            NextPtr = NextPtr->BroPtr;
        }
    }
}