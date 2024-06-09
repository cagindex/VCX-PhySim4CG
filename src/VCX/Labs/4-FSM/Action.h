#pragma once

#include <vector>
#include <string>
#include "Labs/4-FSM/Skeleton.h"
#include <glm/glm.hpp>
#include <glm/ext/quaternion_float.hpp>
#include <glm/ext.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtx/transform.hpp>

namespace VCX::Labs::FSM
{
    struct Action
    {    
        Action();

        void Load(Skeleton &, const float);
        void Reset();


        std::vector<std::vector<float>>     FrameParams;
        std::uint32_t                       TimeIndex = 0;
        std::uint32_t                       Frames;
        float                               FrameTime;

    private:
        void Yoimiya(Joint*, const std::vector<float> &, std::uint32_t &);

        float                               TotalTime = 0.f;
        const std::string                   EndSiteName = "???";
    };
}