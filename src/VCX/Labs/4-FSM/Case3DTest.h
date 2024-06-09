#pragma once

#define GLM_ENABLE_EXPERIMENTAL
#include <vector>
#include <string>
#include <glm/glm.hpp>
#include <glm/ext/quaternion_float.hpp>
#include <glm/ext.hpp>
#include <glm/gtx/quaternion.hpp>

#include "Engine/GL/Frame.hpp"
#include "Engine/GL/Program.h"
#include "Engine/GL/RenderItem.h"
#include "Labs/Common/OrbitCameraManager.h"
#include "Labs/Common/ICase.h"
#include "Labs/Common/ImageRGB.h"

#include "Labs/4-FSM/Skeleton.h"
#include "Labs/4-FSM/Action.h"
#include "Labs/4-FSM/BVHLoader.h"

#include "Labs/4-FSM/MassSpringSystem.h"
#include "Labs/4-FSM/FSMSolver.h"

namespace VCX::Labs::FSM
{   
    class BackGroundRender
    {
    public:
        BackGroundRender();
        void render(Engine::GL::UniqueProgram & program);
    
    public:
        Engine::GL::UniqueIndexedRenderItem LineItem;
    };

    class SkeletonRender
    {
    public: 
        SkeletonRender();

        void render(Engine::GL::UniqueProgram & program);
        void load(const Skeleton & skele);
        void loadAll(const Skeleton & skele);
    
    public:
        Engine::GL::UniqueIndexedRenderItem LineItem;
        Engine::GL::UniqueRenderItem        PointItem;
    };

    class Case3DTest : public Common::ICase 
    {
    public:
        Case3DTest();

        virtual std::string_view const GetName() override { return "Case 3D Test"; }

        virtual void OnSetupPropsUI() override;
        virtual Common::CaseRenderResult OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) override;
        virtual void OnProcessInput(ImVec2 const & pos) override;

        void ResetSystem();
    
    private:
        Engine::GL::UniqueProgram               _program;
        Engine::GL::UniqueRenderFrame           _frame;
        Engine::Camera                          _camera { .Eye = glm::vec3(-3, 3, 3) };
        Common::OrbitCameraManager              _cameraManager;
        bool                                    _stopped       { false };

        BackGroundRender                        BackGround;
        SkeletonRender                          skeletonRender;

        Engine::GL::UniqueRenderItem            _particlesItem;
        Engine::GL::UniqueIndexedRenderItem     _springsItem;


        // self defined object 
        std::string                             _filePath = "assets/BVHs/test5.bvh";
        Skeleton                                _skeleton;
        Action                                  _action;
        BVHLoader                               _BVHLoader;

        // Mass Spring system
        MassSpringSystem                        _massSpringSystem;
        FSMSolver                               _FSMSolver;
    };
}