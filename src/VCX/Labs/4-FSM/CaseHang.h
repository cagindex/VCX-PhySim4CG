#pragma once

#include "Engine/GL/Frame.hpp"
#include "Engine/GL/Program.h"
#include "Engine/GL/RenderItem.h"
#include "Labs/4-FSM/MassSpringSystem.h"
#include "Labs/Common/OrbitCameraManager.h"
#include "Labs/Common/ICase.h"
#include "Labs/Common/ImageRGB.h"

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "Labs/4-FSM/FSMSolver.h"

namespace VCX::Labs::FSM {
    class CaseHang : public Common::ICase {
    public:
        CaseHang();

        virtual std::string_view const GetName() override { return "Case Hang"; }
        
        virtual void OnSetupPropsUI() override;
        virtual Common::CaseRenderResult OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) override;
        virtual void OnProcessInput(ImVec2 const & pos) override;
        void      OnProcessMouseControl(glm::vec3 mourseDelta);
    
    private:
        Engine::GL::UniqueProgram               _program;
        Engine::GL::UniqueRenderFrame           _frame;
        Engine::Camera                          _camera        { .Eye = glm::vec3(-3, 3, 3) };
        Common::OrbitCameraManager              _cameraManager;
        Engine::GL::UniqueRenderItem            _particlesItem;
        Engine::GL::UniqueIndexedRenderItem     _springsItem;
        float                                   _particleSize  { 2 };
        float                                   _springWidth   { 1 };
        glm::vec3                               _particleColor { 1.f, 0.f, 0.f };
        glm::vec3                               _springColor   { 0.f, 0.f, 1.f };
        bool                                    _stopped       { false };

        MassSpringSystem                        _massSpringSystem;
        FSMSolver                               _FSMSolver;

        std::size_t                             _sel_id { 0 };

        void ResetSystem();
    };
}
