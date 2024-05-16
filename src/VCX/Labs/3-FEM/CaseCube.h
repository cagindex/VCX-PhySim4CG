#pragma once

#include "Engine/app.h"
#include "Engine/Async.hpp"
#include "Engine/GL/Frame.hpp"
#include "Engine/GL/Program.h"
#include "Engine/GL/RenderItem.h"

#include "Labs/Common/ICase.h"
#include "Labs/Common/ImageRGB.h"
#include "Labs/Common/OrbitCameraManager.h"

#include "Labs/3-FEM/TetSystem.h"

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/string_cast.hpp>
#include <vector>



namespace VCX::Labs::FEM {

    class CaseCube : public Common::ICase {
    public:
        CaseCube();

        virtual std::string_view const GetName() override { return "FEM Cube"; }
        virtual void                     OnSetupPropsUI() override;
        virtual Common::CaseRenderResult OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) override;
        virtual void                     OnProcessInput(ImVec2 const & pos) override;

        void                               Reset();
        void      OnProcessMouseControl(glm::vec3 mourseDelta);
        const std::vector<std::uint32_t>   GetTriIndex(std::size_t const wx, std::size_t const wy, std::size_t const wz) const;
        const std::vector<std::uint32_t>   GetLineIndex(std::size_t const wx, std::size_t const wy, std::size_t const wz) const;

    private:
        Engine::GL::UniqueProgram     _program;
        Engine::GL::UniqueRenderFrame _frame;
        Engine::Camera                _camera { .Eye = glm::vec3(-3, 3, 3) };
        Common::OrbitCameraManager    _cameraManager;

        Engine::GL::UniqueRenderItem        _particlesItem;
        Engine::GL::UniqueRenderItem        _selectedItem;
        Engine::GL::UniqueIndexedRenderItem _boxItem;
        Engine::GL::UniqueIndexedRenderItem _lineItem;

        /* Render Color Part */
        glm::vec3 _boxColor      { 121.0f / 255, 207.0f / 255, 171.0f / 255 };
        glm::vec3 _lineColor     { 1.f, 1.f, 1.f };
        glm::vec3 _pointColor    { 1.f, 0.f, 0.f };
        glm::vec3 _selectedColor { 0.f, 0.f, 1.f };


        /* Floor */
        const std::vector<glm::vec3> floor = {
            { 2.0f, -1.0f, 0.0f },
            { 0.0f, -1.0f, 2.0f },
            { -2.0f, -1.0f, 0.0f },
            { 0.0f, -1.0f, -2.0f },
        };

        const std::vector<std::uint32_t> floor_idx = { 0, 1, 1, 2, 2, 3, 3, 0, 0, 2, 1, 3}; // line index

        /* FEM Part */
        int wx = 2, wy = 8, wz = 2;
        float       delta = 1.f;
        TetSystem _tetSystem;

        /* Interaction Part */
        std::uint32_t selected_point_idx = 0;

        /* Side Bar */
        bool _stopped = false;
        float PointSize = 3.f;
        int Neohookean = 0;
    };
} // namespace VCX::Labs::FEM