#pragma once

#include "Engine/app.h"
#include "Engine/Async.hpp"
#include "Engine/GL/Frame.hpp"
#include "Engine/GL/Program.h"
#include "Engine/GL/RenderItem.h"

#include "Labs/Common/ICase.h"
#include "Labs/Common/ImageRGB.h"
#include "Labs/Common/OrbitCameraManager.h"

#include "Labs/1-RigidBody/SpecialRigidBodySystem.h"

#include "Labs/1-RigidBody/MyOrbitCameraManager.hpp"

namespace VCX::Labs::RigidBody {
    using RigidBodySys = SpecialRigidBodySystem;

	class CaseMultiCollision : public Common::ICase {
    public:
        CaseMultiCollision();

		virtual std::string_view const GetName() override { return "Spinning Cube"; }

		virtual void					 OnSetupPropsUI() override;
        virtual Common::CaseRenderResult OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) override;
        virtual void                     OnProcessInput(ImVec2 const & pos) override;

        void                             Render(Engine::GL::UniqueIndexedRenderItem& item, glm::vec3 const& color, std::span<std::byte const> const& span_bytes);
        void                             Render(std::uint32_t , glm::vec3 const&, glm::vec3 const&); // Render and upload vertex positions
        void                             RenderAll();

        void                             Reset();
        void                             Reconstruct();

        void                             HandleKey(ImGuiKey value, bool& t, glm::vec3 const& f);

	private:
		Engine::GL::UniqueProgram           _program;
        Engine::GL::UniqueRenderFrame       _frame;
        Engine::Camera                      _camera { .Eye = glm::vec3(-3, 3, 3) };
        MyOrbitCameraManager                _cameraManager;

        std::vector<Engine::GL::UniqueIndexedRenderItem> _boxItems;
        std::vector<Engine::GL::UniqueIndexedRenderItem> _lineItems;

		/* cubes List */
        std::vector<float>                  _masses = {
            1000000.f, 1.f
        };
        std::vector<glm::vec3>              _cubes  = {
            {30.f, 1.f, 30.f},
            {1.f , 1.f , 1.f},
        };
        /* Init X and Q */
        std::vector<glm::vec3>              _X = {
            {0.f, 0.f, 0.f},
            {0.f, 5.f, 0.f},
        };
        std::vector<glm::quat>              _Q = {
            {1.f, 0.f, 0.f, 0.f},
            {1.f, 0.f, 0.f, 0.f},
        };


        glm::vec3 const _boxColor  { 121.0f / 255, 207.0f / 255, 171.0f / 255 };
        glm::vec3 const _lineColor {1.f, 1.f, 1.f};

        RigidBodySys _rigidBodySys;
        glm::vec3 const gravity = {0.f, -9.8f, 0.f};

        /* Player part */
        glm::vec3 const drag_force_x = {5.f, 0.f, 0.f};
        glm::vec3 const drag_force_y = {0.f, 5.f, 0.f};
        glm::vec3 const drag_force_z = {0.f, 0.f, 5.f};

        /* Scene */
        float mass = 1.f;
        int lenNum = 14;
	};
}