#pragma once

#include "Engine/Async.hpp"
#include "Engine/GL/Frame.hpp"
#include "Engine/GL/Program.h"
#include "Engine/GL/RenderItem.h"

#include "Labs/Common/ICase.h"
#include "Labs/Common/ImageRGB.h"
#include "Labs/Common/OrbitCameraManager.h"

#include "Labs/1-RigidBody/RigidBody.h"

namespace VCX::Labs::RigidBody {

	class CaseSpinningCube : public Common::ICase {
    public:
        CaseSpinningCube();

		virtual std::string_view const GetName() override { return "Spinning Cube"; }

		virtual void					 OnSetupPropsUI() override;
        virtual Common::CaseRenderResult OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) override;
        virtual void                     OnProcessInput(ImVec2 const & pos) override;

        void OnProcessMouseControl(glm::vec3 mourseDelta);

	private:
		Engine::GL::UniqueProgram           _program;
        Engine::GL::UniqueRenderFrame       _frame;
        Engine::Camera                      _camera { .Eye = glm::vec3(-3, 3, 3) };
        Common::OrbitCameraManager          _cameraManager;

        Engine::GL::UniqueIndexedRenderItem _boxItem;
        Engine::GL::UniqueIndexedRenderItem _lineItem;

		/* cube */
        float mass = 10.f;
        //     3-----2
        //    /|    /|
        //   0 --- 1 |
        //   | 7 - | 6
        //   |/    |/
        //   4 --- 5
        const std::vector<glm::vec3>     positions = {
            {-2.f,  1.f,  0.5f},
            { 2.f,  1.f,  0.5f},
            { 2.f, -1.f,  0.5f},
            {-2.f, -1.f,  0.5f},
            {-2.f,  1.f, -0.5f},
            { 2.f,  1.f, -0.5f},
            { 2.f, -1.f, -0.5f},
            {-2.f, -1.f, -0.5f},   
        };
        const std::vector<std::uint32_t> line_index = { 0, 1, 1, 2, 2, 3, 3, 0, 4, 5, 5, 6, 6, 7, 7, 4, 0, 4, 1, 5, 2, 6, 3, 7 }; // line index
        const std::vector<std::uint32_t> tri_index = { 0, 1, 2, 0, 2, 3, 1, 4, 0, 1, 4, 5, 1, 6, 5, 1, 2, 6, 2, 3, 7, 2, 6, 7, 0, 3, 7, 0, 4, 7, 4, 5, 6, 4, 6, 7 };

        const glm::mat3 Ibody = {
            {1.0417f, 0.f, 0.f},
            {0.f, 3.5417f, 0.f},
            {0.f, 0.f, 4.1667f}
        };

        glm::vec3 _boxColor { 121.0f / 255, 207.0f / 255, 171.0f / 255 };

        RigidBody _rigidbody;
        mesh      _mesh;

        float time = 0.f;

        float omega = 2.0f;
	};
}