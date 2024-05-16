#pragma once
#define GLM_ENABLE_EXPERIMENTAL

#include "Engine/app.h"
#include "Engine/Async.hpp"
#include "Engine/GL/Frame.hpp"
#include "Engine/GL/Program.h"
#include "Engine/GL/RenderItem.h"

#include "Labs/Common/ICase.h"
#include "Labs/Common/ImageRGB.h"
#include "Labs/Common/OrbitCameraManager.h"

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/string_cast.hpp>
#include <vector>

namespace VCX::Labs::FEM {

	class CaseTest : public Common::ICase {
    public:
        CaseTest();

		virtual std::string_view const GetName() override { return "Test"; }

		virtual void					 OnSetupPropsUI() override;
        virtual Common::CaseRenderResult OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) override;
        virtual void                     OnProcessInput(ImVec2 const & pos) override;

        void Reset();  
        void Simulate(float dt);
        void Smooth_vec();
        glm::mat3 BuildEdgeMatrix();
        void OnProcessMouseControl(glm::vec3 mourseDelta);

	private:
		Engine::GL::UniqueProgram           _program;
        Engine::GL::UniqueRenderFrame       _frame;
        Engine::Camera                      _camera { .Eye = glm::vec3(-3, 3, 3) };
        Common::OrbitCameraManager          _cameraManager;

        Engine::GL::UniqueIndexedRenderItem _boxItem;
        Engine::GL::UniqueIndexedRenderItem _lineItem;

		/* FEM Test */
        float mass = 10.f;

        const std::vector<glm::vec3>     ref_pos = {
            { 0.f,   0.f,  0.f},
            { 1.f,   0.f,  0.f},
            { 0.f,   1.f,  0.f},   
            { 0.f,   0.f,  1.f},

            { 2.0f, -1.0f,  0.0f},
            { 0.0f, -1.0f,  2.0f },
            {-2.0f, -1.0f,  0.0f },
            { 0.0f, -1.0f, -2.0f },

            {0.f, -1.f, 0.f},
            {4.f, -1.f, 0.f},
            {0.f, -1.f, 3.f}
        };
        const std::vector<float>         ref_mass = { 1.f, 1.f, 1.f, 1.f };

        std::vector<glm::vec3> vec;
        std::vector<glm::vec3> pos;
        std::vector<glm::vec3> f;

        const std::vector<std::uint32_t> line_index = { 0, 1, 1, 2, 0, 2, 0, 3, 1, 3, 2, 3, 4, 5, 5, 6, 6, 7, 7, 4, 4, 6, 5, 7, 8, 9, 8, 10 }; // line index
        const std::vector<std::uint32_t> tri_index = { 0, 1, 3, 1, 2, 3, 0, 2, 3, 0, 1, 2 };

        glm::vec3 _boxColor { 121.0f / 255, 207.0f / 255, 171.0f / 255 };

        int Y = 10000.f;
        float P = 0.3;
        float rho = 400.f;
        glm::vec3 gravity = { 0.f, -9.8f, 0.f };

        glm::mat3 Dm { 0.f };
        glm::mat3 Dm_inv { 0.f };
	};
}