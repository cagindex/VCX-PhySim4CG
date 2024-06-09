#include "Engine/app.h"
#include "Labs/4-FSM/Case3DTest.h"
#include "Labs/Common/ImGuiHelper.h"
#include "Labs/4-FSM/utils.hpp"
#include <cmath>

namespace VCX::Labs::FSM
{
    /**
     * BackGround Section
    */
    BackGroundRender::BackGroundRender():
        LineItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Lines)
    {
        std::vector<glm::vec3> poses;
        std::vector<std::uint32_t> indices;

        // Load poses
        int num = 600;
        float dis = 0.1f;
        for (int row = 1; row <= num; ++row)
        {
            for (int col = 1; col <= num; ++col)
            {
                poses.push_back({ dis*(row-num/2), 0.0f, dis*(col-num/2) });
            }
        }

        // Load indices
        for (int row = 0; row < num; ++row)
        {
            int startIdx = row*num;
            for (int col = 0; col < num-1; ++col)
            {
                indices.push_back(startIdx+col);
                indices.push_back(startIdx+col+1);
            }
        }
        for (int col = 0; col < num; ++col)
        {
            int startIdx = col;
            for (int row = 0; row < num-1; ++row)
            {
                indices.push_back(startIdx+row*num);
                indices.push_back(startIdx+(row+1)*num);
            }
        }

        // Update
        LineItem.UpdateVertexBuffer("position", Engine::make_span_bytes<glm::vec3>(poses));
        LineItem.UpdateElementBuffer(indices);
    };

    void BackGroundRender::render(Engine::GL::UniqueProgram & program)
    {
        program.GetUniforms().SetByName("u_Color", glm::vec3( 111.0f/255, 111.0f/255, 111.0f/255 ));
        LineItem.Draw({ program.Use() });
    }
    // BackGround End

    /**
     * SkeletonRender Section
    */
    SkeletonRender::SkeletonRender():
        LineItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Lines),
        PointItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Points)
    {}

    void SkeletonRender::render(Engine::GL::UniqueProgram & program)
    {
        program.GetUniforms().SetByName("u_Color", glm::vec3( 1.0f, 0.0f, 0.0f ));
        glPointSize(10.f);
        PointItem.Draw({ program.Use() });
        glPointSize(1.f);

        program.GetUniforms().SetByName("u_Color", glm::vec3( 1.0f, 1.0f, 1.0f ));
        glLineWidth(3.f);
        LineItem.Draw({ program.Use() });
        glLineWidth(1.f);
    }

    void SkeletonRender::load(const Skeleton & skele)
    {
        auto res = skele.Convert();
        LineItem.UpdateVertexBuffer("position", Engine::make_span_bytes<glm::vec3>(res.first));

        PointItem.UpdateVertexBuffer("position", Engine::make_span_bytes<glm::vec3>(res.first));
    }
    void SkeletonRender::loadAll(const Skeleton & skele)
    {
        auto res = skele.Convert();
        LineItem.UpdateElementBuffer(res.second);
        LineItem.UpdateVertexBuffer("position", Engine::make_span_bytes<glm::vec3>(res.first));

        PointItem.UpdateVertexBuffer("position", Engine::make_span_bytes<glm::vec3>(res.first));
    }
    // SkeletonRender End



    Case3DTest::Case3DTest() :
        _program(
            Engine::GL::UniqueProgram({
                Engine::GL::SharedShader("assets/shaders/flat.vert"),
                Engine::GL::SharedShader("assets/shaders/flat.frag")})),
        _particlesItem(Engine::GL::VertexLayout()
            .Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream , 0), Engine::GL::PrimitiveType::Points),
        _springsItem(Engine::GL::VertexLayout()
            .Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream , 0), Engine::GL::PrimitiveType::Lines)
        {
            _cameraManager.AutoRotate = false;
            _cameraManager.Save(_camera);

            _BVHLoader.Load(_filePath.c_str(), _skeleton, _action);

            skeletonRender.loadAll(_skeleton);

            ResetSystem();
        }

        void Case3DTest::OnSetupPropsUI()
        {
            ImGui::Text("AbaAba %d", 2024);
            if (ImGui::Button(_stopped ? "Start" : "Stop")) _stopped = !_stopped;
        }

        Common::CaseRenderResult Case3DTest::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize)
        {
            if (!_stopped)
            {
                _action.Load(_skeleton, Engine::GetDeltaTime());

                std::size_t const n = 33;
                auto constexpr GetID = [](std::size_t const i, std::size_t const j) { return i * (n + 1) + j; };

                float const dr = .1f;
                float const r0 = .5f;
                float const dtheta = 2 * 3.14f / n;

                glm::vec3 hips_pos = _skeleton.Root->GlobalPosition;
                glm::vec3 spine_pos;
                auto ptr = _skeleton.Root->ChiPtr;
                while(ptr != nullptr){
                    if (ptr->Name == "Spine"){
                        spine_pos = ptr->LocalOffset;
                    }
                    ptr = ptr->BroPtr;
                }

                for (int j = 0; j <= n; ++j){
                    float theta = dtheta * j;
                    float r     = r0;
                    _massSpringSystem.Positions[GetID(0, j)] = glm::vec3(r*std::cos(theta), 0.f, r*std::sin(theta)) + hips_pos;
                }

                _FSMSolver.Solve(_massSpringSystem);
                ShpereCollisionConstraint(_massSpringSystem, hips_pos - spine_pos, .4f);
                ShpereCollisionConstraint(_massSpringSystem, hips_pos - 10.f*spine_pos, .4f);
                SpringConstraint(_massSpringSystem, 0.12f, 15);
            }

            _particlesItem.UpdateVertexBuffer("position", Engine::make_span_bytes<glm::vec3>(_massSpringSystem.Positions));
            _springsItem.UpdateVertexBuffer("position", Engine::make_span_bytes<glm::vec3>(_massSpringSystem.Positions));

            skeletonRender.load(_skeleton);

            _frame.Resize(desiredSize);

            _cameraManager.Update(_camera);

            _program.GetUniforms().SetByName("u_Projection", _camera.GetProjectionMatrix((float(desiredSize.first) / desiredSize.second)));
            _program.GetUniforms().SetByName("u_View"      , _camera.GetViewMatrix());

            gl_using(_frame);

            BackGround.render(_program);
            skeletonRender.render(_program);

            _program.GetUniforms().SetByName("u_Color", glm::vec3(0.f, 0.f, 1.f));
            _springsItem.Draw({ _program.Use() });
            _program.GetUniforms().SetByName("u_Color", glm::vec3(1.f, 0.f, 0.f));
            _particlesItem.Draw({ _program.Use() });

            glPointSize(1.f);
            return Common::CaseRenderResult{
                .Fixed      = false,
                .Flipped    = true,
                .Image      = _frame.GetColorAttachment(),
                .ImageSize  = desiredSize,
            };
        }

        void Case3DTest::OnProcessInput(ImVec2 const & pos)
        {
            _cameraManager.ProcessInput(_camera, pos);
        }

        void Case3DTest::ResetSystem() {
            // _massSpringSystem = { };
            _massSpringSystem.Springs.clear();
            _massSpringSystem.Positions.clear();
            _massSpringSystem.Velocities.clear();
            _massSpringSystem.Fixed.clear();
            _massSpringSystem.Forces.clear();
            _massSpringSystem.Mass.clear();


            std::size_t const n = 33;
            std::size_t const m = 15;
            auto constexpr GetID = [](std::size_t const i, std::size_t const j) { return i * (n + 1) + j; };

            float const dr = .1f;
            float const r0 = .5f;
            float const dtheta = 2 * 3.14f / n;

            for (std::size_t i = 0; i <= m; i++) {
                for (std::size_t j = 0; j <= n; j++) {
                    float theta = dtheta * j;
                    float r     = r0 + i * dr;
                    glm::vec3 position = glm::vec3(r*std::cos(theta), 0.f, r*std::sin(theta)) + _skeleton.Root->GlobalPosition;
                    _massSpringSystem.AddParticle(position, _massSpringSystem.TotalMass/(int(n)*int(n)));
                    if (i > 0) _massSpringSystem.AddSpring(GetID(i, j), GetID(i - 1, j));
                    if (j > 0) _massSpringSystem.AddSpring(GetID(i, j), GetID(i, j - 1));
                    if (j == n) _massSpringSystem.AddSpring(GetID(i, j), GetID(i, 0));
                    // if (i > 0 && j > 0) _massSpringSystem.AddSpring(GetID(i, j), GetID(i - 1, j - 1));
                    // if (i > 0 && j < n) _massSpringSystem.AddSpring(GetID(i, j), GetID(i - 1, j + 1));

                    if (i == 0) _massSpringSystem.Fixed[GetID(i, j)] = true;
                }
            }


            std::vector<std::uint32_t> indices;
            for (auto const & spring : _massSpringSystem.Springs) {
                indices.push_back(std::uint32_t(spring.AdjIdx.first));
                indices.push_back(std::uint32_t(spring.AdjIdx.second));
            }

            _springsItem.UpdateElementBuffer(indices);

            _FSMSolver.Reset(_massSpringSystem, 0.005, 1);
        }
}

