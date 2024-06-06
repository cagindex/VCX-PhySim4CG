#pragma once

#include <vector>
#include <glm/glm.hpp>
#include <Eigen/Sparse>

namespace VCX::Labs::FSM {
    struct MassSpringSystem {
        struct Spring {
            std::pair<std::size_t, std::size_t> AdjIdx;
            float                               RestLength;
        };

        std::vector<glm::vec3>      Positions;
        std::vector<glm::vec3>      Velocities;
        std::vector<glm::vec3>      Forces;
        std::vector<float>          Mass;
        std::vector<int>            Fixed;
        float                       TotalMass { .25f };

        std::vector<Spring>         Springs;
        float                       Stiffness { 1.f };
        float                       Damping   { 0.993f };
        float                       Gravity   { 9.8f };

        Eigen::SparseMatrix<float>  L, J, M;


        void AddParticle(glm::vec3 const & position, float mass, glm::vec3 const & velocity = glm::vec3(0)) {
            Positions.push_back(position);
            Velocities.push_back(velocity);
            Mass.push_back(mass);
            Forces.push_back({ 0.f, -mass * Gravity, 0.f });
            Fixed.push_back(false);
        }

        void AddSpring(std::size_t const adjIdx0, std::size_t const adjIdx1, float const restLength = -1) {
            Springs.push_back({
                .AdjIdx     { adjIdx0, adjIdx1 },
                .RestLength { restLength < 0 ? glm::length(Positions[adjIdx0] - Positions[adjIdx1]) : restLength }
            });
        }
    };
}