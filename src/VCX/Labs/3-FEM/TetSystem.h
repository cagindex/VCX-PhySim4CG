#pragma once
#define GLM_ENABLE_EXPERIMENTAL

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/string_cast.hpp>

#include <Eigen/Dense>
#include <vector>

#include "Labs/3-FEM/utils.hpp"

namespace VCX::Labs::FEM {
    class TetSystem {
    public:
        TetSystem() {};

        void InitCube(std::size_t const wx, std::size_t const wy, std::size_t const wz, float const delta);
        void Clear(); // Clear all contains

        /* Build System Part */
        void AddParticle(glm::vec3 const x); // Add particle X to system
        void AddTet(int id1, int id2, int id3, int id4); // add <id1, id2, id3, id4> as a tet
        int  GetID(std::size_t const i, std::size_t const j, std::size_t k) const;
        std::vector<glm::vec3> const GetX() const;

        /* Simulate Part */
        glm::mat3 Build_Edge_Matrix(const int tet) const;
        void      Update(const float dt, bool Neohookean);
        void      Smooth_V();

        std::vector<glm::vec3> Force;

        float gravity_k   = 1.f;
        float mass        = 1.f;
        float stiffness_0 = 20000;
        float stiffness_1 = 5000;

        std::vector<glm::vec3> V, X; 
    private:
        glm::vec3 gravity = glm::vec3(0.f, -9.8f, 0.f);

        int tet_number = 0, vertex_number = 0;

        std::vector<int>       Tet;         // Tet [ | id1, id2, id3, id4 | id1, id2, id3, id4 | ... ]
        std::vector<glm::mat3> inv_Dm;

        std::size_t width_x = 0, width_y = 0, width_z = 0;

        float damp        = 0.999f;

        // For smoothing
        std::vector<glm::vec3> V_sum;
        std::vector<int>       V_num;
    };
}