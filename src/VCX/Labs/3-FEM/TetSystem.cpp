#include "Labs/3-FEM/TetSystem.h"
#include <cmath>

namespace VCX::Labs::FEM {
    void TetSystem::InitCube(std::size_t const wx, std::size_t const wy, std::size_t const wz, float const delta) {
        width_x = wx;
        width_y = wy;
        width_z = wz;

        std::size_t size = (wx + 1) * (wy + 1) * (wz + 1);
        Tet.reserve(24 * wx * wy * wz);
        inv_Dm.reserve(6 * wx * wy * wz);
        Force.reserve(size);
        V.reserve(size);
        X.reserve(size);

        for (std::size_t i = 0; i <= wx; i++) {
            for (std::size_t j = 0; j <= wy; j++) {
                for (std::size_t k = 0; k <= wz; k++) {
                    AddParticle({ i * delta, j * delta, k * delta });
                }
            }
        }

        /*   ^ Z
             |
             1-----3
            /|    /|
           5 --- 7 |
           | 0 - | 2 ---> Y
           |/    |/
           4 --- 6
          /
         /
        X
        */

        for (std::size_t i = 0; i < wx; i++) {
            for (std::size_t j = 0; j < wy; j++) {
                for (std::size_t k = 0; k < wz; k++) {
                    AddTet(GetID(i, j, k), GetID(i, j, k + 1), GetID(i + 1, j + 1, k + 1), GetID(i, j + 1, k + 1)); // 0 1 7 3
                    AddTet(GetID(i, j, k), GetID(i, j + 1, k), GetID(i, j + 1, k + 1), GetID(i + 1, j + 1, k + 1)); // 0 2 3 7
                    AddTet(GetID(i, j, k), GetID(i, j + 1, k), GetID(i + 1, j + 1, k + 1), GetID(i + 1, j + 1, k)); // 0 2 7 6
                    AddTet(GetID(i, j, k), GetID(i, j, k + 1), GetID(i + 1, j, k + 1), GetID(i + 1, j + 1, k + 1)); // 0 1 5 7
                    AddTet(GetID(i, j, k), GetID(i + 1, j, k), GetID(i + 1, j + 1, k + 1), GetID(i + 1, j, k + 1)); // 0 4 7 5
                    AddTet(GetID(i, j, k), GetID(i + 1, j, k), GetID(i + 1, j + 1, k), GetID(i + 1, j + 1, k + 1)); // 0 4 6 7
                }
            }
        }

        for (int tet = 0; tet < tet_number; ++tet) {
            glm::mat3 Dm = Build_Edge_Matrix(tet);
            inv_Dm.emplace_back(glm::inverse(Dm));
        }

        for (int tet = 0; tet < tet_number; ++tet) {
            glm::vec3 X0 = X[Tet[4 * tet + 0]];
            glm::vec3 X1 = X[Tet[4 * tet + 1]];
            glm::vec3 X2 = X[Tet[4 * tet + 2]];
            glm::vec3 X3 = X[Tet[4 * tet + 3]]; 
        }

        V_sum.reserve(size);
        V_num.reserve(size);
        for (int i = 0; i < size; ++i) {
            V_sum.emplace_back(glm::vec3(0.f, 0.f, 0.f));
            V_num.emplace_back(0);
        }
    }

    void TetSystem::Clear() {
        tet_number = 0; 
        vertex_number = 0;

        Tet.clear(); 
        Force.clear();
        V.clear();
        X.clear();
        inv_Dm.clear();

        width_x = width_y = width_z = 0;

        V_sum.clear();
        V_num.clear();
    }

    inline int TetSystem::GetID(std::size_t const i, std::size_t const j, std::size_t const k) const {
        return i * (width_y + 1) * (width_z + 1) + j * (width_z + 1) + k;
    }

    void TetSystem::AddParticle(glm::vec3 const x) {
        X.emplace_back(x);
        V.emplace_back(glm::vec3(0.f));
        Force.emplace_back(glm::vec3(0.f));
        vertex_number += 1;
    }

    void TetSystem::AddTet(int id1, int id2, int id3, int id4) {
        Tet.emplace_back(id1);
        Tet.emplace_back(id2);
        Tet.emplace_back(id3);
        Tet.emplace_back(id4);

        tet_number += 1;
    }

    std::vector<glm::vec3> const TetSystem::GetX() const {
        return X;
    }

    glm::mat3 TetSystem::Build_Edge_Matrix(const int tet) const {
        glm::mat3 ret { 0.f };

        glm::vec3 X0 = X[Tet[4 * tet + 0]];
        glm::vec3 X1 = X[Tet[4 * tet + 1]];
        glm::vec3 X2 = X[Tet[4 * tet + 2]];
        glm::vec3 X3 = X[Tet[4 * tet + 3]];

        glm::vec3 X10 = X1 - X0;
        glm::vec3 X20 = X2 - X0;
        glm::vec3 X30 = X3 - X0;

        ret[0] = X10;
        ret[1] = X20;
        ret[2] = X30;

        return ret;
    }

    void TetSystem::Update(const float dt, bool Neohookean) {
        for (int i = 0; i < vertex_number; ++i)
            Force[i] += mass * gravity_k * gravity;
        
        for (int tet = 0; tet < tet_number; ++tet) {
            glm::mat3 P;

            if (Neohookean){
                glm::mat3 F = Build_Edge_Matrix(tet) * inv_Dm[tet];

                float J = glm::determinant(F);
                glm::mat3 inv_F_T = glm::transpose(glm::inverse(F));                
                P = stiffness_1 * (F - inv_F_T) + stiffness_0 * std::log(J) * inv_F_T;
            }
            else {
                glm::mat3 F = Build_Edge_Matrix(tet) * inv_Dm[tet];

                glm::mat3 G = 0.5f * (glm::transpose(F) * F - glm::mat3(1.f));

                glm::mat3 S = 2.f * stiffness_1 * G + stiffness_0 * (G[0][0] + G[1][1] + G[2][2]) * glm::mat3(1.f);

                P = F * S;
            }

            float     scale = -1.f / (6.f * glm::determinant(inv_Dm[tet]));
            glm::mat3 result = scale * P * glm::transpose(inv_Dm[tet]);

            Force[Tet[4 * tet + 0]] -= (result[0] + result[1] + result[2]);
            Force[Tet[4 * tet + 1]] += result[0];
            Force[Tet[4 * tet + 2]] += result[1];
            Force[Tet[4 * tet + 3]] += result[2];
        }

        Smooth_V();
        for (int i = 0; i < vertex_number; ++i) {
            V[i] = (V[i] + Force[i] / mass * dt) * damp;
            X[i] = X[i] + V[i] * dt;

            if (X[i].y < -1.f) {
                V[i].x = 0.f;
                V[i].z = 0.f;
                V[i].y += (-1.f - X[i].y) / dt;
                X[i].y = -1.f;
            }

            Force[i] = glm::vec3(0.f);
        }
    }

    void TetSystem::Smooth_V() {
        for (int i = 0; i < vertex_number; ++i) {
            V_sum[i] = glm::vec3(0.f, 0.f, 0.f);
            V_num[i] = 0;
        }

        for (int tet = 0; tet < tet_number; ++tet) {
            glm::vec3 sum = V[Tet[4 * tet + 0]] + V[Tet[4 * tet + 1]] + V[Tet[4 * tet + 2]] + V[Tet[4 * tet + 3]];

            for (int i = 0; i < 4; ++i) {
                V_sum[Tet[4 * tet + i]] += sum;
                V_num[Tet[4 * tet + i]] += 4;
            }
        }

        for (int i = 0; i < vertex_number; ++i) {
            V[i] = 0.9f * V[i] + 0.1f * V_sum[i] / float(V_num[i]);
        }
    }
}