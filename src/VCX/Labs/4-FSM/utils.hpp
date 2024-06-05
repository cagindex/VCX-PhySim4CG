#include <cmath>

#include <iostream>
#include <spdlog/spdlog.h>

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include "Labs/4-FSM/MassSpringSystem.h"

namespace VCX::Labs::FSM {
    static Eigen::VectorXf glm2eigen(std::vector<glm::vec3> const & glm_v) {
        Eigen::VectorXf v = Eigen::Map<Eigen::VectorXf const, Eigen::Aligned>(
            reinterpret_cast<float const *>(glm_v.data()), static_cast<int>(glm_v.size()*3)
        );
        return v;
    }

    static std::vector<glm::vec3> eigen2glm(Eigen::VectorXf const & eigen_v) {
        return std::vector<glm::vec3>(
            reinterpret_cast<glm::vec3 const *> (eigen_v.data()),
            reinterpret_cast<glm::vec3 const *> (eigen_v.data() + eigen_v.size())
        );
    }

    static Eigen::SparseMatrix<float> CreateEigenSparseMatrix(std::size_t w, std::size_t h, std::vector<Eigen::Triplet<float>> const & triplets) {
        Eigen::SparseMatrix<float> matLinearized(w, h);
        matLinearized.setFromTriplets(triplets.begin(), triplets.end());
        return matLinearized;
    }

    static void NormD (MassSpringSystem const & system, Eigen::VectorXf & d) {
        std::size_t s = system.Springs.size();
        for (std::size_t i = 0; i < s; i += 1)
        {
            float x = d(3*i+0), y = d(3*i+1), z = d(3*i+2);
            float len = std::sqrt( x*x + y*y + z*z ); 
            float restLen = system.Springs[i].RestLength;

            d(3*i+0) = restLen * x / len;
            d(3*i+1) = restLen * y / len;
            d(3*i+2) = restLen * z / len;
        }
    }

    /**
     * L := (\sum_i k_i * A_i * A_i^T) \otimes I_3 | Shape (3m, 3m)
     * A := (\sum_i k_i * A_i * A_i^T)             | Shape ( m,  m)
    */
    static Eigen::SparseMatrix<float> GetMatrix_L (MassSpringSystem const & system) {
        std::size_t m = system.Positions.size();
        Eigen::SparseMatrix<float> L( 3*m, 3*m );

        std::vector<Eigen::Triplet<float>> triplets;

        float k  = system.Stiffness;
        for (auto spring : system.Springs) {
            int i1 = spring.AdjIdx.first;
            int i2 = spring.AdjIdx.second;

            for (int j = 0; j < 3; ++j)
            {
                triplets.push_back({ 3*i1+j, 3*i1+j, k });
                triplets.push_back({ 3*i1+j, 3*i2+j, -k });
                triplets.push_back({ 3*i2+j, 3*i1+j, -k });
                triplets.push_back({ 3*i2+j, 3*i2+j, k });
            }
        }
        L.setFromTriplets( triplets.begin(), triplets.end() );

        return L;
    }

    /**
     * J := (\sum_i k_i A_i S_i^T) \otimes I_e | Shape (3m, 3s)
     * S_i := {\delta_ij}_j                    | Shape (s,)
     * A   := (\sum_i k_i A_i S_i^T)           | Shape ( m,  s)
    */
    static Eigen::SparseMatrix<float> GetMatrix_J (MassSpringSystem const & system) {
        std::size_t m = system.Positions.size();
        std::size_t s = system.Springs.size();
        Eigen::SparseMatrix<float> J( 3*m, 3*s );

        std::vector<Eigen::Triplet<float>> triplets;

        int count = 0;
        float k  = system.Stiffness;
        for (auto spring : system.Springs) {
            int i1 = spring.AdjIdx.first;
            int i2 = spring.AdjIdx.second;

            for (int j = 0; j < 3; ++j)
            {
                triplets.push_back({ 3*i1+j, 3*count+j, k });
                triplets.push_back({ 3*i2+j, 3*count+j, -k });
            }
            count++;
        }
        J.setFromTriplets( triplets.begin(), triplets.end() );

        return J;
    }

    static Eigen::SparseMatrix<float> GetMatrix_M (MassSpringSystem const & system) {
        std::size_t m = system.Positions.size();
        Eigen::SparseMatrix<float> M ( 3*m, 3*m );
        for (int i = 0; i < 3*m; ++i)
                M.insert(i, i) = system.Mass;
        return M;
    }

    static Eigen::VectorXf GetVector_d (MassSpringSystem const & system) {
        std::vector<glm::vec3> springs;
        for (auto spring : system.Springs){
            glm::vec3 p1 = system.Positions[spring.AdjIdx.first];
            glm::vec3 p2 = system.Positions[spring.AdjIdx.second];

            springs.push_back( p1 - p2 );
        }

        Eigen::VectorXf d = glm2eigen( springs );
        NormD(system, d);

        return d;
    }

    static Eigen::VectorXf ComputeSimplicialLLT(
        Eigen::SparseMatrix<float> const & A,
        Eigen::VectorXf const & b) {
        auto solver = Eigen::SimplicialLLT<Eigen::SparseMatrix<float>>(A);
        return solver.solve(b);
    }
}