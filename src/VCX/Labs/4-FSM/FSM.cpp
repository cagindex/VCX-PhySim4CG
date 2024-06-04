#include "Labs/4-FSM/FSM.h"
#include <cmath>

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
            reinterpret_cast<glm::vec3 cosnt *> (eigen_v.data() + eigen_v.size())
        );
    }

    static Eigen::SparseMatrix<float> CreateEigenSparseMatrix(std::size_t w, std::size_t h, std::vector<Eigen::Triplet<float>> const & triplets) {
        Eigen::SparseMatrix<float> matLinearized(w, h);
        matLinearized.setFromTriplets(triplets.begin(), triplets.end());
        return matLinearized;
    }

    /**
     * L := (\sum_i k_i * A_i * A_i^T) \otimes I_3 | Shape (3m, 3m)
     * A := (\sum_i k_i * A_i * A_i^T)             | Shape ( m,  m)
    */
    Eigen::SparseMatrix<float> GetMatrix_L (MassSpringSystem const & system) {
        std::size_t m = system.Positions.size();
        Eigen::SparseMatrix<float> L( 3*m, 3*m );

        Eigen::SparseMatrix<float> A(   m,   m ),
                                 tmp(   m,   m );
        std::vector<Eigen::Triplet<float>> triplets{ {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0} };

        // A
        float k  = system.Stiffness;
        for (auto spring : system.Springs) {
            std::size_t i1 = spring.AdjIdx.first;
            std::size_t i2 = spring.AdjIdx.second;

            triplets[0] = { i1, i1, k };
            triplets[1] = { i1, i2, -k };
            triplets[2] = { i2, i1, -k };
            triplets[3] = { i2, i2, k };

            tmp.setFromTriplets(triplets.begin(), triplets.end());
            A += tmp;
            tmp.setZero();
        }

        // L
        for (int j = 0; j < A.outerSize(); ++j)
        {
            for (Eigen::SparseMatrix<float>::InnerIterator it(A, j); it; ++it)
            {
                auto r = it.row(), c = it.col();
                float value = it.value();
                L.insert(r      , c      ) = value;
                L.insert(r +   m, c +   m) = value;
                L.insert(r + 2*m, c + 2*m) = value;
            }
        }
        
        return L;
    }

    /**
     * J := (\sum_i k_i A_i S_i^T) \otimes I_e | Shape (3m, 3s)
     * S_i := {\delta_ij}_j                    | Shape (s,)
     * A   := (\sum_i k_i A_i S_i^T)           | Shape ( m,  s)
    */
    Eigen::SparseMatrix<float> GetMatrix_J (MassSpringSystem const & system) {
        std::size_t m = system.Positions.size();
        std::size_t s = system.Springs.size();
        Eigen::SparseMatrix<float> J( 3*m, 3*s );

        Eigen::SparseMatrix<float> A(   m,   s ),
                                 tmp(   m,   s );
        std::vector<Eigen::Triplet<float>> triplets{ {0, 0, 0}, {0, 0, 0} };

        // A
        float k  = system.Stiffness;
        for (int i = 0; i < s; ++i)
        {
            auto spring = system.Springs[i];

            std::size_t i1 = spring.AdjIdx.first;
            std::size_t i2 = spring.AdjIdx.second;

            triplets[0] = { i1, i, k };
            triplets[1] = { i2, i, -k };

            tmp.setFromTriplets(triplets.begin(), triplets.end());
            A += tmp;
            tmp.setZero();
        }

        // J
        for (int j = 0; j < A.outerSize(); ++j)
        {
            for (Eigen::SparseMatrix<float>::InnerIterator it(A, j); it; ++it)
            {
                auto r = it.row(), c = it.col();
                float value = it.value();
                J.insert(r      , c      ) = value;
                J.insert(r +   m, c +   s) = value;
                J.insert(r + 2*m, c + 2*s) = value;
            }
        }
        
        return J;
    }

    Eigen::SparseMatrix<float> GetMatrix_M (MassSpringSystem const & system) {
        std::size_t m = system.Positions.size();
        Eigen::SparseMatrix<float> M ( 3*m, 3*m );
        for (int i = 0; i < 3*m; ++i)
            for (int j = 0; j < 3*m; ++j)
                M.insert(i, j) = system.Mass;
        return M;
    }

    /**
     * b := h^2 f_ext - M y
     * y := q_n + h * v_n
    */
    Eigen::VectorXf GetVector_b (MassSpringSystem const & system, 
                                 Eigen::VectorXf  const & x,
                                 Eigen::VectorXf  const & v,
                                 float                    ddt) {
        std::size_t m = system.Positions.size();

        Eigen::VectorXf my = system.Mass * (x + ddt * v);
        Eigen::VectorXf f_ext ( 3*m );
        for (int i = 0; i < 3*m; i += 3)
        {
            f_ext(i)   = 0.f;
            f_ext(i+1) = system.Gravity;
            f_ext(i+2) = 0.f;
        }

        return ddt*ddt*f_ext - my;
    }

    void NormD(MassSpringSystem & system, Eigen::VectorXf & d) {
        std::size_t s = system.Springs.size();
        for (std::size_t i = 0; i < 3*s; i += 3)
        {
            float x = d(i+0), y = d(i+1), z = d(i+2);
            float len = std::sqrt( x*x + y*y + z*z ); 

            d(i+0) = x / len;
            d(i+1) = y / len;
            d(i+2) = z / len;
        }
    }

    void Update_d (Eigen::SparseMatrix<float> const & J,
                   Eigen::VectorXf            const & x,
                   Eigen::VectorXf                  & d,
                   float                              lr) {
        d -= lr * (-J.transpose() * x);
    }

    void Update_x (Eigen::SparseMatrix<float> const & L,
                   Eigen::SparseMatrix<float> const & J,
                   Eigen::VectorXf            const & d,
                   Eigen::VectorXf            const & b,
                   Eigen::VectorXf                  & x,
                   float                              mass,
                   float                              ddt,
                   float                              lr) {
        
    }

    void FastMassSpringSimulation(MassSpringSystem & system, float const dt) {
    }
}