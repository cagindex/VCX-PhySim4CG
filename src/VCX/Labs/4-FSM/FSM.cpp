#include "Labs/4-FSM/FSM.h"

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

    static Eigen::SparseMatrix<float> CreateEigenSparseMatrix(std::size_t n, std::vector<Eigen::Triplet<float>> const & triplets) {
        Eigen::SparseMatrix<float> matLinearized(n, n);
        matLinearized.setFromTriplets(triplets.begin(), triplets.end());
        return matLinearized;
    }

    Eigen::SparseMatrix<float> GetMatrix_L (MassSpringSystem const & system) {

    }

    Eigen::SparseMatrix<float> GetMatrix_J (MassSpringSystem const & system) {

    }

    void FastMassSpringSimulation(MassSpringSystem & system, float const dt) {

    }
}