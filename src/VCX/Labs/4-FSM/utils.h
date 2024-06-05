// #pragma once

// #include <Eigen/Dense>
// #include <Eigen/Sparse>
// #include "Labs/4-FSM/MassSpringSystem.h"

// namespace VCX::Labs::FSM {
//     // Convert between glm and eigen
//     static Eigen::VectorXf glm2eigen (std::vector<glm::vec3> const &);
//     static std::vector<glm::vec3> eigen2glm (Eigen::VectorXf const &);

//     // Get L, J, M, d
//     static Eigen::SparseMatrix<float> GetMatrix_L (MassSpringSystem const &);
//     static Eigen::SparseMatrix<float> GetMatrix_J (MassSpringSystem const &);
//     static Eigen::SparseMatrix<float> GetMatrix_M (MassSpringSystem const &);
//     static Eigen::VectorXf GetVector_d (MassSpringSystem const & );

//     // Normalize D to restLength of given MassSpringSystem
//     static void NormD (MassSpringSystem const &, Eigen::VectorXf &);

//     // Solve Ax = b
//     static Eigen::VectorXf ComputeSimplicialLLT(Eigen::SparseMatrix<float> const &, Eigen::VectorXf const &);
// }