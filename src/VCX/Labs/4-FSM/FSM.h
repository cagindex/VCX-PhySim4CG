#pragma once

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include "Labs/4-FSM/MassSpringSystem.h"

namespace VCX::Labs::FSM {
    Eigen::SparseMatrix<float> GetMatrix_L (MassSpringSystem const &);
    Eigen::SparseMatrix<float> GetMatrix_J (MassSpringSystem const &);

    void FastMassSpringSimulation(MassSpringSystem & , float const);
}