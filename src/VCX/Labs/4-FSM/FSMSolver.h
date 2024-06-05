#pragma once

#include <glm/glm.hpp>
#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "Labs/4-FSM/MassSpringSystem.h"

namespace VCX::Labs::FSM {
    class FSMSolver {
    public:
        FSMSolver ();
        FSMSolver (MassSpringSystem const &, float, int);

        void Solve(MassSpringSystem const &);
        void Step (MassSpringSystem &);
    
        Eigen::SparseMatrix<float> L, J, M;
        Eigen::SparseMatrix<float> A;
    private:

        Eigen::VectorXf            f_ext;
        Eigen::VectorXf            x, v;

        float                      dt;
        int                        iters;
    };
}