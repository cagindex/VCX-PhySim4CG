#pragma once

#include <glm/glm.hpp>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <spdlog/spdlog.h>

#include "Labs/4-FSM/MassSpringSystem.h"

namespace VCX::Labs::FSM {
    class FSMSolver {
    public:
        FSMSolver ();

        void Reset(MassSpringSystem const &, float, int);
        void Solve(MassSpringSystem const &);
        void Step (MassSpringSystem &);
    
        Eigen::SparseMatrix<float> L, J, M;
        Eigen::SparseMatrix<float> A;
    private:
        Eigen::SimplicialLLT<Eigen::SparseMatrix<float> > solver;

        Eigen::VectorXf            f_ext;
        Eigen::VectorXf            x, v;

        float                      dt;
        int                        iters;
    };
}