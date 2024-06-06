#include <vector>

#include "Labs/4-FSM/FSMSolver.h"
#include "Labs/4-FSM/utils.hpp"

#include <iostream>
namespace VCX::Labs::FSM {
    FSMSolver::FSMSolver () {}

    void FSMSolver::Reset (MassSpringSystem const & system, float _dt, int _iters)
    {
        dt = _dt; 
        iters = _iters;
        // Init L, J, M
        L = GetMatrix_L(system);
        J = GetMatrix_J(system);
        M = GetMatrix_M(system);

        A = M + (dt / iters) * (dt / iters) * L;

        solver.compute( A );

        // Init f_ext
        f_ext = glm2eigen( system.Forces );

        // Init x, v
        x = glm2eigen( system.Positions  ); 
        v = glm2eigen( system.Velocities ); 
        prev_x = x;
    }

    /**
     * Because g(x) is a convex function
     * the solution of min g(x) is
     * $(M + h^2 L) x = h^2 J d + M y - h^2 f_ext$
     * which is 
     * $Ax = b$ A is symmetric matrix
     * 
     * y := 2x_{n} - x_{n-1} = x_{n} + v_{n} h 
    */
    void FSMSolver::Solve (MassSpringSystem & system) {
        float ddt = dt / iters;

        x = glm2eigen(system.Positions);
        for (int i = 0; i < iters; ++i){
            Eigen::VectorXf y = (system.Damping + 1) * x - system.Damping * prev_x;
            prev_x = x;

            // Local Step
            Eigen::VectorXf d = GetVector_d( system );

            // Global Step
            Eigen::VectorXf b = ddt*ddt*J*d + M*y + ddt*ddt*f_ext;
            Eigen::VectorXf x = solver.solve(b);

            std::vector<glm::vec3> xx = eigen2glm(x);
            for (std::size_t idx = 0; idx < system.Positions.size(); ++idx){
                if (! system.Fixed[idx])
                    system.Positions[idx] = xx[idx];
            }
        }
    }
}