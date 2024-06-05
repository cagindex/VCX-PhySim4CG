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
        std::vector<glm::vec3> f;
        for (auto pos : system.Positions)
            f.push_back({ 0.f, system.Gravity, 0.f });
        f_ext = glm2eigen( f );

        // Init x, v
        x = glm2eigen( system.Positions  ); 
        v = glm2eigen( system.Velocities ); 
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
    void FSMSolver::Solve (MassSpringSystem const & system) {
        float ddt = dt / iters;
        for (int i = 0; i < iters; ++i){
            // Local Step
            Eigen::VectorXf d = GetVector_d( system );

            // Global Step
            Eigen::VectorXf y = x + system.Damping * ddt * v;
            Eigen::VectorXf b = ddt*ddt*J*d + M*y - ddt*ddt*f_ext;

            Eigen::VectorXf x_new = solver.solve(b);
            Eigen::VectorXf v_new = (x_new - x) / ddt;

            x = x_new; 
            v = v_new;  
        }
    }

    void FSMSolver::Step (MassSpringSystem & system) {
        std::vector<glm::vec3> pos_new  = eigen2glm( x );
        std::vector<glm::vec3> vel_new  = eigen2glm( v );

        for (std::size_t idx = 0; idx < system.Positions.size(); ++idx)
        {
            if (system.Fixed[idx] == true)
            {
                pos_new[idx] = system.Positions[idx];
                vel_new[idx] = { 0.f, 0.f, 0.f };
            }
        }

        system.Positions = pos_new;
        system.Velocities = vel_new;

        x = glm2eigen(pos_new);
        v = glm2eigen(vel_new);
    }
}