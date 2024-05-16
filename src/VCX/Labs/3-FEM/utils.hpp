#include <iostream>
#include <Eigen/Dense>
#include <glm/glm.hpp>

namespace VCX::Labs::FEM {
    static inline Eigen::Matrix3f glm2eigen_Matrix3(glm::mat3 const& m){
        Eigen::Matrix3f ret = Eigen::Matrix3f::Zero();
        for (int col = 0; col < 3; ++col)
            for (int row = 0; row < 3; ++row)
                ret(row, col) = m[col][row];
        return ret;
    }

    static inline glm::mat3 eigen2glm_Matrix3(Eigen::Matrix3f const& m){
        glm::mat3 ret = glm::mat3(0.f);
        for (int col = 0; col < 3; ++col)
            for (int row = 0; row < 3; ++row)
                ret[col][row] = m(row, col);
        return ret;
    }
    
    static inline glm::vec3 eigen2glm_Vector3(Eigen::Vector3f const& vec){
        return glm::vec3(vec(0), vec(1), vec(2));
    }  

    struct svd_ret {
        glm::vec3 S;
        glm::mat3 U, V;
    };

    static inline svd_ret svd(glm::mat3 const & m){
        svd_ret ret;

        Eigen::Matrix3f eigen_m = glm2eigen_Matrix3(m);

        Eigen::JacobiSVD<Eigen::Matrix3f> svd(eigen_m, Eigen::ComputeFullU | Eigen::ComputeFullV);

        auto U = svd.matrixU();
        auto sigma = svd.singularValues();
        auto V = svd.matrixV();

        ret.U = eigen2glm_Matrix3(U);
        ret.S = eigen2glm_Vector3(sigma);
        ret.V = eigen2glm_Matrix3(V);

        return ret;
    }
}