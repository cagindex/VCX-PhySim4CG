#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <fcl/narrowphase/collision.h>

#include <Eigen/Dense>
#include <Eigen/Sparse>

namespace VCX::Labs::RigidBody {
    inline Eigen::Quaternionf glm2eigen(glm::quat const& q){
        return Eigen::Quaternionf(q.w, q.x, q.y, q.z);
    }

    inline glm::vec3 flc2glm(fcl::Vector3f const& vec){
        return glm::vec3(vec[0], vec[1], vec[2]);
    }    

    inline glm::mat3 Get_Cross_Matrix(glm::vec3 const& vec){
        return glm::mat3({
            {0.f, -vec.z, vec.y},
            {vec.z, 0.f, -vec.x},
            {-vec.y, vec.x, 0.f}
        });
    }
}