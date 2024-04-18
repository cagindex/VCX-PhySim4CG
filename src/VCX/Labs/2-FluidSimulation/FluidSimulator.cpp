#include "Labs/2-FluidSimulation/FluidSimulator.h"

namespace VCX::Labs::Fluid{
    void Simulator::integrateParticles(float timeStep){
        for (int i = 0; i < m_iNumSpheres; ++i){
            m_particleVel[i] += gravity * timeStep;
            m_particlePos[i] += m_particleVel[i] * timeStep;
        }
    }

    void Simulator::handleParticleCollisions(
        glm::vec3 obstaclePos, 
        float obstacleRadius,
        glm::vec3 obstacleVel
    ){
        for (int i = 0; i < m_iNumSpheres; ++i){
            glm::vec3 pos = m_particlePos[i];
            glm::vec3 vel = m_particleVel[i];

            /* handle obstacle */
            float dis = glm::distance(pos, obstaclePos);
            if (dis < obstacleRadius + m_particleRadius){
                glm::vec3 _norm = glm::normalize(pos - obstaclePos);
                glm::vec3 _t_vel = vel - glm::dot(vel, _norm) * _norm;

                pos = obstaclePos + _norm * (obstacleRadius + m_particleRadius);
                vel = _t_vel + obstacleVel;
            }

            /* handle solid collision */
            glm::vec _min = glm::vec3(-0.5f);
            if (pos.x < _min.x + m_h + m_particleRadius){
                pos.x = m_h + m_particleRadius + _min.x;
                vel.x = 0.f;
            }
            if (pos.x > _min.x + (m_iCellX-1)*m_h - m_particleRadius){
                pos.x = _min.x + (m_iCellX-1)*m_h - m_particleRadius;
                vel.x = 0.f;
            }
            if (pos.y < _min.y + m_h + m_particleRadius){
                pos.y = m_h + m_particleRadius + _min.y;
                vel.y = 0.f;
            }
            if (pos.y > _min.y + (m_iCellY-1)*m_h - m_particleRadius){
                pos.y = _min.y + (m_iCellY-1)*m_h - m_particleRadius;
                vel.y = 0.f;
            }
            if (pos.z < _min.z + m_h + m_particleRadius){
                pos.z = m_h + m_particleRadius + _min.z;
                vel.z = 0.f;
            }
            if (pos.z > _min.z + (m_iCellZ-1)*m_h - m_particleRadius){
                pos.z = _min.z + (m_iCellZ-1)*m_h - m_particleRadius;
                vel.z = 0.f;
            }

            /* set back value */
            m_particlePos[i] = pos;
            m_particleVel[i] = vel;
        }
    }

    void Simulator::pushParticlesApart(int numParticleIters){
        float minDis = 2*m_particleRadius;
        glm::vec3 _min = -glm::vec3(0.5f);
        // init zero
        std::fill(m_hashtable.begin(), m_hashtable.end(), -1);
        std::fill(m_hashtableindex.begin(), m_hashtableindex.end(), 0);

        /* counting */
        for (auto pos : m_particlePos){
            pos -= _min; 
            int x = std::clamp(int(floor(pos.x * m_fInvSpacing)), 0, m_iCellX-1);
            int y = std::clamp(int(floor(pos.y * m_fInvSpacing)), 0, m_iCellY-1);
            int z = std::clamp(int(floor(pos.z * m_fInvSpacing)), 0, m_iCellZ-1);

            int idx = index2GridOffset({ x, y, z });
            m_hashtableindex[idx+1] += 1;
        }
        for (int i = 1; i < m_hashtableindex.size(); ++i)
            m_hashtableindex[i] += m_hashtableindex[i-1];
        for (int i = 0; i < m_iNumSpheres; ++i){
            glm::vec3 pos = m_particlePos[i] - _min;
            int x = std::clamp(int(floor(pos.x * m_fInvSpacing)), 0, m_iCellX-1);
            int y = std::clamp(int(floor(pos.y * m_fInvSpacing)), 0, m_iCellY-1);
            int z = std::clamp(int(floor(pos.z * m_fInvSpacing)), 0, m_iCellZ-1);
            int idx = index2GridOffset({ x, y, z });

            int startIdx = m_hashtableindex[idx];
            while( m_hashtable[startIdx] != -1 ) startIdx ++;
            m_hashtable[startIdx] = i;
            
        }

        /* Loop grid */
        for (int iter = 0; iter < numParticleIters; ++iter){

            for (int i = 0; i < m_iNumSpheres; ++i){
                glm::vec3 pos = m_particlePos[i];
                glm::vec3 _pos = pos - _min;

                int x = floor(_pos.x * m_fInvSpacing);
                int y = floor(_pos.y * m_fInvSpacing);
                int z = floor(_pos.z * m_fInvSpacing);

                int x_back = std::max(x-1, 0);
                int x_front = std::min(x + 1, m_iCellX-1);
                int y_back = std::max(y-1, 0);
                int y_front = std::min(y + 1, m_iCellY-1);
                int z_back = std::max(z-1, 0);
                int z_front = std::min(z + 1, m_iCellZ-1);

                for (int xi = x_back; xi <= x_front; ++xi){
                    for (int yi = y_back; yi <= y_front; ++yi){
                        for (int zi = z_back; zi <= z_front; ++zi){
                            int index = index2GridOffset({ xi, yi, zi });
                            int first = m_hashtableindex[index];
                            int next = m_hashtableindex[index+1];
                            
                            for (int idx = first; idx < next; ++idx){
                                int id = m_hashtable[idx];
                                if (id == i) continue;

                                glm::vec3 other_pos = m_particlePos[id]; 
                                float dis = glm::distance(pos, other_pos);
                                
                                if (dis == 0 || dis > minDis) continue;

                                float s = 0.5 * (minDis - dis)/dis;
                                glm::vec3 dx = (other_pos - pos) * s;

                                m_particlePos[i] -= dx;
                                m_particlePos[id] += dx;
                            }
                        }
                    }
                }
            }
        }
    }

    void Simulator::updateParticleColors(){
        glm::vec3 _min = glm::vec3(-0.5f);
        for (int i = 0; i < m_iNumSpheres; ++i){
            glm::vec3 color = m_particleColor[i];
            glm::vec3 pos = m_particlePos[i];
            pos -= _min;

            int x = std::clamp(int(pos.x * m_fInvSpacing), 1, m_iCellX-1);
            int y = std::clamp(int(pos.y * m_fInvSpacing), 1, m_iCellY-1);
            int z = std::clamp(int(pos.z * m_fInvSpacing), 1, m_iCellZ-1);

            int idx = index2GridOffset({ x, y, z });

            if (m_particleRestDensity > 0.0){
                float rel_d = m_particleDensity[idx] / m_particleRestDensity;
                rel_d = 2.f - std::clamp(rel_d, 0.f, 2.f);

                m_particleColor[i] = glm::vec3(rel_d * 0.1f, rel_d*0.1f, 1.f);
            }
        }
    }

    void Simulator::transferVelocities(bool toGrid, float flipRatio){
        glm::vec3 _min = -glm::vec3(0.5, 0.5, 0.5);
        float h  = m_h;
        float h1 = m_fInvSpacing;
        float h2 = m_h/2.f;

        /* Init type */
        if (toGrid){
            /* Store prev and init */
            m_pre_vel.swap(m_vel);
            std::fill(m_vel.begin(), m_vel.end(), glm::vec3(0.f, 0.f, 0.f));
            for (int i = 0; i < 3; ++i)
                std::fill(m_near_num[i].begin(), m_near_num[i].end(), 0.f);
            
            for(int i = 0; i < m_iNumCells; ++i)
                m_type[i] = m_s[i] == 0.0 ? SOLID_CELL : AIR_CELL;
            
            for (int i = 0; i < m_iNumSpheres; ++i){
                glm::vec3 pos = m_particlePos[i] - _min;

                int x = std::clamp(int(floor(pos.x * h1)), 0, m_iCellX-1);
                int y = std::clamp(int(floor(pos.y * h1)), 0, m_iCellY-1);
                int z = std::clamp(int(floor(pos.z * h1)), 0, m_iCellZ-1);
                int cell_idx = index2GridOffset({ x, y, z });
                if (m_type[cell_idx] == AIR_CELL)
                    m_type[cell_idx] = FLUID_CELL;
            }
        }

        /* Grid2Partial or Partial2Grid */
        for (int component = 0; component < 3; component++){

            float dx, dy, dz;
            if (component == 0)
            { dx = 0.f; dy = h2 ; dz = h2 ; }
            else if (component == 1)
            { dx = h2 ; dy = 0.f; dz = h2 ; }
            else
            { dx = h2 ; dy = h2 ; dz = 0.f; }

            int _i_ = component;

            for (int i = 0; i < m_iNumSpheres; ++i){
                glm::vec3 pos = m_particlePos[i] - _min;

                /////
                pos.x = std::clamp(pos.x, m_h, (m_iCellX - 1)*m_h);
                pos.y = std::clamp(pos.y, m_h, (m_iCellY - 1)*m_h);
                pos.z = std::clamp(pos.z, m_h, (m_iCellZ - 1)*m_h);

                int x0 = std::min(int(floor((pos.x - dx)*h1)), m_iCellX-2);
                int x1 = std::min(x0 + 1, m_iCellX-2);
                float tx = (pos.x - dx - x0 * h) * h1;

                int y0 = std::min(int(floor((pos.y - dy)*h1)), m_iCellY-2);
                int y1 = std::min(y0 + 1, m_iCellY-2);
                float ty = (pos.y - dy - y0 * h) * h1;

                int z0 = std::min(int(floor((pos.z - dz)*h1)), m_iCellZ-2);
                int z1 = std::min(z0 + 1, m_iCellZ-2);
                float tz = (pos.z - dz - z0 * h) * h1;

                float sx = 1.f - tx;
                float sy = 1.f - ty;
                float sz = 1.f - tz;

                //     d7-----d6    ^z
                //     /|    /|    /
                //y^ d4 --- d5|   /
                // |  |d3 - | d2
                // |  |/    |/
                //   d0 --- d1  ---> x
                float d[8];
                int neib[8];

                d[0] = sx*sy*sz;
                neib[0] = index2GridOffset({ x0, y0, z0 });

                d[1] = tx*sy*sz;
                neib[1] = index2GridOffset({ x1, y0, z0 });

                d[2] = tx*sy*tz;
                neib[2] = index2GridOffset({ x1, y0, z1 });

                d[3] = sx*sy*tz;
                neib[3] = index2GridOffset({ x0, y0, z1 });

                d[4] = sx*ty*sz;
                neib[4] = index2GridOffset({ x0, y1, z0 });

                d[5] = tx*ty*sz;
                neib[5] = index2GridOffset({ x1, y1, z0 });

                d[6] = tx*ty*tz;
                neib[6] = index2GridOffset({ x1, y1, z1 });

                d[7] = sx*ty*tz;
                neib[7] = index2GridOffset({ x0, y1, z1 });


                if (toGrid){
                    float pv = m_particleVel[i][_i_];
                    for (int j = 0; j < 8; ++j){
                        m_vel[neib[j]][_i_] += pv * d[j];
                        m_near_num[_i_][neib[j]] += d[j];
                    }
                }
                else{
                    int offset;
                    if (component == 0) offset = m_iCellY * m_iCellZ;
                    else if (component == 1) offset = m_iCellZ;
                    else offset = 1;

                    float valid[8];

                    for (int j = 0; j < 8; ++j){
                        valid[j] = (m_type[neib[j]] != AIR_CELL || m_type[neib[j] - offset] != AIR_CELL) ? 1.0 : 0.0;    
                    }

                    float v = m_particleVel[i][_i_];
                    float dd = 0.f;
                    for (int j = 0; j < 8; ++j){
                        dd += valid[j]*d[j];
                    }

                    if (dd > 0.0){
                        float picV = 0.f;
                        for (int j = 0; j < 8; ++j){
                            picV += valid[j]*d[j]*m_vel[neib[j]][_i_];
                        }
                        picV /= dd;

                        float corr = 0.f;
                        for (int j = 0; j < 8; ++j){
                            corr += valid[j]*d[j]*(m_vel[neib[j]][_i_] - m_pre_vel[neib[j]][_i_]);
                        }
                        corr /= dd;

                        float flipV = v + corr;
                        m_particleVel[i][_i_] = (1.0 - flipRatio) * picV + flipRatio * flipV;
                    }
                }
            }

            /* Solid Neib */
            if (toGrid){
                for (int i = 0; i < m_iNumCells; ++i){
                    if (m_near_num[component][i] > 0.0){
                        m_vel[i][component] /= m_near_num[component][i];
                    }
                }

                for (int i = 0; i < m_iCellX; ++i){
                    for (int j = 0; j < m_iCellY; ++j){
                        for (int k = 0; k < m_iCellZ; ++k){
                            int index = index2GridOffset({ i, j, k });
                            int index_x = index2GridOffset({ i - 1, j, k });
                            int index_y = index2GridOffset({ i, j - 1, k });
                            int index_z = index2GridOffset({ i, j, k - 1 });

                            bool solid = m_type[index] == SOLID_CELL;
                            if (solid || (i > 0 && m_type[index_x] == SOLID_CELL))
                                m_vel[index].x = m_pre_vel[index].x;
                            if (solid || (j > 0 && m_type[index_y] == SOLID_CELL))
                                m_vel[index].y = m_pre_vel[index].y;
                            if (solid || (k > 0 && m_type[index_z] == SOLID_CELL))
                                m_vel[index].z = m_pre_vel[index].z;
                        }
                    }
                }
            }
        }
    }

    void Simulator::updateParticleDensity(){
        std::fill(m_particleDensity.begin(), m_particleDensity.end(), 0.f);

        glm::vec3 _min = -glm::vec3(0.5, 0.5, 0.5);
        float h  = m_h;
        float h1 = m_fInvSpacing;
        float h2 = m_h/2.f;

        for (int i = 0; i < m_iNumSpheres; ++i){
            glm::vec3 pos = m_particlePos[i] - _min;

            int x0 = int(floor((pos.x - h2)*h1));
            int x1 = std::min(x0 + 1, m_iCellX-2);
            float tx = (pos.x - h2 - x0 * h) * h1;

            int y0 = int(floor((pos.y - h2)*h1));
            int y1 = std::min(y0 + 1, m_iCellY-2);
            float ty = (pos.y - h2 - y0 * h) * h1;

            int z0 = int(floor((pos.z - h2)*h1));
            int z1 = std::min(z0 + 1, m_iCellZ-2);
            float tz = (pos.z - h2 - z0 * h) * h1;

            float sx = 1.f - tx;
            float sy = 1.f - ty;
            float sz = 1.f - tz;

            if (x0 < m_iCellX && y0 < m_iCellY && z0 < m_iCellZ) 
                m_particleDensity[index2GridOffset({x0, y0, z0})] += sx*sy*sz;
            if (x1 < m_iCellX && y0 < m_iCellY && z0 < m_iCellZ) 
                m_particleDensity[index2GridOffset({x1, y0, z0})] += tx*sy*sz;
            if (x1 < m_iCellX && y1 < m_iCellY && z0 < m_iCellZ) 
                m_particleDensity[index2GridOffset({x1, y1, z0})] += tx*ty*sz;
            if (x0 < m_iCellX && y1 < m_iCellY && z0 < m_iCellZ) 
                m_particleDensity[index2GridOffset({x0, y1, z0})] += sx*ty*sz;
            if (x0 < m_iCellX && y0 < m_iCellY && z1 < m_iCellZ) 
                m_particleDensity[index2GridOffset({x0, y0, z1})] += sx*sy*tz;
            if (x1 < m_iCellX && y0 < m_iCellY && z1 < m_iCellZ) 
                m_particleDensity[index2GridOffset({x1, y0, z1})] += tx*sy*tz;
            if (x1 < m_iCellX && y1 < m_iCellY && z1 < m_iCellZ) 
                m_particleDensity[index2GridOffset({x1, y1, z1})] += tx*ty*tz;
            if (x1 < m_iCellX && y0 < m_iCellY && z1 < m_iCellZ) 
                m_particleDensity[index2GridOffset({x1, y0, z1})] += tx*sy*tz;
        }

        if (m_particleRestDensity == 0.0){
            float sum = 0.f;
            int numCells = 0;

            for (int i = 0; i < m_iNumCells; ++i){
                if (m_type[i] == FLUID_CELL){
                    sum += m_particleDensity[i];
                    numCells += 1;
                }
            }

            if (numCells > 0) m_particleRestDensity = sum / numCells;
        }
    }

    void Simulator::solveIncompressibility(int numIters, float dt, float overRelaxation, bool compensateDrift){
        for (int i = 0; i < m_iNumCells; ++i){
            m_pre_vel[i] = m_vel[i];
        }
        for (int iter = 0; iter < numIters; ++iter){

            for (int i = 1; i < m_iCellX-1; ++i){
                for (int j = 1; j < m_iCellY-1; ++j){
                    for (int k = 1; k < m_iCellZ-1; ++k){
                        int idx = index2GridOffset({ i, j, k });

                        if (m_type[idx] != FLUID_CELL) continue;

                        int center = idx;
                        int x_back = index2GridOffset({ i-1, j, k });
                        int x_front = index2GridOffset({ i+1, j, k });
                        int y_back = index2GridOffset({ i, j-1, k });
                        int y_front = index2GridOffset({ i, j+1, k });
                        int z_back = index2GridOffset({ i, j, k-1 });
                        int z_front = index2GridOffset({ i, j, k+1 });

                        float s_x_front = m_s[x_front];
                        float s_x_back  = m_s[x_back];
                        float s_y_front = m_s[y_front];
                        float s_y_back  = m_s[y_back];
                        float s_z_front = m_s[z_front];
                        float s_z_back  = m_s[z_back];
                        float s = s_x_front + s_y_front + s_z_front + s_x_back + s_y_back + s_z_back;
                        if ( s == 0.0 ) continue;

                        float d = overRelaxation * (m_vel[x_front].x + m_vel[y_front].y + m_vel[z_front].z - m_vel[idx].x - m_vel[idx].y - m_vel[idx].z);

                        if (m_particleRestDensity > 0.0 && compensateDrift){
                            float k = 1.0;
                            float delta_rho = m_particleDensity[center] - m_particleRestDensity;
                            if (delta_rho > 0.0)
                                d -= k * delta_rho;
                        }

                        float Genshin = d/s;
                        m_vel[center].x += Genshin * s_x_back;
                        m_vel[center].y += Genshin * s_y_back;
                        m_vel[center].z += Genshin * s_z_back;
                        m_vel[x_front].x -= Genshin * s_x_front;
                        m_vel[y_front].y -= Genshin * s_y_front;
                        m_vel[z_front].z -= Genshin * s_z_front;
                    }
                }
            }
        }
    }

    inline bool Simulator::isValidVelocity(int i, int j, int k, int dir){}
    inline int Simulator::index2GridOffset(glm::ivec3 index){
        return ((index.x * m_iCellY) + index.y)*m_iCellZ + index.z;
    }

    void Simulator::updateSolidCells(glm::vec3 obstaclePos, float r){
        glm::vec3 _min = -glm::vec3(0.5f);
        for (int i = 1; i < m_iCellX-2; ++i){
            for (int j = 1; j < m_iCellY-2; ++j){
                for (int k = 1; k < m_iCellZ-2; ++k){
                    int index = index2GridOffset({ i, j, k });
                    glm::vec3 pos = _min + glm::vec3(i*m_h, j*m_h, k*m_h);

                    float dx = (pos.x - obstaclePos.x);
                    float dy = (pos.y - obstaclePos.y);
                    float dz = (pos.z - obstaclePos.z);
                    float dis2 = dx * dx + dy * dy + dz * dz;

                    m_s[index] = 1.f;
                    if (dis2 < r*r){
                        m_s[index] = 0.f;
                    }
                }
            }
        }
    }
}