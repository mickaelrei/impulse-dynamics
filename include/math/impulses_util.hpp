#include <vector>
#include <glm/glm.hpp>

#include <glm/glm.hpp>

#include "matrix.hpp"

// Esta função agora trabalha com coordenadas 3D (3N graus de liberdade)
Matrix<float> resolveSimultaneousCollisions3D(
    const std::vector<glm::vec3> &global_pos,
    const std::vector<glm::vec3> &global_vel,
    const std::vector<float> &global_mass,
    const std::vector<std::pair<size_t, size_t>> &colliding_pairs,
    float e = 1.0f
) {
    size_t N = global_mass.size();       // Total de objetos no mundo
    size_t K = colliding_pairs.size();   // Total de colisões acontecendo AGORA
    size_t threeN = 3 * N;

    // 1. Montar vetor v (3N x 1) e matriz invM (3N x 3N)
    Matrix<float> v(threeN, 1, 0.0f);
    Matrix<float> invM(threeN, threeN, 0.0f);

    for (size_t i = 0; i < N; ++i) {
        v.data[3 * i    ][0] = global_vel[i].x;
        v.data[3 * i + 1][0] = global_vel[i].y;
        v.data[3 * i + 2][0] = global_vel[i].z; // Componente Z

        float mInv = (global_mass[i] > 0.0f) ? (1.0f / global_mass[i]) : 0.0f;
        invM.data[3 * i    ][3 * i    ] = mInv;
        invM.data[3 * i + 1][3 * i + 1] = mInv;
        invM.data[3 * i + 2][3 * i + 2] = mInv; // Massa inversa para Z
    }

    // 2. Montar a matriz de contato Z (3N x K)
    Matrix<float> Z(threeN, K, 0.0f);

    for (size_t k = 0; k < K; ++k) {
        size_t idxA = colliding_pairs[k].first;
        size_t idxB = colliding_pairs[k].second;

        // Calcula a normal de contato 3D entre A e B
        glm::vec3 delta = global_pos[idxB] - global_pos[idxA];
        if (glm::length(delta) == 0.0f) delta = glm::vec3(1.0f, 0.0f, 0.0f);
        glm::vec3 norm = glm::normalize(delta);

        // Mapeamento 3D: -norm para o objeto A e +norm para o objeto B
        Z.data[3 * idxA    ][k] = -norm.x;
        Z.data[3 * idxA + 1][k] = -norm.y;
        Z.data[3 * idxA + 2][k] = -norm.z;

        Z.data[3 * idxB    ][k] = norm.x;
        Z.data[3 * idxB + 1][k] = norm.y;
        Z.data[3 * idxB + 2][k] = norm.z;
    }

    // 3. Multiplicações de matrizes (idêntico ao fluxo do post)
    auto Z_T = Z.transpose();
    auto uImp = Z_T * v;                  // Velocidades relativas (K x 1)
    auto A = Z_T * invM * Z;              // Matriz de Delassus (K x K)

    auto b_vec = uImp * -(1.0f + e);
    auto J = solveGaussSeidel(A, b_vec);  // Magnitudes dos impulsos (K x 1)

    // Truncar impulsos negativos
    // for (size_t i = 0; i < J.getRows(); ++i) {
    //     J.data[i][0] = std::max(0.0f, J.data[i][0]);
    // }

    // 4. Retorna o Delta V global em 3D (3N x 1) -> dv = invM * Z * J
    return invM * Z * J;
}