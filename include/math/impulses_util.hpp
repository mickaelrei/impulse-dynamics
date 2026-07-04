#include <vector>
#include <algorithm>

#include <glm/glm.hpp>

#include "linear_algebra/matrix.hpp"
#include "linear_algebra/gauss_jordan.hpp"
#include "linear_algebra/gauss_seidel.hpp"
#include "linear_algebra/block_solver.hpp"

using linalg::Matrix;

Matrix<float> resolveSimultaneousCollisions3D(
    const std::vector<glm::vec3> &global_pos,
    const std::vector<glm::vec3> &global_vel,
    const std::vector<float> &global_mass,
    const std::vector<std::pair<size_t, size_t>> &colliding_pairs,
    float e = 1.0f
) {
    size_t N = global_mass.size();
    size_t K = colliding_pairs.size();
    size_t threeN = 3 * N;

    Matrix<float> v(threeN, 1, 0.0f);
    Matrix<float> invM(threeN, threeN, 0.0f);

    for (size_t i = 0; i < N; ++i) {
        v(3 * i,     0) = global_vel[i].x;
        v(3 * i + 1, 0) = global_vel[i].y;
        v(3 * i + 2, 0) = global_vel[i].z;

        float mInv = (global_mass[i] > 0.0f) ? (1.0f / global_mass[i]) : 0.0f;
        invM(3 * i,     3 * i)     = mInv;
        invM(3 * i + 1, 3 * i + 1) = mInv;
        invM(3 * i + 2, 3 * i + 2) = mInv;
    }

    Matrix<float> Z(threeN, K, 0.0f);

    for (size_t k = 0; k < K; ++k) {
        size_t idxA = colliding_pairs[k].first;
        size_t idxB = colliding_pairs[k].second;

        glm::vec3 delta = global_pos[idxB] - global_pos[idxA];
        if (glm::length(delta) == 0.0f) delta = glm::vec3(1.0f, 0.0f, 0.0f);
        glm::vec3 norm = glm::normalize(delta);

        Z(3 * idxA,     k) = -norm.x;
        Z(3 * idxA + 1, k) = -norm.y;
        Z(3 * idxA + 2, k) = -norm.z;

        Z(3 * idxB,     k) = norm.x;
        Z(3 * idxB + 1, k) = norm.y;
        Z(3 * idxB + 2, k) = norm.z;
    }

    auto Z_T = Z.transpose();
    auto uImp = Z_T * v;
    auto A = Z_T * invM * Z;

    auto b_vec = uImp * -(1.0f + e);

    // Gauss-Seidel
    // Vale checar `result.converged` se a estabilidade virar problema
    // auto result = linalg::solveGaussSeidel(A, b_vec, 1e-5f, 100);
    // Matrix<float> J = result.x;

    // Gauss-Jordan
    Matrix<float> J = linalg::solveGaussJordan(A, b_vec);

    // Particionado por bloco
    // size_t k_split = K / 2;
    // Matrix<float> J = linalg::solveBlockPartition(A, b_vec, k_split);

    // Truncar impulsos negativos
    // for (size_t i = 0; i < J.rows(); ++i) {
    //     J(i, 0) = std::max(0.0f, J(i, 0));
    // }

    return invM * Z * J;
}