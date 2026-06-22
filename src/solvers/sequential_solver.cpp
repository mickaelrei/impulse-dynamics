#include "solvers/sequential_solver.hpp"

#include <glm/glm.hpp>

void SequentialSolver::solve1D(
    glm::vec3 v1, float m1,
    glm::vec3 v2, float m2,
    float restitution,
    glm::vec3* out1, glm::vec3* out2
) {
    const float e = restitution;
    *out1 = (m2 * v2 * (1.f + e) + v1 * (m1 - e * m2)) / (m1 + m2);
    *out2 = (m1 * v1 * (1.f + e) + v2 * (m2 - e * m1)) / (m1 + m2);
}

void SequentialSolver::solvePair(
    glm::vec3 p1, glm::vec3 v1, float m1,
    glm::vec3 p2, glm::vec3 v2, float m2,
    float restitution,
    glm::vec3* out1, glm::vec3* out2
) {
    glm::vec3 n = glm::normalize(p2 - p1);
    const glm::vec3 vn1 = glm::dot(v1, n) * n;
    const glm::vec3 vn2 = glm::dot(v2, n) * n;
    const glm::vec3 vt1 = v1 - vn1;
    const glm::vec3 vt2 = v2 - vn2;

    glm::vec3 vn1Out, vn2Out;
    solve1D(vn1, m1, vn2, m2, restitution, &vn1Out, &vn2Out);
    *out1 = vn1Out + vt1;
    *out2 = vn2Out + vt2;
}

void SequentialSolver::resolve(
    std::span<Body> bodies,
    const std::vector<std::pair<size_t, size_t>>& pairs,
    float restitution
) {
    for (const auto& [i, j] : pairs) {
        glm::vec3 v1Out, v2Out;
        solvePair(
            bodies[i].position, bodies[i].velocity, bodies[i].mass,
            bodies[j].position, bodies[j].velocity, bodies[j].mass,
            restitution,
            &v1Out, &v2Out
        );
        bodies[i].velocity = v1Out;
        bodies[j].velocity = v2Out;
    }
}
