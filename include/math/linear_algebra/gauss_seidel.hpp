#pragma once

#include "matrix.hpp"
#include <cmath>
#include <stdexcept>

namespace linalg {

template <typename T>
struct IterativeResult {
    Matrix<T> x;
    size_t iterations;
    bool converged;
};

/// @brief Solve A x = b iteratively with Gauss-Seidel / SOR.
///
/// @param tol      stop when the largest per-component change drops below this
/// @param maxIter  hard cap on sweeps, in case the system doesn't converge
/// @param omega    relaxation factor. 1.0 = plain Gauss-Seidel, (1,2) = SOR
///                 (can speed up convergence for well-behaved systems),
///                 (0,1) = under-relaxation (can rescue mildly ill-behaved ones)
///
/// Note: convergence is only guaranteed for diagonally dominant or symmetric
/// positive-definite A. This does not check for that -- it will happily
/// diverge on a system that doesn't satisfy it, so check `converged` and
/// `iterations` on the result rather than assuming success.
template <typename T>
IterativeResult<T> solveGaussSeidel(const Matrix<T> &A, const Matrix<T> &b,
                                     T tol = T{1e-8}, size_t maxIter = 1000,
                                     T omega = T{1}) {
    size_t n = A.rows();
    if (A.cols() != n || b.rows() != n || b.cols() != 1)
        throw std::invalid_argument("Invalid dimensions for Gauss-Seidel solver");

    for (size_t i = 0; i < n; ++i) {
        if (A(i, i) == T{0})
            throw std::runtime_error("Zero on diagonal; reorder equations or use a direct solver");
    }

    Matrix<T> x(n, 1, T{0});

    size_t iter = 0;
    bool converged = false;
    for (; iter < maxIter; ++iter) {
        T maxChange = T{0};

        for (size_t i = 0; i < n; ++i) {
            T sum = T{0};
            for (size_t j = 0; j < n; ++j) {
                if (j != i) sum += A(i, j) * x(j, 0);
            }
            T gaussSeidelVal = (b(i, 0) - sum) / A(i, i);
            T oldVal = x(i, 0);
            T newVal = oldVal + omega * (gaussSeidelVal - oldVal); // SOR step

            T change = std::abs(newVal - oldVal);
            if (change > maxChange) maxChange = change;

            x(i, 0) = newVal;
        }

        if (maxChange < tol) { converged = true; ++iter; break; }
    }

    return IterativeResult<T>{x, iter, converged};
}

} // namespace linalg