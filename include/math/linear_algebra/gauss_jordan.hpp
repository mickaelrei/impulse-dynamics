#pragma once

#include "matrix.hpp"
#include <cmath>
#include <stdexcept>

namespace linalg {

/// @brief Solve A x = B for X, where B can have multiple columns (multiple
/// right-hand sides solved in a single elimination pass). This is the
/// workhorse both solveGaussJordan and the block partition solver build on.
template <typename T>
Matrix<T> solveMultiRHS(const Matrix<T> &A, const Matrix<T> &B) {
    size_t n = A.rows();
    if (A.cols() != n || B.rows() != n)
        throw std::invalid_argument("Invalid dimensions for solveMultiRHS");
    size_t m = B.cols();

    Matrix<T> aug(n, n + m);
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = 0; j < n; ++j) aug(i, j) = A(i, j);
        for (size_t j = 0; j < m; ++j) aug(i, n + j) = B(i, j);
    }

    for (size_t c = 0; c < n; ++c) {
        // Partial pivoting: pick the largest-magnitude entry in the column,
        // not just the first nonzero. This is what keeps error from blowing
        // up as the system gets larger.
        size_t piv = c;
        T best = std::abs(aug(c, c));
        for (size_t i = c + 1; i < n; ++i) {
            T v = std::abs(aug(i, c));
            if (v > best) { best = v; piv = i; }
        }
        if (best == T{0})
            throw std::runtime_error("Matrix is singular and cannot be solved");
        if (piv != c) aug.swapRows(piv, c);

        T inv = T{1} / aug(c, c);
        aug.scaleRow(c, inv);

        for (size_t i = 0; i < n; ++i) {
            if (i == c) continue;
            T factor = aug(i, c);
            if (factor != T{0}) aug.axpyRow(i, c, -factor);
        }
    }

    Matrix<T> X(n, m);
    for (size_t i = 0; i < n; ++i)
        for (size_t j = 0; j < m; ++j)
            X(i, j) = aug(i, n + j);
    return X;
}

/// @brief Solve A x = b via full Gauss-Jordan elimination (single RHS).
template <typename T>
Matrix<T> solveGaussJordan(const Matrix<T> &A, const Matrix<T> &b) {
    if (b.cols() != 1)
        throw std::invalid_argument("solveGaussJordan expects a single-column b");
    return solveMultiRHS(A, b);
}

/// @brief Invert A by solving A X = I. Prefer solving the actual system you
/// need over forming the inverse explicitly when you can -- it's cheaper
/// and more numerically stable -- but this is here when you truly need A^-1.
template <typename T>
Matrix<T> invert(const Matrix<T> &A) {
    if (A.rows() != A.cols())
        throw std::invalid_argument("Inverse is defined only for square matrices");
    return solveMultiRHS(A, Matrix<T>::identity(A.rows()));
}

} // namespace linalg