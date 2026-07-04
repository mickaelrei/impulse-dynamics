#pragma once

#include "matrix.hpp"
#include "gauss_jordan.hpp"

namespace linalg {

/// @brief Solve A x = b by 2x2 block partitioning (Schur complement).
///
/// A is split at index k into:
///     A = [ A11  A12 ]     b = [ b1 ]
///         [ A21  A22 ]         [ b2 ]
/// where A11 is k x k.
///
/// Elimination reduces to a smaller (n-k) x (n-k) system in the Schur
/// complement S = A22 - A21 * A11^-1 * A12, then back-substitutes:
///     x1 = A11^-1 b1 - A11^-1 A12 * x2
///     S x2 = b2 - A21 * A11^-1 b1
///
/// This is the primitive for splitting a large system into smaller pieces --
/// useful when A has natural block structure (e.g. coupling a small number
/// of "interface" unknowns to large, mostly-independent sub-blocks), and it
/// composes: solveMultiRHS/solveGaussJordan on A11 or S can themselves be
/// swapped for a recursive block solve if that block is still large.
template <typename T>
Matrix<T> solveBlockPartition(const Matrix<T> &A, const Matrix<T> &b, size_t k) {
    size_t n = A.rows();
    if (A.cols() != n || b.rows() != n || b.cols() != 1)
        throw std::invalid_argument("Invalid dimensions for block partition solve");
    if (k == 0 || k >= n)
        throw std::invalid_argument("Partition index must split A into two non-empty blocks");

    size_t n2 = n - k;

    Matrix<T> A11 = A.block(0, 0, k, k);
    Matrix<T> A12 = A.block(0, k, k, n2);
    Matrix<T> A21 = A.block(k, 0, n2, k);
    Matrix<T> A22 = A.block(k, k, n2, n2);
    Matrix<T> B1  = b.block(0, 0, k, 1);
    Matrix<T> B2  = b.block(k, 0, n2, 1);

    // One elimination pass on A11 handles both A11^-1*A12 and A11^-1*b1.
    Matrix<T> rhs(k, n2 + 1);
    rhs.setBlock(0, 0, A12);
    rhs.setBlock(0, n2, B1);
    Matrix<T> Y = solveMultiRHS(A11, rhs);
    Matrix<T> A11invA12 = Y.block(0, 0, k, n2);
    Matrix<T> A11invB1  = Y.block(0, n2, k, 1);

    Matrix<T> S    = A22 - (A21 * A11invA12);
    Matrix<T> rhs2 = B2 - (A21 * A11invB1);

    Matrix<T> x2 = solveGaussJordan(S, rhs2);
    Matrix<T> x1 = A11invB1 - (A11invA12 * x2);

    Matrix<T> x(n, 1);
    x.setBlock(0, 0, x1);
    x.setBlock(k, 0, x2);
    return x;
}

} // namespace linalg