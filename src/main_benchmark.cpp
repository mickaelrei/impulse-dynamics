#include "../include/math/linear_algebra/matrix.hpp"
#include "../include/math/linear_algebra/gauss_jordan.hpp"
#include "../include/math/linear_algebra/gauss_seidel.hpp"
#include "../include/math/linear_algebra/block_solver.hpp"

#include <iostream>
#include <iomanip>
#include <random>
#include <chrono>
#include <vector>

using namespace linalg;
using Clock = std::chrono::steady_clock;

// Build a random, diagonally dominant system (needed so Gauss-Seidel
// actually converges -- an arbitrary random matrix would not).
static void buildSystem(size_t n, unsigned seed, Matrix<double> &A, Matrix<double> &b) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> dist(-1.0, 1.0);

    A = Matrix<double>(n, n);
    for (size_t i = 0; i < n; ++i)
        for (size_t j = 0; j < n; ++j)
            A(i, j) = dist(rng);

    // Force strict diagonal dominance: |A(i,i)| > sum of the rest of the row.
    for (size_t i = 0; i < n; ++i) {
        double rowSum = 0.0;
        for (size_t j = 0; j < n; ++j) if (j != i) rowSum += std::abs(A(i, j));
        A(i, i) = rowSum + 1.0 + std::abs(dist(rng)); // margin above the bound
    }

    b = Matrix<double>(n, 1);
    for (size_t i = 0; i < n; ++i) b(i, 0) = dist(rng);
}

static double residualNorm(const Matrix<double> &A, const Matrix<double> &x, const Matrix<double> &b) {
    Matrix<double> r = A * x - b;
    double s = 0.0;
    for (size_t i = 0; i < r.rows(); ++i) s += r(i, 0) * r(i, 0);
    return std::sqrt(s);
}

struct Result {
    std::string name;
    double ms;
    double residual;
    std::string extra;
};

int main() {
    std::vector<size_t> sizes = {100, 300, 600, 1000, 1500};

    std::cout << std::left
               << std::setw(6)  << "n"
               << std::setw(18) << "solver"
               << std::setw(14) << "time (ms)"
               << std::setw(16) << "residual ||Ax-b||"
               << "extra\n";
    std::cout << std::string(70, '-') << "\n";

    for (size_t n : sizes) {
        Matrix<double> A, b;
        buildSystem(n, /*seed=*/ (unsigned)n * 7919u + 1, A, b);

        std::vector<Result> results;

        // --- Gauss-Jordan (direct) ---
        {
            auto t0 = Clock::now();
            Matrix<double> x = solveGaussJordan(A, b);
            auto t1 = Clock::now();
            double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
            results.push_back({"Gauss-Jordan", ms, residualNorm(A, x, b), ""});
        }

        // --- Gauss-Seidel (iterative) ---
        {
            auto t0 = Clock::now();
            auto r = solveGaussSeidel(A, b, 1e-10, 2000);
            auto t1 = Clock::now();
            double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
            std::string extra = "iters=" + std::to_string(r.iterations)
                               + (r.converged ? "" : " (NOT CONVERGED)");
            results.push_back({"Gauss-Seidel", ms, residualNorm(A, r.x, b), extra});
        }

        // --- Block partition (Schur complement, split down the middle) ---
        {
            size_t k = n / 2;
            auto t0 = Clock::now();
            Matrix<double> x = solveBlockPartition(A, b, k);
            auto t1 = Clock::now();
            double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
            results.push_back({"Block partition", ms, residualNorm(A, x, b), "k=" + std::to_string(k)});
        }

        for (auto &r : results) {
            std::cout << std::left
                       << std::setw(6)  << n
                       << std::setw(18) << r.name
                       << std::setw(14) << std::fixed << std::setprecision(3) << r.ms
                       << std::setw(16) << std::scientific << std::setprecision(2) << r.residual
                       << r.extra << "\n";
        }
        std::cout << std::string(70, '-') << "\n";
    }

    return 0;
}