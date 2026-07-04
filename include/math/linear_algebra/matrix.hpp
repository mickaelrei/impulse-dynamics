#pragma once

#include <vector>
#include <stdexcept>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <initializer_list>

namespace linalg {

/// @brief Dense matrix with contiguous row-major storage.
///
/// Unlike a vector<vector<T>> layout, a single flat buffer keeps rows
/// adjacent in memory, which is what makes elimination and multiplication
/// loops cache-friendly and vectorizable at any real system size.
template <typename T>
class Matrix {
public:
    Matrix() : rows_(0), cols_(0) {}

    Matrix(size_t r, size_t c, const T &v = T{0})
        : rows_(r), cols_(c), data_(r * c, v) {}

    Matrix(std::initializer_list<std::initializer_list<T>> init) {
        rows_ = init.size();
        cols_ = rows_ ? init.begin()->size() : 0;
        data_.reserve(rows_ * cols_);
        for (const auto &row : init) {
            if (row.size() != cols_)
                throw std::invalid_argument("Ragged initializer list");
            data_.insert(data_.end(), row.begin(), row.end());
        }
    }

    size_t rows() const { return rows_; }
    size_t cols() const { return cols_; }

    T &operator()(size_t i, size_t j) { return data_[i * cols_ + j]; }
    const T &operator()(size_t i, size_t j) const { return data_[i * cols_ + j]; }

    T *row(size_t i) { return data_.data() + i * cols_; }
    const T *row(size_t i) const { return data_.data() + i * cols_; }

    static Matrix identity(size_t n) {
        Matrix m(n, n, T{0});
        for (size_t i = 0; i < n; ++i) m(i, i) = T{1};
        return m;
    }

    // --- Elementary row operations, used by every solver below ---

    void swapRows(size_t a, size_t b) {
        if (a == b) return;
        T *ra = row(a), *rb = row(b);
        for (size_t j = 0; j < cols_; ++j) std::swap(ra[j], rb[j]);
    }

    void scaleRow(size_t r, const T &s) {
        T *p = row(r);
        for (size_t j = 0; j < cols_; ++j) p[j] *= s;
    }

    /// row[dst] += s * row[src]
    void axpyRow(size_t dst, size_t src, const T &s) {
        T *d = row(dst);
        const T *sc = row(src);
        for (size_t j = 0; j < cols_; ++j) d[j] += s * sc[j];
    }

    Matrix transpose() const {
        Matrix t(cols_, rows_);
        for (size_t i = 0; i < rows_; ++i)
            for (size_t j = 0; j < cols_; ++j)
                t(j, i) = (*this)(i, j);
        return t;
    }

    // --- Block helpers, needed for partitioned/block solving ---

    Matrix block(size_t r0, size_t c0, size_t nr, size_t nc) const {
        Matrix m(nr, nc);
        for (size_t i = 0; i < nr; ++i)
            for (size_t j = 0; j < nc; ++j)
                m(i, j) = (*this)(r0 + i, c0 + j);
        return m;
    }

    void setBlock(size_t r0, size_t c0, const Matrix &src) {
        for (size_t i = 0; i < src.rows_; ++i)
            for (size_t j = 0; j < src.cols_; ++j)
                (*this)(r0 + i, c0 + j) = src(i, j);
    }

    // --- Arithmetic ---

    Matrix operator+(const Matrix &o) const {
        checkSameShape(o, "addition");
        Matrix r(*this);
        for (size_t i = 0; i < data_.size(); ++i) r.data_[i] += o.data_[i];
        return r;
    }

    Matrix operator-(const Matrix &o) const {
        checkSameShape(o, "subtraction");
        Matrix r(*this);
        for (size_t i = 0; i < data_.size(); ++i) r.data_[i] -= o.data_[i];
        return r;
    }

    Matrix operator-() const {
        Matrix r(*this);
        for (auto &v : r.data_) v = -v;
        return r;
    }

    /// Matrix product using ikj loop order: the inner loop walks two
    /// contiguous rows, which is where the flat layout actually pays off.
    Matrix operator*(const Matrix &o) const {
        if (cols_ != o.rows_)
            throw std::invalid_argument("Matrix dimension mismatch for multiplication");
        Matrix res(rows_, o.cols_, T{0});
        for (size_t i = 0; i < rows_; ++i) {
            const T *arow = row(i);
            T *rrow = res.row(i);
            for (size_t k = 0; k < cols_; ++k) {
                T a = arow[k];
                if (a == T{0}) continue;
                const T *brow = o.row(k);
                for (size_t j = 0; j < o.cols_; ++j) rrow[j] += a * brow[j];
            }
        }
        return res;
    }

    Matrix operator*(const T &s) const {
        Matrix r(*this);
        for (auto &v : r.data_) v *= s;
        return r;
    }

    bool operator==(const Matrix &o) const {
        return rows_ == o.rows_ && cols_ == o.cols_ && data_ == o.data_;
    }
    bool operator!=(const Matrix &o) const { return !(*this == o); }

private:
    void checkSameShape(const Matrix &o, const char *op) const {
        if (rows_ != o.rows_ || cols_ != o.cols_)
            throw std::invalid_argument(std::string("Matrix dimensions must match for ") + op);
    }

    size_t rows_, cols_;
    std::vector<T> data_;
};

template <typename T>
Matrix<T> operator*(const T &s, const Matrix<T> &m) { return m * s; }

template <typename T>
std::ostream &operator<<(std::ostream &os, const Matrix<T> &m) {
    for (size_t i = 0; i < m.rows(); ++i) {
        os << "[";
        for (size_t j = 0; j < m.cols(); ++j) {
            os << m(i, j);
            if (j + 1 != m.cols()) os << " ";
        }
        os << "]";
        if (i + 1 != m.rows()) os << "\n";
    }
    return os;
}

} // namespace linalg