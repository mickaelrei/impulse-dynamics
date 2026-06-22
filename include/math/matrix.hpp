#pragma once

#include <cstdlib>
#include <iostream>
#include <cassert>
#include <vector>
#include <cmath>
#include <stdexcept>

/// @brief Matrix class with dynamic dimensions
/// @tparam T Matrix data type
template <typename T>
class Matrix {
private:
    size_t rows;
    size_t cols;

public:
    // Expor o contêiner interno via alias para facilitar inicializações complexas
    using array2d = std::vector<std::vector<T>>;
    
    // Armazenamento dinâmico dos dados
    array2d data;

    /// @brief Constructor with dimensions and initial cell value
    Matrix(size_t r = 0, size_t c = 0, const T &v = T{0});

    /// @brief Constructor from 2D vector
    Matrix(const array2d &initialData);

    /// @brief Copy constructor
    Matrix(const Matrix<T> &m);

    /// @brief Destructor
    ~Matrix();

    // Getters para as dimensões dinâmicas
    size_t getRows() const { return rows; }
    size_t getCols() const { return cols; }

    /// @brief Transpose of this matrix
    Matrix<T> transpose() const;

    /// @brief Calculate this matrix's determinant
    T determinant() const;

    /// @brief Tries to calculate inverse of this matrix.
    Matrix<T> inverse() const;

    /// @brief Copy assign operator
    Matrix<T> &operator=(const Matrix<T> &m);

    /// @brief Equal check operator
    bool operator==(const Matrix<T> &m) const;

    /// @brief Not equal check operator
    bool operator!=(const Matrix<T> &m) const;

    /// @brief Addition of 2 matrices
    Matrix<T> operator+(const Matrix<T> &m) const;
    Matrix<T> &operator+=(const Matrix<T> &m);

    /// @brief Difference of 2 matrices
    Matrix<T> operator-(const Matrix<T> &m) const;
    Matrix<T> &operator-=(const Matrix<T> &m);

    /// @brief Negative matrix operator
    Matrix<T> operator-() const;

    /// @brief Product of 2 matrices
    Matrix<T> operator*(const Matrix<T> &m) const;
    Matrix<T> &operator*=(const Matrix<T> &m);

    /// @brief Product of scalar and matrix
    Matrix<T> operator*(const T &s) const;
    Matrix<T> &operator*=(const T &s);

    /// @brief Division of matrix by scalar
    Matrix<T> operator/(const T &s) const;
    Matrix<T> &operator/=(const T &s);

    /// @brief Identity matrix
    static Matrix<T> identity(size_t size);

    /// @brief Elementary operation - swap two rows
    void swapRows(size_t r0, size_t r1);

    /// @brief Elementary operation - multiply row by scalar
    void multiplyRow(size_t r, const T &s);

    /// @brief Elementary operation - add on row another row multiplied by scalar
    void addScaledRow(size_t r0, size_t r1, const T &s = T{1});
};

// --- Funções Globais / Livres ---

template <typename T>
Matrix<T> operator*(const T &s, const Matrix<T> &m) {
    return m * s;
}

template <typename T>
std::ostream &operator<<(std::ostream &os, const Matrix<T> &m) {
    for (size_t i = 0; i < m.getRows(); ++i) {
        os << "[";
        for (size_t j = 0; j < m.getCols(); ++j) {
            os << m.data[i][j];
            if (j != m.getCols() - 1) os << " ";
        }
        os << "]";
        if (i != m.getRows() - 1) os << "\n";
    }
    return os;
}

// --- Implementação dos Métodos ---

template <typename T>
Matrix<T>::Matrix(size_t r, size_t c, const T &v) 
    : rows(r), cols(c), data(r, std::vector<T>(c, v)) {}

template <typename T>
Matrix<T>::Matrix(const array2d &initialData) {
    rows = initialData.size();
    cols = rows > 0 ? initialData[0].size() : 0;
    data = initialData;
}

template <typename T>
Matrix<T>::Matrix(const Matrix<T> &m) 
    : rows(m.rows), cols(m.cols), data(m.data) {}

template <typename T>
Matrix<T>::~Matrix() {}

template <typename T>
Matrix<T> Matrix<T>::identity(size_t size) {
    Matrix<T> m(size, size, T{0});
    for (size_t i = 0; i < size; ++i) {
        m.data[i][i] = T{1};
    }
    return m;
}

template <typename T>
Matrix<T> Matrix<T>::transpose() const {
    Matrix<T> m(cols, rows);
    for (size_t j = 0; j < cols; ++j) {
        for (size_t i = 0; i < rows; ++i) {
            m.data[j][i] = data[i][j];
        }
    }
    return m;
}

// Helper interno para cálculo recursivo de Laplace (já que R-1 e C-1 não funcionam mais em templates)
template <typename T>
T lapLaceDeterminantInternal(const Matrix<T> &m) {
    size_t r = m.getRows();
    if (r == 0) return 0;
    if (r == 1) return m.data[0][0];

    T det = T{0};
    for (size_t i = 0; i < r; ++i) {
        if (m.data[0][i] == 0) continue;

        // Construir submatriz (r-1) x (r-1)
        Matrix<T> n(r - 1, r - 1);
        for (size_t j = 0; j < r - 1; ++j) {
            for (size_t k = 0; k < r - 1; ++k) {
                size_t col = k < i ? k : k + 1;
                n.data[j][k] = m.data[j + 1][col];
            }
        }
        det += T{i & 1 ? -1 : 1} * m.data[0][i] * lapLaceDeterminantInternal(n);
    }
    return det;
}

template <typename T>
T rowReductionDeterminant(const Matrix<T> &m) {
    if (m.getRows() != m.getCols()) {
        throw std::invalid_argument("Determinant is defined only for square matrices");
    }

    Matrix<T> tmp{m};
    T scale = T{1};
    size_t C = m.getCols();
    size_t R = m.getRows();

    for (size_t c = 0; c < C; ++c) {
        size_t row = -1;
        for (size_t i = c; i < R; ++i) {
            if (tmp.data[i][c] != T{0}) {
                row = i;
                break;
            }
        }
        if (row == -1UL) return T{0};

        if (row != c) {
            tmp.swapRows(row, c);
            scale *= T{-1};
        }

        T rowScale = tmp.data[c][c];
        if (rowScale != T{1}) {
            T mult = T{1} / rowScale;
            tmp.multiplyRow(c, mult);
            scale *= rowScale;
        }

        for (size_t i = c + 1; i < R; ++i) {
            T elem = tmp.data[i][c];
            if (elem == T{0}) continue;
            tmp.addScaledRow(i, c, -elem);
        }
    }
    return scale;
}

template <typename T>
T Matrix<T>::determinant() const {
    return rowReductionDeterminant(*this);
    // Para usar Laplace, seria: return lapLaceDeterminantInternal(*this);
}

template <typename T>
Matrix<T> Matrix<T>::inverse() const {
    if (rows != cols) {
        throw std::invalid_argument("Inverse is defined only for square matrices");
    }

    // Criar matriz aumentada (R, 2C)
    Matrix<T> inv(rows, 2 * cols, T{0});
    for (size_t i = 0; i < rows; ++i) {
        for (size_t j = 0; j < 2 * cols; ++j) {
            if (j < cols) {
                inv.data[i][j] = data[i][j];
            } else {
                inv.data[i][j] = (i == j - cols) ? T{1} : T{0};
            }
        }
    }

    for (size_t c = 0; c < cols; ++c) {
        size_t row = -1;
        for (size_t i = c; i < rows; ++i) {
            if (inv.data[i][c] != T{0}) {
                row = i;
                break;
            }
        }
        if (row == -1UL) {
            throw std::runtime_error("Matrix is singular and cannot be inverted.");
        }

        if (row != c) {
            inv.swapRows(row, c);
        }

        T elem = inv.data[c][c];
        if (elem != T{1}) {
            T mult = T{1} / elem;
            inv.multiplyRow(c, mult);
        }

        for (size_t i = 0; i < rows; ++i) {
            if (i == c) continue;
            T currentElem = inv.data[i][c];
            if (currentElem == T{0}) continue;
            inv.addScaledRow(i, c, -currentElem);
        }
    }

    Matrix<T> res(rows, cols);
    for (size_t i = 0; i < rows; ++i) {
        for (size_t j = 0; j < cols; ++j) {
            res.data[i][j] = inv.data[i][j + cols];
        }
    }
    return res;
}

// --- Solvers de Sistemas Lineares ---

template <typename T>
Matrix<T> solveGaussJordan(const Matrix<T> &A, const Matrix<T> &b) {
    if (A.getRows() != A.getCols() || A.getRows() != b.getRows() || b.getCols() != 1) {
        throw std::invalid_argument("Invalid dimensions for Gauss-Jordan solver");
    }

    size_t N = A.getRows();
    Matrix<T> aug(N, N + 1);

    for (size_t i = 0; i < N; ++i) {
        for (size_t j = 0; j < N; ++j) {
            aug.data[i][j] = A.data[i][j];
        }
        aug.data[i][N] = b.data[i][0];
    }

    for (size_t c = 0; c < N; ++c) {
        size_t pivot = -1;
        for (size_t i = c; i < N; ++i) {
            if (aug.data[i][c] != T{0}) {
                pivot = i;
                break;
            }
        }

        if (pivot == -1UL) {
            throw std::runtime_error("System has no unique solution");
        }

        if (pivot != c) aug.swapRows(pivot, c);

        T val = aug.data[c][c];
        if (val != T{1}) aug.multiplyRow(c, T{1} / val);

        for (size_t i = 0; i < N; ++i) {
            if (i == c) continue;
            T factor = aug.data[i][c];
            if (factor != T{0}) aug.addScaledRow(i, c, -factor);
        }
    }

    Matrix<T> x(N, 1);
    for (size_t i = 0; i < N; ++i) {
        x.data[i][0] = aug.data[i][N];
    }
    return x;
}

template <typename T>
Matrix<T> solveGaussSeidel(const Matrix<T> &A, const Matrix<T> &b) {
    if (A.getRows() != A.getCols() || A.getRows() != b.getRows() || b.getCols() != 1) {
        throw std::invalid_argument("Invalid dimensions for Gauss-Seidel solver");
    }

    size_t N = A.getRows();
    Matrix<T> x(N, 1, T{0});

    size_t attempt = 0;
    constexpr size_t MAX_ATTEMPTS = 30;
    while (attempt < MAX_ATTEMPTS) {
        T maxChange{0};

        for (size_t i = 0; i < N; i++) {
            T v{0};
            for (size_t j = 0; j < N; j++) {
                if (i != j) v += A.data[i][j] * x.data[j][0];
            }

            T newVal = (b.data[i][0] - v) / A.data[i][i];
            if (i == 0 || maxChange > x.data[i][0] - newVal) {
                maxChange = std::abs(x.data[i][0] - newVal);
            }
            assert(newVal < 0.0f);
            // if (newVal < 0.0f) printf("newVal < 0 %.3f\n", newVal);
            x.data[i][0] = newVal;
        }

        if (maxChange < 1e-5) break;
        attempt++;
    }
    return x;
}

// --- Operadores de Atribuição e Aritméticos ---

template <typename T>
Matrix<T> &Matrix<T>::operator=(const Matrix<T> &m) {
    if (this != &m) {
        rows = m.rows;
        cols = m.cols;
        data = m.data;
    }
    return *this;
}

template <typename T>
bool Matrix<T>::operator==(const Matrix<T> &m) const {
    return (rows == m.rows && cols == m.cols && data == m.data);
}

template <typename T>
bool Matrix<T>::operator!=(const Matrix<T> &m) const {
    return !(*this == m);
}

template <typename T>
Matrix<T> Matrix<T>::operator+(const Matrix<T> &m) const {
    Matrix<T> tmp{*this};
    tmp += m;
    return tmp;
}

template <typename T>
Matrix<T> &Matrix<T>::operator+=(const Matrix<T> &m) {
    if (rows != m.rows || cols != m.cols) throw std::invalid_argument("Matrix dimensions must match for addition");
    for (size_t i = 0; i < rows; ++i) {
        for (size_t j = 0; j < cols; ++j) {
            data[i][j] += m.data[i][j];
        }
    }
    return *this;
}

template <typename T>
Matrix<T> Matrix<T>::operator-(const Matrix<T> &m) const {
    Matrix<T> tmp{*this};
    tmp -= m;
    return tmp;
}

template <typename T>
Matrix<T> &Matrix<T>::operator-=(const Matrix<T> &m) {
    return *this += -m;
}

template <typename T>
Matrix<T> Matrix<T>::operator-() const {
    Matrix<T> m{rows, cols};
    for (size_t i = 0; i < rows; ++i) {
        for (size_t j = 0; j < cols; ++j) {
            m.data[i][j] = -data[i][j];
        }
    }
    return m;
}

// Multiplicação de Matrizes (Tamanho dinâmico verificado em tempo de execução)
template <typename T>
Matrix<T> Matrix<T>::operator*(const Matrix<T> &m) const {
    if (cols != m.rows) {
        throw std::invalid_argument("Matrix dimension mismatch for multiplication");
    }

    Matrix<T> res(rows, m.cols, T{0});
    for (size_t i = 0; i < rows; ++i) {
        for (size_t j = 0; j < m.cols; ++j) {
            for (size_t k = 0; k < cols; ++k)
                res.data[i][j] += data[i][k] * m.data[k][j];
        }
    }
    return res;
}

template <typename T>
Matrix<T> &Matrix<T>::operator*=(const Matrix<T> &m) {
    *this = *this * m;
    return *this;
}

template <typename T>
Matrix<T> Matrix<T>::operator*(const T &s) const {
    Matrix<T> tmp{*this};
    tmp *= s;
    return tmp;
}

template <typename T>
Matrix<T> &Matrix<T>::operator*=(const T &s) {
    for (size_t i = 0; i < rows; ++i) {
        for (size_t j = 0; j < cols; ++j) {
            data[i][j] *= s;
        }
    }
    return *this;
}

template <typename T>
Matrix<T> Matrix<T>::operator/(const T &s) const {
    Matrix<T> tmp{*this};
    tmp /= s;
    return tmp;
}

template <typename T>
Matrix<T> &Matrix<T>::operator/=(const T &s) {
    return *this *= (T{1} / s);
}

// --- Métodos de Operações Elementares ---

template <typename T>
void Matrix<T>::swapRows(size_t r0, size_t r1) {
    std::swap(data[r0], data[r1]); 
}

template <typename T>
void Matrix<T>::multiplyRow(size_t r, const T &s) {
    for (size_t i = 0; i < cols; ++i) {
        data[r][i] *= s;
    }
}

template <typename T>
void Matrix<T>::addScaledRow(size_t r0, size_t r1, const T &s) {
    for (size_t i = 0; i < cols; ++i) {
        data[r0][i] += s * data[r1][i];
    }
}