#pragma once

#include <iostream>
#include <algorithm>
#include <array>
#include <initializer_list>
#include <stdexcept>
#include <type_traits>
#include <cmath>
#include "Pacific.h"

const Float Epsilon = 1e-4f;
const Float Pi = 3.14159265358979323846;
const Float InvPi = 0.31830988618379067154;
const Float Inv2Pi = 0.15915494309189533577;
const Float Inv4Pi = 0.07957747154594766788;
const Float PiOver2 = 1.57079632679489661923;
const Float PiOver4 = 0.78539816339744830961;
const Float Sqrt2 = 1.41421356237309504880;

#pragma region Vec

template<typename T, size_t N>
class Vec
{
private:
    T data[N];

public:
    Vec() {
        for (size_t i = 0; i < N; i++)
            data[i] = T(0);
    }
    Vec(T x) {
        for (size_t i = 0; i < N; i++)
            data[i] = x;
    }
    template <size_t M = N, std::enable_if_t<M == 2, int> = 0>
    Vec(T x, T y) {
        data[0] = x;
        data[1] = y;
    }
    template <size_t M = N, std::enable_if_t<M == 3, int> = 0>
    Vec(T x, T y, T z) {
        data[0] = x;
        data[1] = y;
        data[2] = z;
    }
    template <size_t M = N, std::enable_if_t<M == 4, int> = 0>
    Vec(T x, T y, T z, T w) {
        data[0] = x;
        data[1] = y;
        data[2] = z;
        data[3] = w;
    }
    // Default copy constructor
    Vec(const Vec& other) = default;
    ~Vec() = default;

    inline T& x() requires (N >= 1) { return data[0]; }
    inline const T& x() const requires (N >= 1) { return data[0]; }

    inline T& y() requires (N >= 2) { return data[1]; }
    inline const T& y() const requires (N >= 2) { return data[1]; }

    inline T& z() requires (N >= 3) { return data[2]; }
    inline const T& z() const requires (N >= 3) { return data[2]; }

    inline T& w() requires (N >= 4) { return data[3]; }
    inline const T& w() const requires (N >= 4) { return data[3]; }

    // Default (member-wise) copy
    Vec& operator=(const Vec& other) = default;
    Vec& operator=(T scalar) {
        for (size_t i = 0; i < N; i++)
            data[i] = scalar;
        return *this;
    }

    // Array access operators
    T& operator[](size_t index) {
        if (index >= N)
            throw std::runtime_error("Index out of range.");
        return data[index];
    }

    const T& operator[](size_t index) const {
        if (index >= N)
            throw std::runtime_error("Index out of range.");
        return data[index];
    }

    // Unary operators
    Vec operator+() const {
        return *this;
    }

    Vec operator-() const {
        Vec temp = *this;
        for (size_t i = 0; i < N; i++)
            temp[i] = -temp[i];
        return temp;
    }

    // Binary arithmetic (vec with vec)
    Vec operator+(const Vec& other) const {
        Vec result;
        for (size_t i = 0; i < N; ++i)
            result[i] = data[i] + other.data[i];
        return result;
    }

    Vec operator-(const Vec& other) const {
        Vec result;
        for (size_t i = 0; i < N; ++i)
            result[i] = data[i] - other.data[i];
        return result;
    }

    Vec operator*(const Vec& other) const {
        Vec result;
        for (size_t i = 0; i < N; ++i)
            result[i] = data[i] * other.data[i];
        return result;
    }

    Vec operator/(const Vec& other) const {
        Vec result;
        for (size_t i = 0; i < N; ++i)
            result[i] = data[i] / other.data[i];
        return result;
    }

    // Binary arithmetic (vec with scalar)
    Vec operator+(T scalar) const {
        Vec result;
        for (size_t i = 0; i < N; ++i)
            result[i] = data[i] + scalar;
        return result;
    }

    Vec operator-(T scalar) const {
        Vec result;
        for (size_t i = 0; i < N; ++i)
            result[i] = data[i] - scalar;
        return result;
    }

    Vec operator*(T scalar) const {
        Vec result;
        for (size_t i = 0; i < N; ++i)
            result[i] = data[i] * scalar;
        return result;
    }

    Vec operator/(T scalar) const {
        Vec result;
        for (size_t i = 0; i < N; ++i)
            result[i] = data[i] / scalar;
        return result;
    }

    // Compound assignment operators with Vec
    Vec& operator+=(const Vec& other) {
        for (size_t i = 0; i < N; i++)
            data[i] += other.data[i];
        return *this;
    }

    Vec& operator-=(const Vec& other) {
        for (size_t i = 0; i < N; i++)
            data[i] -= other.data[i];
        return *this;
    }

    Vec& operator*=(const Vec& other) {
        for (size_t i = 0; i < N; i++)
            data[i] *= other.data[i];
        return *this;
    }

    Vec& operator/=(const Vec& other) {
        for (size_t i = 0; i < N; i++)
            data[i] /= other.data[i];
        return *this;
    }

    // Compound assignment operators with scalar
    Vec& operator+=(T scalar) {
        for (size_t i = 0; i < N; i++)
            data[i] += scalar;
        return *this;
    }

    Vec& operator-=(T scalar) {
        for (size_t i = 0; i < N; i++)
            data[i] -= scalar;
        return *this;
    }

    Vec& operator*=(T scalar) {
        for (size_t i = 0; i < N; i++)
            data[i] *= scalar;
        return *this;
    }

    Vec& operator/=(T scalar) {
        for (size_t i = 0; i < N; i++)
            data[i] /= scalar;
        return *this;
    }

    // Comparison operators
    bool operator==(const Vec& other) const {
        if constexpr (std::is_floating_point_v<T>) {
            for (size_t i = 0; i < N; i++)
                if (std::abs(data[i] - other.data[i]) >= Epsilon)
                    return false;
            return true;
        }
        else {
            for (size_t i = 0; i < N; i++)
                if (data[i] != other.data[i])
                    return false;
            return true;
        }
    }

    bool operator!=(const Vec& other) const {
        return !(*this == other);
    }
};

// Binary arithmetic operators - scalar with Vec (left side)
template<typename T, size_t N>
inline Vec<T, N> operator+(T scalar, const Vec<T, N>& vec) {
    Vec<T, N> result;
    for (size_t i = 0; i < N; i++)
        result[i] = scalar + vec.data[i];
    return result;
}

template<typename T, size_t N>
inline Vec<T, N> operator-(T scalar, const Vec<T, N>& vec) {
    Vec<T, N> result;
    for (size_t i = 0; i < N; i++)
        result[i] = scalar - vec.data[i];
    return result;
}

template<typename T, size_t N>
inline Vec<T, N> operator*(T scalar, const Vec<T, N>& vec) {
    Vec<T, N> result;
    for (size_t i = 0; i < N; i++)
        result[i] = scalar * vec[i];
    return result;
}

template<typename T, size_t N>
inline Vec<T, N> operator/(T scalar, const Vec<T, N>& vec) {
    Vec<T, N> result;
    for (size_t i = 0; i < N; i++)
        result[i] = scalar / vec.data[i];
    return result;
}

// Stream operators
template<typename T, size_t N>
inline std::ostream& operator<<(std::ostream& os, const Vec<T, N>& vec) {
    os << "(";
    for (int i = 0; i < N - 1; i++)
        os << vec[i] << ", ";
    os << vec[N - 1] << ")";
    return os;
}

template<typename T, size_t N>
inline std::istream& operator>>(std::istream& is, Vec<T, N>& vec) {
    for (size_t i = 0; i < N; i++)
        is >> vec.data[i];
    return is;
}

#pragma endregion
using Vec2f = Vec<Float, 2>;
using Vec3f = Vec<Float, 3>;
using Vec4f = Vec<Float, 4>;
using Vec2i = Vec<Int,   2>;
using Vec3i = Vec<Int,   3>;
using Vec4i = Vec<Int,   4>;

#pragma region Matrix

template<typename T, size_t Rows, size_t Cols>
class Matrix {
private:
    std::array<T, Rows * Cols> data;

public:
    // Type aliases
    using value_type = T;
    static constexpr size_t rows = Rows;
    static constexpr size_t cols = Cols;
    static constexpr size_t size = Rows * Cols;

    // Constructors
    Matrix() : data{} {} // Zero-initialize

    // Initialize from nested initializer list
    Matrix(std::initializer_list<std::initializer_list<T>> list) : data{} {
        if (list.size() != Rows) {
            throw std::invalid_argument("Wrong number of rows in initializer");
        }

        size_t row = 0;
        for (const auto& row_list : list) {
            if (row_list.size() != Cols) {
                throw std::invalid_argument("Wrong number of columns in initializer");
            }
            size_t col = 0;
            for (const T& val : row_list) {
                (*this)(row, col) = val;
                ++col;
            }
            ++row;
        }
    }

    // Initialize from flat initializer list (row-major order)
    Matrix(std::initializer_list<T> list) : data{} {
        if (list.size() != size) {
            throw std::invalid_argument("Wrong number of elements in initializer");
        }
        std::copy(list.begin(), list.end(), data.begin());
    }

    // Fill constructor
    explicit Matrix(const T& value) {
        data.fill(value);
    }

    // Copy constructor
    Matrix(const Matrix&) = default;
    Matrix& operator=(const Matrix&) = default;

    // Element access
    T& operator()(size_t row, size_t col) {
        return data[row * Cols + col];
    }

    const T& operator()(size_t row, size_t col) const {
        return data[row * Cols + col];
    }

    // Flat array access
    T& operator[](size_t index) {
        return data[index];
    }

    const T& operator[](size_t index) const {
        return data[index];
    }

    // Raw data access
    T* ptr() { return data.data(); }
    const T* ptr() const { return data.data(); }

    // Iterator support
    auto begin() { return data.begin(); }
    auto end() { return data.end(); }
    auto begin() const { return data.begin(); }
    auto end() const { return data.end(); }

    // Static factory methods
    static Matrix zero() {
        return Matrix();
    }

    static Matrix identity() {
        static_assert(Rows == Cols, "Identity matrix requires square matrix");
        Matrix result;
        for (size_t i = 0; i < Rows; ++i) {
            result(i, i) = T(1);
        }
        return result;
    }

    // Arithmetic operators
    Matrix operator+(const Matrix& other) const {
        Matrix result;
        for (size_t i = 0; i < size; ++i) {
            result[i] = data[i] + other[i];
        }
        return result;
    }

    Matrix operator+(const T& scalar) const {
        Matrix result;
        for (size_t i = 0; i < size; ++i) {
            result[i] = data[i] + scalar;
        }
        return result;
    }

    Matrix operator-(const Matrix& other) const {
        Matrix result;
        for (size_t i = 0; i < size; ++i) {
            result[i] = data[i] - other[i];
        }
        return result;
    }

    Matrix operator-(const T& scalar) const {
        Matrix result;
        for (size_t i = 0; i < size; ++i) {
            result[i] = data[i] - scalar;
        }
        return result;
    }

    Matrix operator*(const T& scalar) const {
        Matrix result;
        for (size_t i = 0; i < size; ++i) {
            result[i] = data[i] * scalar;
        }
        return result;
    }

    Matrix operator/(const T& scalar) const {
        Matrix result;
        for (size_t i = 0; i < size; ++i) {
            result[i] = data[i] / scalar;
        }
        return result;
    }

    Matrix operator-() const {
        Matrix result;
        for (size_t i = 0; i < size; ++i) {
            result[i] = -data[i];
        }
        return result;
    }

    // Compound assignment operators
    Matrix& operator+=(const Matrix& other) {
        for (size_t i = 0; i < size; ++i) {
            data[i] += other[i];
        }
        return *this;
    }

    Matrix& operator-=(const Matrix& other) {
        for (size_t i = 0; i < size; ++i) {
            data[i] -= other[i];
        }
        return *this;
    }

    Matrix& operator*=(const T& scalar) {
        for (size_t i = 0; i < size; ++i) {
            data[i] *= scalar;
        }
        return *this;
    }

    Matrix& operator/=(const T& scalar) {
        for (size_t i = 0; i < size; ++i) {
            data[i] /= scalar;
        }
        return *this;
    }

    // Matrix multiplication
    template<size_t OtherCols>
    Matrix<T, Rows, OtherCols> operator*(const Matrix<T, Cols, OtherCols>& other) const {
        Matrix<T, Rows, OtherCols> result;
        for (size_t i = 0; i < Rows; ++i) {
            for (size_t j = 0; j < OtherCols; ++j) {
                T sum = T(0);
                for (size_t k = 0; k < Cols; ++k) {
                    sum += (*this)(i, k) * other(k, j);
                }
                result(i, j) = sum;
            }
        }
        return result;
    }

    // Comparison operators
    bool operator==(const Matrix& other) const {
        return data == other.data;
    }

    bool operator!=(const Matrix& other) const {
        return !(*this == other);
    }

    // Transpose
    Matrix<T, Cols, Rows> transpose() const {
        Matrix<T, Cols, Rows> result;
        for (size_t i = 0; i < Rows; ++i) {
            for (size_t j = 0; j < Cols; ++j) {
                result(j, i) = (*this)(i, j);
            }
        }
        return result;
    }
};

// Scalar arithmetic (scalar with matrix)
template<typename T, size_t Rows, size_t Cols>
Matrix<T, Rows, Cols> operator+(const T& scalar, const Matrix<T, Rows, Cols>& matrix) {
    return matrix + scalar;
}

template<typename T, size_t Rows, size_t Cols>
Matrix<T, Rows, Cols> operator-(const T& scalar, const Matrix<T, Rows, Cols>& matrix) {
    return -matrix + scalar;
}

template<typename T, size_t Rows, size_t Cols>
Matrix<T, Rows, Cols> operator*(const T& scalar, const Matrix<T, Rows, Cols>& matrix) {
    return matrix * scalar;
}

#pragma endregion
using Mat2f = Matrix<Float, 2, 2>;
using Mat3f = Matrix<Float, 3, 3>;
using Mat4f = Matrix<Float, 4, 4>;
using Mat2i = Matrix<Int,   2, 2>;
using Mat3i = Matrix<Int,   3, 3>;
using Mat4i = Matrix<Int,   4, 4>;

#pragma region UtilityFunctions

// Dot product
template<typename T, size_t N>
inline T dot(const Vec<T, N>& a, const Vec<T, N>& b);

// Squared magnitude (length squared)
template<typename T, size_t N>
inline T lengthSquared(const Vec<T, N>& v);

// Magnitude (length)
template<typename T, size_t N>
inline T length(const Vec<T, N>& v);

// Normalize vector (unit vector)
template<typename T, size_t N>
inline Vec<T, N> normalize(const Vec<T, N>& v);

// Distance between two points
template<typename T, size_t N>
inline T distance(const Vec<T, N>& a, const Vec<T, N>& b);

// Squared distance (more efficient when you don't need the actual distance)
template<typename T, size_t N>
inline T distanceSquared(const Vec<T, N>& a, const Vec<T, N>& b);

// Cross product (3D only)
template<typename T>
inline Vec<T, 3> cross(const Vec<T, 3>& a, const Vec<T, 3>& b);

// Linear interpolation
template<typename T, size_t N>
inline Vec<T, N> lerp(const Vec<T, N>& a, const Vec<T, N>& b, T t);

// Spherical linear interpolation (for normalized vectors)
template<typename T, size_t N>
inline Vec<T, N> slerp(const Vec<T, N>& a, const Vec<T, N>& b, T t);

// Cubic interpolation (Hermite)
template<typename T, size_t N>
inline Vec<T, N> smoothstep(const Vec<T, N>& a, const Vec<T, N>& b, T t);

// Reflect vector v across normal n (for normalized normal)
template<typename T, size_t N>
inline Vec<T, N> reflect(const Vec<T, N>& v, const Vec<T, N>& n);

// Refract vector v through normal n with ratio eta
template<typename T, size_t N>
inline Vec<T, N> refract(const Vec<T, N>& v, const Vec<T, N>& n, T eta);

// Project vector a onto vector b
template<typename T, size_t N>
inline Vec<T, N> project(const Vec<T, N>& a, const Vec<T, N>& b);

// Angle between two vectors (in radians)
template<typename T, size_t N>
inline T angle(const Vec<T, N>& a, const Vec<T, N>& b) {
    T cosTheta = dot(a, b) / (length(a) * length(b));
    return std::acos(std::clamp(cosTheta, T(-1), T(1)));
}

// Signed angle between two 2D vectors (in radians)
template<typename T>
inline T signedAngle(const Vec<T, 2>& a, const Vec<T, 2>& b) {
    T cross = a.data[0] * b.data[1] - a.data[1] * b.data[0];
    T dot_product = dot(a, b);
    return std::atan2(cross, dot_product);
}

// Component-wise minimum
template<typename T, size_t N>
inline Vec<T, N> min(const Vec<T, N>& a, const Vec<T, N>& b) {
    Vec<T, N> result;
    for (size_t i = 0; i < N; i++)
        result.data[i] = std::min(a.data[i], b.data[i]);
    return result;
}

// Component-wise maximum
template<typename T, size_t N>
inline Vec<T, N> max(const Vec<T, N>& a, const Vec<T, N>& b) {
    Vec<T, N> result;
    for (size_t i = 0; i < N; i++)
        result.data[i] = std::max(a.data[i], b.data[i]);
    return result;
}

// Component-wise clamp
template<typename T, size_t N>
inline Vec<T, N> clamp(const Vec<T, N>& v, const Vec<T, N>& minVal, const Vec<T, N>& maxVal);

// Component-wise clamp with scalar bounds
template<typename T, size_t N>
inline Vec<T, N> clamp(const Vec<T, N>& v, T minVal, T maxVal);

// Component-wise absolute value
template<typename T, size_t N>
inline Vec<T, N> abs(const Vec<T, N>& v);

// Component-wise sign function
template<typename T, size_t N>
inline Vec<T, N> sign(const Vec<T, N>& v);

// Rotate 2D vector by angle (in radians)
template<typename T>
inline Vec<T, 2> rotate(const Vec<T, 2>& v, T angle);

// Check if vector is normalized (unit vector)
template<typename T, size_t N>
inline bool isNormalized(const Vec<T, N>& v);

// Check if vector is zero
template<typename T, size_t N>
inline bool isZero(const Vec<T, N>& v);

// Sum of all components
template<typename T, size_t N>
inline T sum(const Vec<T, N>& v);

// Product of all components
template<typename T, size_t N>
inline T product(const Vec<T, N>& v);

// Average of all components
template<typename T, size_t N>
inline T average(const Vec<T, N>& v);

// apply translation to the matrix (translation * matrix)
template<typename T>
Matrix<T, 4, 4> translate(const Matrix<T, 4, 4>& matrix, Vec<T, 3> vec);

// apply scale to the matrix (scale * matrix)
template<typename T>
Matrix<T, 4, 4> scale(const Matrix<T, 4, 4>& matrix, Vec<T, 3> vec);

// TODO: Sanity check!!!
// apply rotation to the matrix (rotation * matrix). angle degrees rotation.
template<typename T>
Matrix<T, 4, 4> rotate(const Matrix<T, 4, 4>& matrix, Vec<T, 3> axis, Float angle);

#pragma endregion

