#include "core/MathUtils.h"

template<typename T, size_t N>
inline T dot(const Vec<T, N>& a, const Vec<T, N>& b) {
    T result = T(0);
    for (size_t i = 0; i < N; i++)
        result += a[i] * b[i];
    return result;
}

template<typename T, size_t N>
inline T lengthSquared(const Vec<T, N>& v) {
    return dot(v, v);
}

template<typename T, size_t N>
inline T length(const Vec<T, N>& v) {
    return std::sqrt(lengthSquared(v));
}

template<typename T, size_t N>
inline Vec<T, N> normalize(const Vec<T, N>& v) {
    T len = length(v);
    if (len < Epsilon)
        return Vec<T, N>(T(0)); // Return zero vector if too small
    return v / len;
}

template<typename T, size_t N>
inline T distance(const Vec<T, N>& a, const Vec<T, N>& b) {
    return length(a - b);
}

template<typename T, size_t N>
inline T distanceSquared(const Vec<T, N>& a, const Vec<T, N>& b) {
    return lengthSquared(a - b);
}

template<typename T>
inline Vec<T, 3> cross(const Vec<T, 3>& a, const Vec<T, 3>& b) {
    return Vec<T, 3>(
        a.data[1] * b.data[2] - a.data[2] * b.data[1],
        a.data[2] * b.data[0] - a.data[0] * b.data[2],
        a.data[0] * b.data[1] - a.data[1] * b.data[0]
    );
}

template<typename T, size_t N>
inline Vec<T, N> lerp(const Vec<T, N>& a, const Vec<T, N>& b, T t) {
    return a + (b - a) * t;
}

// TODO: Sanity check
template<typename T, size_t N>
inline Vec<T, N> slerp(const Vec<T, N>& a, const Vec<T, N>& b, T t) {
    T cosTheta = dot(a, b);

    // If vectors are nearly parallel, use linear interpolation
    if (std::abs(cosTheta) > T(1) - Epsilon) {
        return normalize(lerp(a, b, t));
    }

    T theta = std::acos(std::clamp(cosTheta, T(-1), T(1)));
    T sinTheta = std::sin(theta);

    T wa = std::sin((T(1) - t) * theta) / sinTheta;
    T wb = std::sin(t * theta) / sinTheta;

    return a * wa + b * wb;
}

template<typename T, size_t N>
inline Vec<T, N> smoothstep(const Vec<T, N>& a, const Vec<T, N>& b, T t) {
    t = std::clamp(t, T(0), T(1));
    T smoothT = t * t * (T(3) - T(2) * t);
    return lerp(a, b, smoothT);
}

// TODO: Sanity check
template<typename T, size_t N>
inline Vec<T, N> reflect(const Vec<T, N>& v, const Vec<T, N>& n) {
    return v - n * (T(2) * dot(v, n));
}

// TODO: Sanity check
template<typename T, size_t N>
inline Vec<T, N> refract(const Vec<T, N>& v, const Vec<T, N>& n, T eta) {
    // TODO: Sanity check!!!
    T cosI = -dot(n, v);
    T sinT2 = eta * eta * (T(1) - cosI * cosI);

    if (sinT2 >= T(1))
        return Vec<T, N>(T(0)); // Total internal reflection

    T cosT = std::sqrt(T(1) - sinT2);
    return v * eta + n * (eta * cosI - cosT);
}

template<typename T, size_t N>
inline Vec<T, N> project(const Vec<T, N>& a, const Vec<T, N>& b) {
    T bLengthSq = lengthSquared(b);
    if (bLengthSq < Epsilon)
        return Vec<T, N>(T(0));
    return b * (dot(a, b) / bLengthSq);
}

template<typename T, size_t N>
inline Vec<T, N> clamp(const Vec<T, N>& v, const Vec<T, N>& minVal, const Vec<T, N>& maxVal) {
    Vec<T, N> result;
    for (size_t i = 0; i < N; i++)
        result.data[i] = std::clamp(v.data[i], minVal.data[i], maxVal.data[i]);
    return result;
}

template<typename T, size_t N>
inline Vec<T, N> clamp(const Vec<T, N>& v, T minVal, T maxVal) {
    Vec<T, N> result;
    for (size_t i = 0; i < N; i++)
        result.data[i] = std::clamp(v.data[i], minVal, maxVal);
    return result;
}

template<typename T, size_t N>
inline Vec<T, N> abs(const Vec<T, N>& v) {
    Vec<T, N> result;
    for (size_t i = 0; i < N; i++)
        result.data[i] = std::abs(v.data[i]);
    return result;
}

template<typename T, size_t N>
inline Vec<T, N> sign(const Vec<T, N>& v) {
    Vec<T, N> result;
    for (size_t i = 0; i < N; i++) {
        if (v.data[i] > Epsilon)
            result.data[i] = T(1);
        else if (v.data[i] < -Epsilon)
            result.data[i] = T(-1);
        else
            result.data[i] = T(0);
    }
    return result;
}

template<typename T>
inline Vec<T, 2> rotate(const Vec<T, 2>& v, T angle) {
    T cosA = std::cos(angle);
    T sinA = std::sin(angle);
    return Vec<T, 2>(
        v.data[0] * cosA - v.data[1] * sinA,
        v.data[0] * sinA + v.data[1] * cosA
    );
}

template<typename T, size_t N>
inline bool isNormalized(const Vec<T, N>& v) {
    T len = length(v);
    return std::abs(len - T(1)) < Epsilon;
}

template<typename T, size_t N>
inline bool isZero(const Vec<T, N>& v) {
    if constexpr (std::is_floating_point_v<T>) {
        for (size_t i = 0; i < N; i++)
            if (std::abs(v.data[i]) >= Epsilon)
                return false;
        return true;
    }
    else {
        for (size_t i = 0; i < N; i++)
            if (v.data[i] != 0)
                return false;
        return true;
    }
}

template<typename T, size_t N>
inline T sum(const Vec<T, N>& v) {
    T result = T(0);
    for (size_t i = 0; i < N; i++)
        result += v.data[i];
    return result;
}

template<typename T, size_t N>
inline T product(const Vec<T, N>& v) {
    T result = T(1);
    for (size_t i = 0; i < N; i++)
        result *= v.data[i];
    return result;
}

template<typename T, size_t N>
inline T average(const Vec<T, N>& v) {
    return sum(v) / T(N);
}

template<typename T>
Matrix<T, 4, 4> translate(const Matrix<T, 4, 4>& matrix, Vec<T, 3> vec) {
    auto transform = Matrix<T, 4, 4>({ {T(1), T(0), T(0), vec.x()},
                                                {T(0), T(1), T(0), vec.y()},
                                                {T(0), T(0), T(1), vec.z()},
                                                {T(0), T(0), T(0), T(1) } });
    return transform * matrix;
}

template<typename T>
Matrix<T, 4, 4> scale(const Matrix<T, 4, 4>& matrix, Vec<T, 3> vec) {
    auto transform = Matrix<T, 4, 4>({ {vec.x(), T(0), T(0), T(0)},
                                                {T(0), vec.y(), T(0), T(0)},
                                                {T(0), T(0), vec.z(), T(0)},
                                                {T(0), T(0), T(0), T(1) } });
    return transform * matrix;
}

template<typename T>
Matrix<T, 4, 4> rotate(const Matrix<T, 4, 4>& matrix, Vec<T, 3> axis, Float angle) {
    // Convert angle from degrees to radians
    T radians = angle * T(Pi) / T(180.0);

    // Normalize the axis vector
    Vec<T, 3> normalizedAxis = normalize(axis);
    T x = normalizedAxis.x();
    T y = normalizedAxis.y();
    T z = normalizedAxis.z();

    // Calculate sine and cosine
    T c = cos(radians);
    T s = sin(radians);
    T oneMinusC = T(1) - c;

    // Create rotation matrix using Rodrigues' rotation formula
    Matrix<T, 4, 4> rotation = Matrix<T, 4, 4>::identity();

    // First row
    rotation(0, 0) = c + x * x * oneMinusC;
    rotation(0, 1) = x * y * oneMinusC - z * s;
    rotation(0, 2) = x * z * oneMinusC + y * s;

    // Second row
    rotation(1, 0) = y * x * oneMinusC + z * s;
    rotation(1, 1) = c + y * y * oneMinusC;
    rotation(1, 2) = y * z * oneMinusC - x * s;

    // Third row
    rotation(2, 0) = z * x * oneMinusC - y * s;
    rotation(2, 1) = z * y * oneMinusC + x * s;
    rotation(2, 2) = c + z * z * oneMinusC;

    // Fourth row and column remain as identity (0, 0, 0, 1)

    return rotation * matrix;
}


