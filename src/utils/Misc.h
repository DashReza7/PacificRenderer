#pragma once
#include <string>
#include <typeinfo>

inline std::string check_type(const auto& x) {
    const std::type_info& type = typeid(x);
    if (type == typeid(float))
        return "float\n";
    else if (type == typeid(double))
        return "double\n";
    else
        return "unknown type\n";
}

inline std::string trim(const std::string& s) {
    size_t start = 0;
    while (start < s.size() && std::isspace(static_cast<unsigned char>(s[start]))) {
        ++start;
    }

    size_t end = s.size();
    while (end > start && std::isspace(static_cast<unsigned char>(s[end - 1]))) {
        --end;
    }

    return s.substr(start, end - start);
}

inline Vec3f strToVec3f(const std::string& value_str) {
    Vec3f vec;
    std::istringstream iss(value_str);
    char comma;
    iss >> vec.x >> comma >> vec.y >> comma >> vec.z;
    return vec;
}

inline Mat4f strToMat4f(std::string mat_str) {
    Mat4f mat;
    std::istringstream iss(mat_str);

    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            iss >> mat[i][j];

    return mat;
}

inline std::string mat4fToStr(Mat4f mat) {
    std::ostringstream oss;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            oss << mat[i][j];
            if (i < 3 || j < 3) oss << " ";
        }
    }
    return oss.str();
}

/// Print a 4x4 matrix
inline void print_matrix(const Mat4f& mat) {
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            std::cout << mat[j][i] << " "; // Note the swapped indices for column-major order in GLM
        }
        std::cout << std::endl;
    }
}

