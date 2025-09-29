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
