#pragma once
#include <string>
#include <typeinfo>

std::string check_type(const auto& x) {
    const std::type_info& type = typeid(x);
    if (type == typeid(float))
        return "float\n";
    else if (type == typeid(double))
        return "double\n";
    else
        return "unknown type\n";
}