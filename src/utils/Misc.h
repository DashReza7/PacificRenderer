#pragma once

#include <typeinfo>
#include <string>

std::string check_type(const auto& x)
{
	if (typeid(x) == typeid(float))
		return "float\n";
	else if (typeid(x) == typeid(double))
		return "double\n";
	else
		return "unknown type\n";
}
