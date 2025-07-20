#pragma once

#include <typeinfo>
#include <string>

std::string check_type(const auto& x)
{
	if (typeid(x) == typeid(float))
		return "Float\n";
	else if (typeid(x) == typeid(double))
		return "Double\n";
	else
		return "Unknown type\n";
}
